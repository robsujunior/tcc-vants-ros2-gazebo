#!/usr/bin/env python3
"""
Gera a simulação a partir dos arquivos enviados pelo professor.

Entradas:
  - D_T30_R6_preprocessing.txt  (base, alvos, configuração dos robôs)
  - path_0.txt ... path_N.txt   (rota de cada VANT, em índices de nós)
  - obstacles_*.json            (polígonos dos obstáculos, em metros)

Saídas (na pasta --out):
  - routes.json  waypoints de cada VANT (coordenadas da simulação)
  - world.sdf    mundo do Gazebo (solo, obstáculos reais, VANTs)
  - bridge.sh    comando do ros_gz_bridge para todos os VANTs
"""
import argparse
import glob
import itertools
import json
import math
import os
import re
import sys
import time

import networkx as nx
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

PALETTE = [(0.90, 0.20, 0.20), (0.20, 0.45, 0.90), (0.15, 0.70, 0.30),
           (0.95, 0.75, 0.10), (0.65, 0.30, 0.80), (0.95, 0.50, 0.15)]


# ----------------------------------------------------------------- entrada
def parse_sections(text):
    sections, cur = {}, None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith('@'):
            cur = line[1:]
            sections[cur] = []
        elif cur is not None:
            sections[cur].append(line)
    return sections


def read_professor_file(fn):
    sec = parse_sections(open(fn, encoding='utf-8').read())
    m = re.match(r'(\S+)@(\S+)\s+(\S+)', sec['base'][0])
    base = (float(m.group(2)), float(m.group(3)))
    targets = [tuple(map(float, l.split())) for l in sec['target']]

    cfgs, cur = {}, None
    for l in sec['robots_configuration']:
        k, v = l.split(':', 1)
        if k == 'configId':
            cur = v
            cfgs[cur] = {}
        else:
            cfgs[cur][k] = float(v)
    robots, cur = {}, None
    for l in sec['robot']:
        k, v = l.split(':', 1)
        if k == 'robotId':
            cur = v
        elif k == 'configId':
            robots[cur] = v
    return base, targets, cfgs, robots


def read_path(fn):
    lines = [l.strip() for l in open(fn, encoding='utf-8') if l.strip()]
    hdr = lines[1].split(';')
    legs = []
    for l in lines[3:]:
        a, b, c = l.split(';')[:3]
        legs.append((int(a), int(b), float(c)))
    return int(hdr[2]), float(hdr[1]), legs


def phys(n, n_targets):
    """Mesma regra do postprocessor.py: ids > n_targets são cópias (depósitos)."""
    if n == 0:
        return 0
    if n <= n_targets:
        return n
    r = n % n_targets
    return n_targets if r == 0 else r


# ---------------------------------------------------------------- geometria
def dedupe(pts, eps=1e-6):
    out = []
    for p in pts:
        if not out or math.dist(out[-1], p) > eps:
            out.append((float(p[0]), float(p[1])))
    return out


class Geometry:
    def __init__(self, polys_true, margin):
        self.true = unary_union(polys_true)
        self.infl = unary_union([p.buffer(margin, join_style=2) for p in polys_true])
        self.shr = self.infl.buffer(-1e-6)
        geoms = list(self.infl.geoms) if self.infl.geom_type == 'MultiPolygon' else [self.infl]
        self.verts = [tuple(c) for g in geoms for c in g.exterior.coords[:-1]]
        self._cache = {}

    def free(self, a, b):
        return not LineString([a, b]).intersects(self.shr)

    def shortest(self, a, b):
        key = (round(a[0], 6), round(a[1], 6), round(b[0], 6), round(b[1], 6))
        if key in self._cache:
            return list(self._cache[key])
        if self.free(a, b):
            path = [a, b]
        else:
            nodes = [a, b] + self.verts
            g = nx.Graph()
            g.add_nodes_from(range(len(nodes)))
            for i, j in itertools.combinations(range(len(nodes)), 2):
                if self.free(nodes[i], nodes[j]):
                    g.add_edge(i, j, weight=math.dist(nodes[i], nodes[j]))
            try:
                idx = nx.shortest_path(g, 0, 1, weight='weight')
            except nx.NetworkXNoPath:
                raise RuntimeError(f"Sem caminho livre entre {a} e {b}")
            path = [nodes[i] for i in idx]
        self._cache[key] = path
        return list(path)

    def line_route(self, a, b):
        """Percorre a linha de cobertura e contorna só os trechos bloqueados."""
        seg = LineString([a, b])
        inter = self.infl.intersection(seg)
        if inter.is_empty:
            return [a, b]
        parts = [inter] if inter.geom_type == 'LineString' else \
            [g for g in getattr(inter, 'geoms', []) if g.geom_type == 'LineString']
        parts = [g for g in parts if g.length > 1e-6]
        if not parts:
            return [a, b]
        parts.sort(key=lambda g: seg.project(Point(g.coords[0])))
        pts = [a]
        for g in parts:
            p1, p2 = g.coords[0], g.coords[-1]
            if seg.project(Point(p1)) > seg.project(Point(p2)):
                p1, p2 = p2, p1
            pts += self.shortest(p1, p2)
        pts.append(b)
        return dedupe(pts)


def build_route(geo, P, legs, n_targets, speed_real=None, q=0.0, speedup=1.0):
    """Monta a rota em pontos de passagem e marca as paradas de recarga.

    Na formulação de Santin (2021), nós com índice > n_targets são estações de recarga
    (cópias dos alvos). Ao chegar numa estação o VANT recarrega por q * (tempo de voo
    desde a última recarga). Retorna (pontos, paradas), onde paradas é uma lista de
    dicionários {wp, dur_real, dur_sim, voo_real}. A chegada final à base não gera parada.
    """
    pts = [P[0]]
    stops = []
    flown = 0.0
    for k, (a, b, _) in enumerate(legs):
        pa, pb = phys(a, n_targets), phys(b, n_targets)
        if pa != pb:
            lo, hi = sorted((pa, pb))
            if lo >= 1 and lo % 2 == 1 and hi == lo + 1:      # extremidades da mesma linha
                seg = geo.line_route(P[lo], P[hi])
                if pa > pb:
                    seg = seg[::-1]
            else:
                seg = geo.shortest(P[pa], P[pb])
            if math.dist(pts[-1], seg[0]) > 1e-3:
                print(f"  aviso: descontinuidade antes do trecho {a}->{b}", file=sys.stderr)
            for p in seg:
                if math.dist(pts[-1], p) > 1e-6:
                    flown += math.dist(pts[-1], p)
                    pts.append((float(p[0]), float(p[1])))
        is_station = b > n_targets
        last = k == len(legs) - 1
        if is_station and not last and speed_real and flown > 0:
            t_voo = flown / speed_real
            dur = q * t_voo
            stops.append({"wp": len(pts) - 1, "node": b, "voo_real": t_voo,
                          "dur_real": dur, "dur_sim": dur / speedup})
            flown = 0.0
    return pts, stops


# ---------------------------------------------------------------------- SDF
def drone_sdf(name, x, y, z, D, color, ):
    r, g, b = color
    arms = "".join(
        f"""
        <visual name="rotor{i}"><pose>{sx*0.45*D:.4f} {sy*0.45*D:.4f} 0 0 0 0</pose>
          <geometry><cylinder><radius>{0.18*D:.4f}</radius><length>{0.06*D:.4f}</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>"""
        for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (1, -1), (-1, -1)]))
    return f"""
    <model name="{name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <visual name="body">
          <geometry><box><size>{D:.4f} {D:.4f} {0.12*D:.4f}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>{arms}
        <collision name="collision">
          <geometry><box><size>{D:.4f} {D:.4f} {0.12*D:.4f}</size></box></geometry>
        </collision>
      </link>
      <plugin filename="libignition-gazebo-velocity-control-system.so" name="ignition::gazebo::systems::VelocityControl">
        <topic>/model/{name}/cmd_vel</topic>
      </plugin>
      <plugin filename="libignition-gazebo-odometry-publisher-system.so" name="ignition::gazebo::systems::OdometryPublisher">
        <odom_topic>/model/{name}/odometry</odom_topic>
        <dimensions>3</dimensions>
      </plugin>
    </model>"""


def obstacle_sdf(i, poly_sim, height):
    pts = "\n".join(f"              <point>{x:.4f} {y:.4f}</point>" for x, y in poly_sim)
    geom = f"""<geometry><polyline>
{pts}
              <height>{height}</height>
            </polyline></geometry>"""
    return f"""
    <model name="obstacle_{i}">
      <static>true</static>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}
          <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def grid_sdf(bbox, step, width=0.2, z=0.02):
    """Grade desenhada no próprio mundo (só visual, sem colisão), alinhada ao chão."""
    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    vis, k, x = [], 0, x0
    while x <= x1 + 1e-6:
        vis.append(f'<visual name="gx{k}"><pose>{x:.3f} {cy:.3f} {z} 0 0 0</pose><geometry><box><size>{width} {y1-y0:.3f} 0.02</size></box></geometry>'
                   '<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>')
        x += step; k += 1
    k, y = 0, y0
    while y <= y1 + 1e-6:
        vis.append(f'<visual name="gy{k}"><pose>{cx:.3f} {y:.3f} {z} 0 0 0</pose><geometry><box><size>{x1-x0:.3f} {width} 0.02</size></box></geometry>'
                   '<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>')
        y += step; k += 1
    return ('<model name="grid"><static>true</static><link name="link">'
            + "".join(vis) + '</link></model>')


def world_sdf(drones_xml, obstacles_xml, bbox, grid_xml=""):
    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="drone_world">
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"></plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>{cx:.1f} {cy:.1f} 200 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>{cx:.2f} {cy:.2f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>{x1-x0:.1f} {y1-y0:.1f}</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{x1-x0:.1f} {y1-y0:.1f}</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
{grid_xml}
{obstacles_xml}
{drones_xml}
  </world>
</sdf>
"""


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', default='D_T30_R6_preprocessing.txt')
    ap.add_argument('--paths-dir', default='.')
    ap.add_argument('--obstacles', default='obstacles_provisorio.json')
    ap.add_argument('--out', default='out')
    ap.add_argument('--scale', type=float, default=0.01, help='metros reais -> metros da simulação')
    ap.add_argument('--speedup', type=float, default=20.0, help='fator de aceleração do tempo da missão')
    ap.add_argument('--drone-size', type=float, default=1.0, help='lado do corpo do VANT na simulação (m)')
    ap.add_argument('--margin-sim', type=float, default=None, help='margem de segurança na simulação (m); padrão = 2 x tamanho do VANT')
    ap.add_argument('--obstacle-height', type=float, default=12.0)
    ap.add_argument('--alt0', type=float, default=1.5, help='altitude do VANT 0 (m)')
    ap.add_argument('--dalt', type=float, default=0.6, help='diferença de altitude entre VANTs (m)')
    ap.add_argument('--grid-step', type=float, default=10.0,
                    help='espaçamento da grade desenhada no mundo (m da simulação); 0 desliga')
    ap.add_argument('--sem-recarga', action='store_true',
                    help='não para nas estações de recarga (só voo)')
    ap.add_argument('--robot-offset', type=int, default=1,
                    help='Robot_ID do path N corresponde a Robot_(N+offset) no arquivo do professor (HIPÓTESE a confirmar)')
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    base, targets, cfgs, robots = read_professor_file(a.input)
    n_targets = len(targets)
    P = {0: base}
    P.update({i + 1: t for i, t in enumerate(targets)})

    obs_raw = json.load(open(a.obstacles))
    polys_true = [Polygon(v) for v in obs_raw.values()]
    margin_sim = a.margin_sim if a.margin_sim is not None else 2.0 * a.drone_size
    margin_real = margin_sim / a.scale
    geo = Geometry(polys_true, margin_real)

    xs = [p[0] for p in P.values()] + [x for v in obs_raw.values() for x, _ in v]
    ys = [p[1] for p in P.values()] + [y for v in obs_raw.values() for _, y in v]
    extent = max(max(xs) - min(xs), max(ys) - min(ys))
    suggested = 240.0 / extent
    if abs(math.log10(suggested / a.scale)) > 0.3:
        print(f"ATENÇÃO: extensão do cenário = {extent:.0f} m; com --scale {a.scale} fica {extent*a.scale:.0f} m. "
              f"Para ~240 m use --scale {suggested:.4g}", file=sys.stderr)
    print(f"Alvos: {n_targets} | base: {base} | obstáculos: {len(polys_true)}")
    print(f"Escala {a.scale} | margem de segurança: {margin_sim} m (sim) = {margin_real:.0f} m (real) | aceleração x{a.speedup}")
    for tid, p in P.items():
        if Point(p).distance(geo.infl) < 1e-6 or geo.infl.contains(Point(p)):
            print(f"  ATENÇÃO: ponto {tid} {p} está dentro da margem de segurança de um obstáculo!")

    path_files = sorted(glob.glob(os.path.join(a.paths_dir, 'path_*.txt')),
                        key=lambda f: int(re.search(r'path_(\d+)', f).group(1)))
    drones, sdf_drones = [], ""
    build_time = 0.0
    true_shr = geo.true.buffer(-1e-6)
    print("\nVANT    robô  config    pontos  comprimento real  custo(solver)  colisões  folga mín. (real)")
    for fn in path_files:
        n = int(re.search(r'path_(\d+)', fn).group(1))
        rid, solver_cost, legs = read_path(fn)
        cfg_id = robots.get(f"Robot_{rid + a.robot_offset}")
        cfg = cfgs[cfg_id]
        _t = time.perf_counter()
        route, stops = build_route(geo, P, legs, n_targets, cfg['maxVel'], cfg['prop'], a.speedup)
        build_time += time.perf_counter() - _t
        if a.sem_recarga:
            stops = []

        segs = list(zip(route[:-1], route[1:]))
        length = sum(math.dist(p, q) for p, q in segs)
        collisions = sum(1 for p, q in segs if LineString([p, q]).intersects(true_shr))
        clearance = min(LineString([p, q]).distance(geo.true) for p, q in segs)

        speed = cfg['maxVel'] * a.scale * a.speedup
        tol = max(0.3 * a.drone_size, 1.5 * speed * 0.1)
        alt = a.alt0 + a.dalt * n
        spawn = (base[0] * a.scale, base[1] * a.scale, alt)
        name = f"drone_{n}"
        drones.append({
            "name": name, "path_file": os.path.basename(fn), "robot_id": rid, "config": cfg_id,
            "max_vel_real": cfg['maxVel'], "max_fuel_s": cfg['maxFuel'], "prop": cfg['prop'], "speed": speed, "tolerance": tol, "altitude": alt,
            "spawn": list(spawn), "solver_cost": solver_cost,
            "real_length_m": length, "collisions": collisions, "min_clearance_real_m": clearance,
            "waypoints": [[x * a.scale, y * a.scale] for x, y in route],
            "stops": stops,
        })
        sdf_drones += drone_sdf(name, spawn[0], spawn[1], spawn[2], a.drone_size, PALETTE[n % len(PALETTE)])
        print(f"{name:<7} {rid:>4}  {cfg_id:<8} {len(route):>6}  {length/1000:>12.1f} km  {solver_cost:>12.1f}  {collisions:>8}  {clearance:>10.0f} m   recargas: {len(stops)}")

    obs_sim = [[[x * a.scale, y * a.scale] for x, y in v] for v in obs_raw.values()]
    sdf_obs = "".join(obstacle_sdf(i + 1, v, a.obstacle_height) for i, v in enumerate(obs_sim))

    allx = [p[0] for d in drones for p in d["waypoints"]] + [x for v in obs_sim for x, _ in v]
    ally = [p[1] for d in drones for p in d["waypoints"]] + [y for v in obs_sim for _, y in v]
    pad = 40.0
    bbox = (min(allx) - pad, min(ally) - pad, max(allx) + pad, max(ally) + pad)
    grid_xml = ""
    if a.grid_step > 0:
        g = a.grid_step
        bbox = (math.floor(bbox[0] / g) * g, math.floor(bbox[1] / g) * g,
                math.ceil(bbox[2] / g) * g, math.ceil(bbox[3] / g) * g)
        grid_xml = grid_sdf(bbox, g)

    json.dump({"cenario_input": os.path.basename(a.input), "recarga": not a.sem_recarga, "n_targets": n_targets, "n_obstacles": len(polys_true), "route_build_seconds": build_time, "scale": a.scale, "speedup": a.speedup, "drone_size": a.drone_size,
               "margin_sim": margin_sim, "obstacle_height": a.obstacle_height,
               "obstacles_sim": obs_sim, "bbox": bbox, "drones": drones},
              open(os.path.join(a.out, 'routes.json'), 'w'), indent=1)
    open(os.path.join(a.out, 'world.sdf'), 'w', encoding='utf-8').write(world_sdf(sdf_drones, sdf_obs, bbox, grid_xml))

    bridge = ["#!/bin/bash", "source /opt/ros/humble/setup.bash", "ros2 run ros_gz_bridge parameter_bridge \\"]
    for d in drones:
        bridge.append(f"  '/model/{d['name']}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \\")
        bridge.append(f"  '/model/{d['name']}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \\")
    bridge[-1] = bridge[-1].rstrip(' \\')
    open(os.path.join(a.out, 'bridge.sh'), 'w').write("\n".join(bridge) + "\n")
    os.chmod(os.path.join(a.out, 'bridge.sh'), 0o755)
    print(f"\nArquivos gerados em '{a.out}': routes.json, world.sdf, bridge.sh")


if __name__ == '__main__':
    main()
