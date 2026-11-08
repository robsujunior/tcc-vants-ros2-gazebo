#!/usr/bin/env python3
"""Teste sem Gazebo: simula o mesmo controle (ponto de massa, laço de controle a 10 Hz)
e confere conclusão das rotas, duração e folga em relação aos obstáculos reais.

Serve também de referência cinemática para comparar com o Gazebo: com --csv grava
arquivos no mesmo formato do registrador.py (<prefixo>_odom.csv e <prefixo>_eventos.csv).

Uso:  python3 dry_run.py out/routes.json [--hz 10] [--csv resultados/c1_cinematico]
"""
import argparse
import csv
import json
import math
import os

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from follower_logic import RouteFollower


def simulate(cfg, hz=10.0, sim_dt=0.01, prefixo=None, relativa=False, t_max=6 * 3600):
    """Integra o movimento com passo sim_dt; o controlador só atualiza o comando a cada 1/hz s."""
    obst = unary_union([Polygon(v) for v in cfg['obstacles_sim']])
    half_diag = cfg['drone_size'] * math.sqrt(2) / 2
    ctrl_every = max(1, round((1.0 / hz) / sim_dt))
    w_odom = w_ev = None
    if prefixo:
        os.makedirs(os.path.dirname(prefixo) or '.', exist_ok=True)
        f_odom = open(prefixo + '_odom.csv', 'w', newline='')
        f_ev = open(prefixo + '_eventos.csv', 'w', newline='')
        w_odom, w_ev = csv.writer(f_odom), csv.writer(f_ev)
        w_odom.writerow(['t_wall', 't_sim', 'drone', 'x', 'y', 'z'])
        w_ev.writerow(['t_wall', 't_sim', 'drone', 'evento', 'wp'])
    rows = []
    for d in cfg['drones']:
        f = RouteFollower(d['waypoints'], d['spawn'], d['speed'], d['altitude'], d['tolerance'], d.get('stops', []))
        pos = list(d['spawn'])
        t, k, min_clear, contact = 0.0, 0, float('inf'), 0
        cmd = (0.0, 0.0, 0.0)
        if w_ev:
            w_ev.writerow([f'{t:.3f}', f'{t:.3f}', d['name'], 'inicio', 0])
        while not f.done and t < t_max:
            if k % ctrl_every == 0:
                odom = tuple(p - s for p, s in zip(pos, d['spawn'])) if relativa else tuple(pos)
                cmd = f.step(odom, t)
                if w_ev:
                    for kind, idx in f.events:
                        w_ev.writerow([f'{t:.3f}', f'{t:.3f}', d['name'], kind, idx])
                if w_odom and k % round(0.1 / sim_dt) == 0:
                    w_odom.writerow([f'{t:.3f}', f'{t:.3f}', d['name'], f'{pos[0]:.4f}', f'{pos[1]:.4f}', f'{pos[2]:.4f}'])
            for j in range(3):
                pos[j] += cmd[j] * sim_dt
            t += sim_dt
            k += 1
            c = Point(pos[0], pos[1]).distance(obst)
            min_clear = min(min_clear, c)
            if c < half_diag:
                contact += 1
        rows.append({"name": d['name'], "duration": t, "reached": f.i, "total": len(f.wp),
                     "min_clearance": min_clear, "contact": contact, "done": f.done})
    if prefixo:
        w_ev.writerow([f'{t:.3f}', f'{max(r["duration"] for r in rows):.3f}', 'todos', 'fim', -1])
        f_odom.close(); f_ev.close()
    return rows, half_diag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('routes')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--csv', default=None)
    ap.add_argument('--odom-relativa', action='store_true')
    a = ap.parse_args()
    cfg = json.load(open(a.routes))
    rows, half_diag = simulate(cfg, a.hz, prefixo=a.csv, relativa=a.odom_relativa)
    print(f"Laço de controle a {a.hz:g} Hz | meia diagonal do VANT: {half_diag:.2f} m\n")
    print("VANT     duração    waypoints   folga mín. ao obstáculo   passos com contato")
    for r in rows:
        status = "ok" if r['done'] else "NÃO CONCLUIU"
        print(f"{r['name']:<8} {r['duration']:7.0f} s   {r['reached']:>3}/{r['total']:<3}      "
              f"{r['min_clearance']:6.2f} m                {r['contact']:>4}   [{status}]")


if __name__ == '__main__':
    main()
