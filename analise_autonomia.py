#!/usr/bin/env python3
"""Confere, trecho a trecho, se cada VANT respeita a autonomia (maxFuel) entre recargas.

Na formulação de Santin (2021), cada aresta custa (1 + q_k) * d_ij / v_k, e os nós com
índice maior que o número de alvos são estações de recarga (cópias dos alvos). O VANT
recarrega ao chegar numa estação ou na base. Aqui se compara o tempo de voo entre
recargas com F_k (maxFuel) em duas geometrias:
  - solver : comprimento implícito no custo do arquivo path (d = f * v / (1 + q));
  - sim    : comprimento das rotas recalculadas com os obstáculos expandidos.

Uso: python3 analise_autonomia.py   (gera autonomia.csv e imprime o resumo)
"""
import csv
import glob
import json
import math
import os
import re

from shapely.geometry import Polygon

import build_scenario as bs
from cenarios import CENARIOS, DRONE_SIZE, ROBOT_OFFSET


def leg_route(geo, P, a, b, nt):
    pa, pb = bs.phys(a, nt), bs.phys(b, nt)
    if pa == pb:
        return [P[pa]]
    lo, hi = sorted((pa, pb))
    if lo >= 1 and lo % 2 == 1 and hi == lo + 1:
        seg = geo.line_route(P[lo], P[hi])
        return seg[::-1] if pa > pb else seg
    return geo.shortest(P[pa], P[pb])


def length(pts):
    return sum(math.dist(p, q) for p, q in zip(pts[:-1], pts[1:]))


def analisar(num, margin_sim=None):
    c = CENARIOS[num]
    d = c["dir"]
    base, targets, cfgs, robots = bs.read_professor_file(os.path.join(d, c["input"]))
    nt = len(targets)
    P = {0: base, **{i + 1: t for i, t in enumerate(targets)}}
    obs = json.load(open(os.path.join(d, "obstacles.json")))
    m_sim = 2 * DRONE_SIZE if margin_sim is None else margin_sim
    geo = bs.Geometry([Polygon(v) for v in obs.values()], m_sim / c["scale"])
    rows = []
    for fn in sorted(glob.glob(os.path.join(d, "path_*.txt")), key=lambda f: int(re.search(r"path_(\d+)", f).group(1))):
        n = int(re.search(r"path_(\d+)", fn).group(1))
        rid, cost, legs = bs.read_path(fn)
        cfg_id = robots[f"Robot_{rid + ROBOT_OFFSET}"]
        v, F, q = cfgs[cfg_id]["maxVel"], cfgs[cfg_id]["maxFuel"], cfgs[cfg_id]["prop"]
        seg_solver = seg_sim = 0.0
        trechos = []          # (voo_solver_s, voo_sim_s)
        tot_solver = tot_sim = 0.0
        recargas = 0
        for k, (a, b, f) in enumerate(legs):
            ls = f * v / (1 + q)
            lr = length(leg_route(geo, P, a, b, nt))
            seg_solver += ls; seg_sim += lr
            tot_solver += ls; tot_sim += lr
            is_station = b == 0 or b > nt
            if is_station and (seg_solver > 0 or seg_sim > 0):
                trechos.append((seg_solver / v, seg_sim / v))
                seg_solver = seg_sim = 0.0
                if b != 0:
                    recargas += 1
        voo_solver, voo_sim = tot_solver / v, tot_sim / v
        rows.append({
            "cenario": num, "vant": f"drone_{n}", "config": cfg_id, "v": v, "F": F, "q": q,
            "custo_solver_s": cost, "voo_solver_s": voo_solver, "voo_sim_s": voo_sim,
            "missao_prevista_sim_s": (1 + q) * voo_sim,
            "comprimento_solver_km": tot_solver / 1000, "comprimento_sim_km": tot_sim / 1000,
            "acrescimo_pct": 100 * (tot_sim / tot_solver - 1),
            "recargas_intermediarias": recargas, "trechos": len(trechos),
            "maior_trecho_solver_s": max(t[0] for t in trechos),
            "maior_trecho_sim_s": max(t[1] for t in trechos),
            "trechos_acima_F_sim": sum(1 for t in trechos if t[1] > F + 1e-6),
            "trechos_acima_F_solver": sum(1 for t in trechos if t[0] > F + 1e-6),
        })
    return rows


def main():
    allrows = []
    for num in CENARIOS:
        allrows += analisar(num)
    with open("autonomia.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(allrows[0].keys()))
        w.writeheader(); w.writerows(allrows)
    print("cen VANT     cfg       F(s) recargas  maior trecho solver/sim (s)  >F(sim)  compr. solver/sim (km)  +%   custo   (1+q)voo_sim")
    for r in allrows:
        print(f"{r['cenario']:>3} {r['vant']:<8} {r['config']:<9}{r['F']:>5.0f} {r['recargas_intermediarias']:>6}"
              f"      {r['maior_trecho_solver_s']:7.0f} / {r['maior_trecho_sim_s']:7.0f}        {r['trechos_acima_F_sim']:>3}"
              f"      {r['comprimento_solver_km']:6.1f} / {r['comprimento_sim_km']:6.1f}   {r['acrescimo_pct']:4.1f} "
              f"{r['custo_solver_s']:8.0f} {r['missao_prevista_sim_s']:8.0f}")


if __name__ == "__main__":
    main()
