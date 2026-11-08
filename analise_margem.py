#!/usr/bin/env python3
"""Sensibilidade à margem de segurança: comprimento total, maior acréscimo por VANT e
trechos entre recargas que passam da autonomia (maxFuel), para margens de 0 a 2 m (simulação).

Uso: python3 analise_margem.py  -> margem.csv
"""
import csv
from analise_autonomia import analisar
from cenarios import CENARIOS

linhas = []
print("cen  margem sim (m)  margem real (m)  compr. total (km)  +% vs solver  maior +% VANT  trechos > autonomia")
for n, c in CENARIOS.items():
    for m in (0.0, 0.5, 1.0, 1.5, 2.0):
        rows = analisar(n, margin_sim=m)
        tot_sim = sum(r['comprimento_sim_km'] for r in rows)
        tot_sol = sum(r['comprimento_solver_km'] for r in rows)
        acima = sum(r['trechos_acima_F_sim'] for r in rows)
        maior = max(r['acrescimo_pct'] for r in rows)
        linhas.append({'cenario': n, 'margem_sim_m': m, 'margem_real_m': m / c['scale'],
                       'comprimento_total_km': round(tot_sim, 2), 'acrescimo_vs_solver_pct': round(100 * (tot_sim / tot_sol - 1), 2),
                       'maior_acrescimo_vant_pct': round(maior, 2), 'trechos_acima_autonomia': acima})
        l = linhas[-1]
        print(f"{n:>3}  {m:>13.1f}  {l['margem_real_m']:>15.0f}  {tot_sim:>17.1f}  {l['acrescimo_vs_solver_pct']:>12.2f}  {maior:>13.2f}  {acima:>19}")
with open('margem.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(linhas[0].keys())); w.writeheader(); w.writerows(linhas)
