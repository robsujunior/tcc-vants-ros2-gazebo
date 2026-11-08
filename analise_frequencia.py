#!/usr/bin/env python3
"""Efeito da frequência do laço de controle (seção do controlador).

Roda a simulação cinemática (dry_run) com o laço de controle a 1, 2, 5, 10, 20 e 50 Hz
nos três cenários, mantendo velocidade e tolerância, e mede: conclusão das rotas,
erro máximo de seguimento, folga mínima e contatos com obstáculos.

Uso: python3 analise_frequencia.py   -> frequencia.csv
"""
import csv
import json
import os
import tempfile

import analisar_resultados as ar
import dry_run
from cenarios import CENARIOS

FREQS = [1, 2, 5, 10, 20, 50]
linhas = []
tmp = tempfile.mkdtemp()
print("cen    Hz  concluíram  erro seg. máx (m)  folga mín (m)  contatos   passo v/f (m)  tolerância (m)")
for n, c in CENARIOS.items():
    rfile = os.path.join(c['dir'], 'out', 'routes.json')
    cfg = json.load(open(rfile))
    t_exp = max(d['real_length_m'] * cfg['scale'] / d['speed'] + sum(s['dur_sim'] for s in d['stops']) for d in cfg['drones'])
    for hz in FREQS:
        pref = os.path.join(tmp, f'c{n}_cinematico{hz}')
        dry_run.simulate(cfg, hz=hz, sim_dt=0.005, prefixo=pref, t_max=2.0 * t_exp)
        vants, res = ar.analisar_execucao(pref)
        passo = max(d['speed'] for d in cfg['drones']) / hz
        tol = cfg['drones'][0]['tolerance']
        row = {'cenario': n, 'hz': hz, 'concluiram': res['concluiram'], 'vants': res['vants'],
               'erro_seg_max_m': res['erro_seg_max_m'], 'folga_min_m': res['folga_min_m'],
               'contatos': res['contatos_total'], 'passo_m': round(passo, 2), 'tolerancia_m': round(tol, 2),
               'makespan_sim_s': res['makespan_sim_s']}
        linhas.append(row)
        print(f"{n:>3} {hz:>5}   {res['concluiram']:>3}/{res['vants']:<3}      {res['erro_seg_max_m']:>8.2f}        "
              f"{res['folga_min_m']:>7.2f}    {res['contatos_total']:>6}      {passo:>8.2f}       {tol:>6.2f}")
with open('frequencia.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(linhas[0].keys()))
    w.writeheader(); w.writerows(linhas)
