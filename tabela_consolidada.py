#!/usr/bin/env python3
"""Consolida as repetições do Gazebo: média e desvio-padrão por VANT (tabela do TCC)."""
import csv, statistics as st, sys
from collections import defaultdict
pasta = sys.argv[1] if len(sys.argv) > 1 else 'resultados'
rows = [r for r in csv.DictReader(open(f'{pasta}/metricas_por_vant.csv')) if '_r' in r['execucao']]
g = defaultdict(list)
for r in rows:
    g[(int(r['cenario']), r['vant'])].append(r)
campos = ['missao_equiv_real_s', 'dif_missao_solver_pct', 'comprimento_gazebo_km', 'dif_comprimento_pct',
          'erro_seg_medio_m', 'erro_seg_max_m', 'folga_min_m', 'folga_min_real_m', 'recargas']
out = []
for (c, v), rs in sorted(g.items()):
    o = {'cenario': c, 'vant': v.replace('drone_', 'VANT '), 'config': rs[0]['config'].replace('_', ' '),
         'repeticoes': len(rs), 'concluiu_todas': all(r['concluiu'] == '1' for r in rs),
         'custo_solver_s': float(rs[0]['custo_solver_s']), 'comprimento_planejado_km': float(rs[0]['comprimento_planejado_km'])}
    for k in campos:
        xs = [float(r[k]) for r in rs]
        o[k + '_media'] = round(st.mean(xs), 3); o[k + '_dp'] = round(st.pstdev(xs), 3)
    out.append(o)
with open(f'{pasta}/tabela_por_vant_consolidada.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
print(f'{len(out)} VANTs -> {pasta}/tabela_por_vant_consolidada.csv')
