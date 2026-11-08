#!/usr/bin/env python3
"""Figura do teste sem margem: rotas planejadas (margem 0) e onde cada VANT parou no Gazebo."""
import csv, json, os
from collections import defaultdict
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import figuras as F
import build_scenario as bs
from cenarios import CENARIOS

c = CENARIOS[1]
base, targets, _, _ = bs.read_professor_file(os.path.join(c['dir'], c['input']))
obs = json.load(open(os.path.join(c['dir'], 'obstacles.json')))
cfg = json.load(open(os.path.join(c['dir'], 'out_margem0', 'routes.json')))
s = cfg['scale']
tr = defaultdict(list)
for r in csv.DictReader(open('resultados/c1_margem0_odom.csv')):
    tr[r['drone']].append((float(r['x']) / s / 1000, float(r['y']) / s / 1000))
fig, ax = plt.subplots(figsize=(6.3, 6.3))
fins = []
F.mapa_base(ax, base, targets, obs, 0.0)
for i, d in enumerate(cfg['drones']):
    xs = [p[0] / s / 1000 for p in d['waypoints']]; ys = [p[1] / s / 1000 for p in d['waypoints']]
    ax.plot(xs, ys, color=F.CORES[i], lw=0.9, ls=(0, (4, 2)), alpha=0.8, zorder=2)
    t = tr[d['name']]
    ax.plot([p[0] for p in t], [p[1] for p in t], color=F.CORES[i], lw=2.4, zorder=3)
    ax.plot(*t[-1], 'X', ms=10, mfc=F.CORES[i], mec='#0b0b0b', mew=0.8, zorder=6)
    fins.append((i, t[-1]))
grupos = []
for i, p in fins:
    for g in grupos:
        if abs(g[1][0] - p[0]) < 0.4 and abs(g[1][1] - p[1]) < 0.4:
            g[0].append(i); break
    else:
        grupos.append(([i], p))
for ids, p in grupos:
    rot = 'VANT ' + ' e '.join(str(i) for i in ids) if len(ids) > 1 else f'VANT {ids[0]}'
    if len(ids) > 1:
        rot = 'VANTs ' + ' e '.join(str(i) for i in ids)
    vizinho = any(q is not p and 0 < q[0] - p[0] < 2.5 and abs(q[1] - p[1]) < 0.8 for _, q in grupos)
    off = (-6, -16) if vizinho else (8, 4)
    ax.annotate(rot, p, xytext=off, textcoords='offset points', fontsize=8, color='#0b0b0b', zorder=7)
leg = [Line2D([], [], color='#888', lw=0.9, ls=(0, (4, 2)), label='Rota planejada (sem margem)'),
       Line2D([], [], color='#888', lw=2.4, label='Trajetória executada no Gazebo'),
       Line2D([], [], marker='X', ls='', mfc='#888', mec='#0b0b0b', ms=9, label='Posição em que o VANT ficou preso')]
ax.legend(handles=leg, loc='upper left', bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=8)
ax.set_xlabel('x (km)'); ax.set_ylabel('y (km)')
fig.savefig('figuras/cenario_1_sem_margem.png', dpi=300, bbox_inches='tight')
