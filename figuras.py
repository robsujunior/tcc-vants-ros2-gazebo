#!/usr/bin/env python3
"""Gera as figuras do TCC (PNG, 300 dpi) para os três cenários.

  figuras/cenario_N_rotas.png       todas as rotas planejadas sobre o mapa
  figuras/cenario_N_por_vant.png    um painel por VANT (rota planejada; se houver
                                    dados do Gazebo, a trajetória executada por cima)
  figuras/cenario_N_linha_tempo.png voo x recarga de cada VANT ao longo da missão

Uso: python3 figuras.py [pasta_resultados]
     (se a pasta tiver c<N>_r1_odom.csv, usa a repetição 1 do Gazebo; senão usa só o planejado)
"""
import csv
import json
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter
from matplotlib.patches import Patch
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Polygon

import build_scenario as bs
from cenarios import CENARIOS

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['DejaVu Serif'], 'font.size': 9,
                     'axes.edgecolor': '#52514e', 'axes.labelcolor': '#0b0b0b',
                     'xtick.color': '#52514e', 'ytick.color': '#52514e'})
# paleta categórica de referência (ordem fixa); o 9º VANT usa cinza escuro tracejado
CORES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948', '#52514e']
ESTILO = ['-'] * 8 + ['--']
OBST, MARGEM, LINHA = '#b9a48c', '#7a6a58', '#d9d8d4'
OUT = 'figuras'
VIRG = FuncFormatter(lambda v, _: f'{v:g}'.replace('.', ','))


def br(v, nd=1):
    return f'{v:.{nd}f}'.replace('.', ',')


def carregar(n):
    c = CENARIOS[n]
    base, targets, cfgs, robots = bs.read_professor_file(os.path.join(c['dir'], c['input']))
    obs = json.load(open(os.path.join(c['dir'], 'obstacles.json')))
    cfg = json.load(open(os.path.join(c['dir'], 'out', 'routes.json')))
    return base, targets, obs, cfg


def mapa_base(ax, base, targets, obs, margem_real, km=True, lw_linha=3.0):
    f = 1e-3 if km else 1.0
    for v in obs.values():
        p = Polygon(v)
        infl = p.buffer(margem_real, join_style=2)
        ax.add_patch(MplPolygon([(x * f, y * f) for x, y in infl.exterior.coords], closed=True,
                                fill=False, ls=(0, (3, 2)), lw=0.7, ec=MARGEM, zorder=1))
        ax.add_patch(MplPolygon([(x * f, y * f) for x, y in v], closed=True, fc=OBST, ec='#7a6a58', lw=0.6, zorder=1))
    for k in range(0, len(targets), 2):
        (x1, y1), (x2, y2) = targets[k], targets[k + 1]
        ax.plot([x1 * f, x2 * f], [y1 * f, y2 * f], color=LINHA, lw=lw_linha, solid_capstyle='butt', zorder=0)
    ax.plot(base[0] * f, base[1] * f, 's', ms=7, mfc='#0b0b0b', mec='white', mew=1.2, zorder=6)
    ax.set_aspect('equal')
    ax.xaxis.set_major_formatter(VIRG); ax.yaxis.set_major_formatter(VIRG)
    ax.grid(True, color='#ecebe8', lw=0.5)
    ax.set_axisbelow(True)


def estacoes(d, scale):
    return [d['waypoints'][s['wp']] for s in d.get('stops', [])]


def fig_rotas(n, base, targets, obs, cfg):
    s = cfg['scale']
    fig, ax = plt.subplots(figsize=(6.3, 6.3))
    mapa_base(ax, base, targets, obs, cfg['margin_sim'] / s)
    handles = []
    for i, d in enumerate(cfg['drones']):
        xs = [p[0] / s / 1000 for p in d['waypoints']]
        ys = [p[1] / s / 1000 for p in d['waypoints']]
        ax.plot(xs, ys, color=CORES[i], ls=ESTILO[i], lw=1.4, zorder=3 + i * 0.01)
        handles.append(Line2D([], [], color=CORES[i], ls=ESTILO[i], lw=1.6, label=f"VANT {i} ({d['config'].replace('_', ' ')})"))
        for x, y in estacoes(d, s):
            ax.plot(x / s / 1000, y / s / 1000, '^', ms=7, mfc=CORES[i], mec='white', mew=0.9, zorder=5)
    handles += [Line2D([], [], marker='s', ls='', mfc='#0b0b0b', mec='white', ms=7, label='Base'),
                Line2D([], [], marker='^', ls='', mfc='#888', mec='white', ms=7, label='Recarga'),
                Line2D([], [], color=LINHA, lw=3, label='Linha de cobertura'),
                Patch(fc=OBST, ec='#7a6a58', label='Obstáculo'),
                Line2D([], [], color=MARGEM, ls=(0, (3, 2)), lw=0.8, label='Margem de segurança')]
    ax.set_xlabel('x (km)'); ax.set_ylabel('y (km)')
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=8)
    fig.savefig(os.path.join(OUT, f'cenario_{n}_rotas.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def ler_odom(pasta, n):
    fn = os.path.join(pasta, f'c{n}_r1_odom.csv') if pasta else None
    if not fn or not os.path.exists(fn):
        return None
    tr = defaultdict(list)
    with open(fn) as fh:
        for r in csv.DictReader(fh):
            tr[r['drone']].append((float(r['x']), float(r['y'])))
    return tr


def fig_por_vant(n, base, targets, obs, cfg, odom):
    s = cfg['scale']
    nd = len(cfg['drones'])
    ys_all = [t[1] for t in targets]; xs_all = [t[0] for t in targets]
    alto = (max(ys_all) - min(ys_all)) > 2 * (max(xs_all) - min(xs_all))
    cols = min(nd, 6) if alto else 3
    rows = math.ceil(nd / cols)
    altura = 3.9 if alto else 2.35
    fig, axs = plt.subplots(rows, cols, figsize=(6.3, altura * rows), squeeze=False)
    for i, d in enumerate(cfg['drones']):
        ax = axs[i // cols][i % cols]
        mapa_base(ax, base, targets, obs, cfg['margin_sim'] / s, lw_linha=1.2)
        xs = [p[0] / s / 1000 for p in d['waypoints']]
        ys = [p[1] / s / 1000 for p in d['waypoints']]
        ax.plot(xs, ys, color=CORES[i], lw=2.2, zorder=3)
        if odom and d['name'] in odom:
            ox = [p[0] / s / 1000 for p in odom[d['name']]]
            oy = [p[1] / s / 1000 for p in odom[d['name']]]
            ax.plot(ox, oy, color='#0b0b0b', lw=0.6, zorder=4)
        for x, y in estacoes(d, s):
            ax.plot(x / s / 1000, y / s / 1000, '^', ms=6, mfc=CORES[i], mec='white', mew=0.8, zorder=5)
        sep = '\n' if alto else ' · '
        ax.set_title(f"VANT {i}{sep}{d['config'].replace('_', ' ')}{sep}{br(d['real_length_m']/1000)} km", fontsize=8)
        ax.tick_params(labelsize=6)
    for j in range(nd, rows * cols):
        axs[j // cols][j % cols].axis('off')
    leg = [Line2D([], [], color='#888', lw=2.2, label='Rota planejada')]
    if odom:
        leg.append(Line2D([], [], color='#0b0b0b', lw=0.8, label='Trajetória executada no Gazebo'))
    leg += [Line2D([], [], marker='^', ls='', mfc='#888', mec='white', ms=6, label='Recarga'),
            Line2D([], [], marker='s', ls='', mfc='#0b0b0b', mec='white', ms=6, label='Base')]
    fig.legend(handles=leg, loc='lower center', ncol=2 if alto else len(leg), frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.07 if alto else 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, f'cenario_{n}_por_vant.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def ler_eventos(pasta, n):
    """Eventos da repetição 1 no Gazebo: {drone: [(evento, t_sim), ...]}."""
    fn = os.path.join(pasta, f'c{n}_r1_eventos.csv') if pasta else None
    if not fn or not os.path.exists(fn):
        return None
    ev = defaultdict(list)
    with open(fn) as fh:
        for r in csv.DictReader(fh):
            if r['evento'] in ('inicio', 'inicio_recarga', 'fim_recarga', 'concluido'):
                ev[r['drone']].append((r['evento'], float(r['t_sim'])))
    return ev


def blocos_planejados(d):
    stops = d.get('stops', [])
    voos = [s['voo_real'] for s in stops]
    voo_total = d['real_length_m'] / d['max_vel_real']
    voos.append(voo_total - sum(voos))
    recs = [s['dur_real'] for s in stops] + [d['prop'] * voos[-1]]
    return voos, recs


def blocos_gazebo(d, ev, speedup):
    """Voos e recargas medidos no Gazebo (convertidos para tempo real); a recarga final na
    base não é executada na simulação e é calculada como q vezes o último voo planejado,
    como no tempo de missão equivalente."""
    t0 = next(t for k, t in ev if k == 'inicio')
    voos, recs, t = [], [], t0
    for k, tk in ev:
        if k == 'inicio_recarga':
            voos.append((tk - t) * speedup); t = tk
        elif k == 'fim_recarga':
            recs.append((tk - t) * speedup); t = tk
        elif k == 'concluido':
            voos.append((tk - t) * speedup)
    _, recs_plan = blocos_planejados(d)
    recs.append(recs_plan[-1])
    return voos, recs


def fig_linha_tempo(n, cfg, ev=None):
    """Voo e recarga de cada VANT em tempo real equivalente (horas): medidos no Gazebo
    (repetição 1) quando houver eventos; senão, a partir do planejado."""
    fig, ax = plt.subplots(figsize=(6.3, 0.42 * len(cfg['drones']) + 0.9))
    pmax = max(d['solver_cost'] for d in cfg['drones']) / 3600
    for i, d in enumerate(cfg['drones']):
        if ev and d['name'] in ev:
            voos, recs = blocos_gazebo(d, ev[d['name']], cfg['speedup'])
        else:
            voos, recs = blocos_planejados(d)
        t = 0.0
        for k, (v, r) in enumerate(zip(voos, recs)):
            ax.barh(i, v / 3600, left=t / 3600, height=0.6, color=CORES[i], ec='white', lw=1)
            t += v
            last = k == len(voos) - 1
            ax.barh(i, r / 3600, left=t / 3600, height=0.6, color='white' if last else '#d9d8d4',
                    ec='#52514e' if last else 'white', lw=0.6 if last else 1, hatch='///' if last else None)
            t += r
        ax.text(t / 3600 + 0.03, i, f"{br(t/3600, 2)} h", va='center', fontsize=7, color='#52514e', zorder=5,
                bbox=dict(fc='white', ec='none', pad=0.6))
    ax.axvline(pmax, color='#0b0b0b', ls=(0, (4, 3)), lw=0.8, zorder=0)
    ax.set_yticks(range(len(cfg['drones'])))
    ax.set_yticklabels([f"VANT {i}" for i in range(len(cfg['drones']))])
    ax.invert_yaxis()
    ax.set_xlabel('tempo real equivalente (h)')
    ax.xaxis.set_major_formatter(VIRG)
    ax.set_xlim(0, max(ax.get_xlim()[1], pmax + 0.45))
    ax.grid(True, axis='x', color='#ecebe8', lw=0.5); ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    leg = [Patch(fc='#888', label='Voo (cor do VANT)'), Patch(fc='#d9d8d4', label='Recarga em estação'),
           Patch(fc='white', ec='#52514e', hatch='///', label='Recarga final na base'),
           Line2D([], [], color='#0b0b0b', ls=(0, (4, 3)), lw=0.8, label='Maior custo previsto pelo solver')]
    ax.legend(handles=leg, loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=8)
    fig.savefig(os.path.join(OUT, f'cenario_{n}_linha_tempo.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def main():
    pasta = sys.argv[1] if len(sys.argv) > 1 else None
    os.makedirs(OUT, exist_ok=True)
    for n in CENARIOS:
        base, targets, obs, cfg = carregar(n)
        fig_rotas(n, base, targets, obs, cfg)
        fig_por_vant(n, base, targets, obs, cfg, ler_odom(pasta, n))
        fig_linha_tempo(n, cfg, ler_eventos(pasta, n))
        print(f'cenário {n}: figuras geradas em {OUT}/')


if __name__ == '__main__':
    main()
