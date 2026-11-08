#!/usr/bin/env python3
"""Calcula as métricas das execuções no Gazebo a partir dos CSVs do registrador.

Procura em <pasta> arquivos  c<N>_r<R>_odom.csv / c<N>_r<R>_eventos.csv  (cenário N,
repetição R) e também  c1_margem0_*  (teste sem margem). Para cada VANT calcula:
  - se concluiu a rota e o tempo de missão (tempo de simulação e equivalente real);
  - comparação do tempo equivalente com o custo previsto pelo solver;
  - comprimento percorrido no Gazebo x comprimento planejado;
  - erro de seguimento (distância da posição até a rota planejada);
  - folga mínima até os obstáculos reais e contatos;
  - tempo total de recarga;
e, por execução, o fator de tempo real e a menor distância entre dois VANTs.

Uso:  python3 analisar_resultados.py resultados
Saída: <pasta>/metricas_por_vant.csv, <pasta>/metricas_por_execucao.csv e resumo na tela.
"""
import csv
import glob
import json
import math
import os
import re
import statistics as st
import sys
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from cenarios import CENARIOS


def ler_csv(fn):
    with open(fn, newline='') as fh:
        return list(csv.DictReader(fh))


def routes_de(nome):
    m = re.match(r'c(\d+)_(r\d+|margem0|cinematico\w*)', nome)
    cen = int(m.group(1))
    sub = 'out_margem0' if 'margem0' in nome else 'out'
    return cen, os.path.join(CENARIOS[cen]['dir'], sub, 'routes.json')


def analisar_execucao(prefixo):
    nome = os.path.basename(prefixo)
    cen, rfile = routes_de(nome)
    cfg = json.load(open(rfile))
    scale, speedup = cfg['scale'], cfg['speedup']
    half = cfg['drone_size'] * math.sqrt(2) / 2
    obst = unary_union([Polygon(v) for v in cfg['obstacles_sim']])

    odom = defaultdict(list)
    for r in ler_csv(prefixo + '_odom.csv'):
        odom[r['drone']].append((float(r['t_wall']), float(r['t_sim']), float(r['x']), float(r['y']), float(r['z'])))
    ev = defaultdict(list)
    for r in ler_csv(prefixo + '_eventos.csv'):
        ev[r['drone']].append((r['evento'], float(r['t_sim']), int(r['wp'])))

    linhas = []
    for d in cfg['drones']:
        nm = d['name']
        amostras = sorted(odom.get(nm, []), key=lambda a: a[1])
        e = ev.get(nm, [])
        t_ini = next((t for k, t, _ in e if k == 'inicio'), amostras[0][1] if amostras else 0.0)
        t_fim = next((t for k, t, _ in e if k == 'concluido'), None)
        concluiu = t_fim is not None
        wps_alcancados = max([w + 1 for k, _, w in e if k == 'waypoint'], default=0)

        # recargas: pares inicio_recarga / fim_recarga
        rec, aberto = 0.0, None
        for k, t, _ in e:
            if k == 'inicio_recarga':
                aberto = t
            elif k == 'fim_recarga' and aberto is not None:
                rec += t - aberto
                aberto = None

        # trecho da missão (do início até a conclusão ou o fim do registro)
        fim = t_fim if concluiu else (amostras[-1][1] if amostras else t_ini)
        miss = [a for a in amostras if t_ini - 1e-6 <= a[1] <= fim + 1e-6]
        comp = sum(math.dist(p[2:4], q[2:4]) for p, q in zip(miss[:-1], miss[1:]))
        rota = LineString(d['waypoints'])
        erros = [rota.distance(Point(a[2], a[3])) for a in miss] or [0.0]
        folgas = [obst.distance(Point(a[2], a[3])) for a in miss] or [float('inf')]
        contatos = sum(1 for f in folgas if f < half)
        alt_err = [abs(a[4] - d['altitude']) for a in miss] or [0.0]

        voo_total_real = d['real_length_m'] / d['max_vel_real']
        voo_ultimo = voo_total_real - sum(s['voo_real'] for s in d.get('stops', []))
        recarga_final = d['prop'] * voo_ultimo
        dur_sim = fim - t_ini
        missao_eq = dur_sim * speedup + (recarga_final if cfg.get('recarga', True) else 0.0)

        linhas.append({
            'execucao': nome, 'cenario': cen, 'vant': nm, 'config': d['config'],
            'concluiu': int(concluiu), 'waypoints': f"{wps_alcancados}/{len(d['waypoints'])}",
            'duracao_sim_s': round(dur_sim, 1),
            'recarga_sim_s': round(rec, 1),
            'recargas': sum(1 for k, _, _ in e if k == 'inicio_recarga'),
            'missao_equiv_real_s': round(missao_eq, 0),
            'custo_solver_s': d['solver_cost'],
            'dif_missao_solver_pct': round(100 * (missao_eq / d['solver_cost'] - 1), 2),
            'comprimento_gazebo_km': round(comp / scale / 1000, 2),
            'comprimento_planejado_km': round(d['real_length_m'] / 1000, 2),
            'dif_comprimento_pct': round(100 * (comp / scale / d['real_length_m'] - 1), 2),
            'erro_seg_medio_m': round(st.mean(erros), 3),
            'erro_seg_max_m': round(max(erros), 3),
            'erro_alt_medio_m': round(st.mean(alt_err), 3),
            'folga_min_m': round(min(folgas), 3),
            'folga_min_real_m': round(min(folgas) / scale, 0),
            'amostras_contato': contatos,
        })

    # fator de tempo real e distância mínima entre VANTs (amostras alinhadas a 0,1 s)
    todas = [a for v in odom.values() for a in v]
    if todas:
        tw = [a[0] for a in todas]; ts = [a[1] for a in todas]
        rtf = (max(ts) - min(ts)) / max(1e-9, max(tw) - min(tw))
    else:
        rtf = float('nan')
    grade = defaultdict(dict)
    for nm, v in odom.items():
        for a in v:
            grade[round(a[1], 1)][nm] = a[2:5]
    # Distância entre VANTs fora da base (na base eles nascem empilhados por altitude).
    # "Cruzamento": distância horizontal menor que o tamanho do VANT, ou seja, só a
    # separação de altitude impede a colisão.
    base = cfg['drones'][0]['spawn'][:2]
    raio_base = 3 * cfg['drone_size']
    dmin, par = float('inf'), None
    em_cruz, cruzamentos = set(), 0
    for t in sorted(grade):
        pos = grade[t]
        nomes = sorted(n for n in pos if math.dist(pos[n][:2], base) > raio_base)
        agora = set()
        for i in range(len(nomes)):
            for j in range(i + 1, len(nomes)):
                a, b = pos[nomes[i]], pos[nomes[j]]
                dd = math.dist(a, b)
                if dd < dmin:
                    dmin, par = dd, (nomes[i], nomes[j])
                if math.dist(a[:2], b[:2]) < cfg['drone_size']:
                    agora.add((nomes[i], nomes[j]))
        cruzamentos += len(agora - em_cruz)
        em_cruz = agora
    resumo = {
        'execucao': nome, 'cenario': cen, 'vants': len(cfg['drones']),
        'concluiram': sum(l['concluiu'] for l in linhas),
        'makespan_sim_s': max(l['duracao_sim_s'] for l in linhas),
        'makespan_equiv_s': max(l['missao_equiv_real_s'] for l in linhas),
        'maior_custo_solver_s': max(d['solver_cost'] for d in cfg['drones']),
        'fator_tempo_real': round(rtf, 3),
        'dist_min_entre_vants_m': round(dmin, 2),
        'par_mais_proximo': '-'.join(par) if par else '',
        'cruzamentos_sep_altitude': cruzamentos,
        'contatos_total': sum(l['amostras_contato'] for l in linhas),
        'erro_seg_max_m': max(l['erro_seg_max_m'] for l in linhas),
        'folga_min_m': min(l['folga_min_m'] for l in linhas),
    }
    return linhas, resumo


def main():
    pasta = sys.argv[1] if len(sys.argv) > 1 else 'resultados'
    prefixos = sorted(f[:-len('_odom.csv')] for f in glob.glob(os.path.join(pasta, 'c*_odom.csv')))
    if not prefixos:
        print(f'Nenhum arquivo c*_odom.csv em {pasta}')
        sys.exit(1)
    todas, resumos = [], []
    for p in prefixos:
        l, r = analisar_execucao(p)
        todas += l; resumos.append(r)
    for fn, rows in (('metricas_por_vant.csv', todas), ('metricas_por_execucao.csv', resumos)):
        with open(os.path.join(pasta, fn), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)

    print('execução        VANTs concl.  makespan sim  equiv. real  solver   RTF    dist. mín. VANTs  cruzam.  folga mín.  contatos')
    for r in resumos:
        print(f"{r['execucao']:<16}{r['vants']:>4} {r['concluiram']:>5}   {r['makespan_sim_s']:>9.0f} s"
              f"  {r['makespan_equiv_s']:>9.0f} s {r['maior_custo_solver_s']:>8.0f}  {r['fator_tempo_real']:5.2f}"
              f"   {r['dist_min_entre_vants_m']:>8.2f} m   {r['cruzamentos_sep_altitude']:>5}     {r['folga_min_m']:6.2f} m   {r['contatos_total']:>5}")
    print(f"\nArquivos gravados em {pasta}/metricas_por_vant.csv e {pasta}/metricas_por_execucao.csv")


if __name__ == '__main__':
    main()
