#!/usr/bin/env python3
"""Mede o tempo de pré-processamento (cálculo dos desvios) para os três cenários.

Usa as mesmas funções do pré-processador (PathPlanningContext.compute_all), sem a
interface gráfica: lê os alvos do arquivo do cenário, adiciona os obstáculos
(expandidos pela margem, pelo próprio pré-processador) e cronometra o cálculo de
todos os desvios.

Uso: python3 tempo_preprocessamento.py [repeticoes]
"""
import json, os, sys, time, statistics as st
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ferramentas'))
import matplotlib
matplotlib.use('Agg')
import shapely as spl
import preprocessor as pp
import build_scenario as bs
from cenarios import CENARIOS, DRONE_SIZE

reps = int(sys.argv[1]) if len(sys.argv) > 1 else 5
print("cen alvos obst  margem(m)  pares avaliados  desvios>0   tempo médio (s)   desvio-padrão")
saida = []
for n, c in CENARIOS.items():
    base, targets, _, _ = bs.read_professor_file(os.path.join(c['dir'], c['input']))
    obs = json.load(open(os.path.join(c['dir'], 'obstacles.json')))
    for margem_sim in (0.0, 2 * DRONE_SIZE):
        margem = margem_sim / c['scale']
        tempos = []
        for _ in range(reps):
            plane = pp.PlaneContext(safety_margin=margem)
            plane.add_depot(pp.Depot(*base))
            for k in range(0, len(targets), 2):
                plane.add_line(pp.Target(*targets[k]), pp.Target(*targets[k + 1]))
            for v in obs.values():
                plane.add_obstacle(spl.Polygon(v))
            ctx = pp.PathPlanningContext(plane, (0.0, 1.0))
            t0 = time.perf_counter()
            warn = ctx.compute_all()
            tempos.append(time.perf_counter() - t0)
        pares = len(ctx.line_results) + len(ctx.all_transition_results)
        desv = sum(1 for _, dc in ctx.line_results.values() if dc > 1e-10) + len(ctx.transition_results)
        sd = st.stdev(tempos) if len(tempos) > 1 else 0.0
        print(f"{n:>3} {len(targets):>5} {len(obs):>4}  {margem:>8.0f}  {pares:>15}  {desv:>9}   {st.mean(tempos):>12.3f}   {sd:>10.3f}"
              + (f"   avisos: {len(warn)}" if warn else ""))
        saida.append({"cenario": n, "alvos": len(targets), "obstaculos": len(obs), "margem_real_m": margem,
                      "pares": pares, "desvios": desv, "tempo_medio_s": st.mean(tempos), "desvio_padrao_s": sd})
json.dump(saida, open('tempo_preprocessamento.json', 'w'), indent=1)
