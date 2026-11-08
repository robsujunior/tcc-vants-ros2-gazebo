#!/bin/bash
# Roda todos os experimentos no Gazebo sozinho, um depois do outro, e grava os dados
# em resultados/. Não precisa clicar em nada: o Gazebo abre sem janela (modo servidor)
# e já começa rodando.
#
# Uso (dentro da pasta simulacao):
#   bash rodar_experimentos.sh            -> teste sem margem + 3 repetições de cada cenário
#   bash rodar_experimentos.sh 2          -> teste sem margem + 2 repetições de cada cenário
#   JANELA=1 bash rodar_experimentos.sh   -> igual, mas abrindo a janela do Gazebo
#
# Tempo aproximado: ~5 min do teste sem margem + ~47 min por repetição dos 3 cenários.

REPS=${1:-3}
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
mkdir -p resultados

limpar() {
  pkill -f multi_follower.py 2>/dev/null
  pkill -INT -f registrador.py 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f "gz_sim.launch.py" 2>/dev/null
  pkill -f "ign gazebo" 2>/dev/null
  sleep 2
  pkill -9 -f "ign gazebo" 2>/dev/null
  pkill -9 -f ruby 2>/dev/null
  sleep 2
}

rodar() {   # rodar <pasta out> <prefixo dos arquivos de saída>
  local OUT=$1 PREF=$2
  local LIMITE
  LIMITE=$(python3 -c "
import json; c = json.load(open('$OUT/routes.json'))
t = max(d['real_length_m'] * c['scale'] / d['speed'] + sum(s['dur_sim'] for s in d.get('stops', [])) for d in c['drones'])
print(int(1.6 * t + 120))")
  echo ""
  echo "=== $(date +%H:%M) | $PREF | $OUT | limite ${LIMITE}s ==="
  limpar
  if [ "${JANELA:-0}" = "1" ]; then GZARGS="-r $PWD/$OUT/world.sdf"; else GZARGS="-s -r $PWD/$OUT/world.sdf"; fi
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$GZARGS" > "${PREF}_gazebo.log" 2>&1 &
  sleep 10
  bash "$OUT/bridge.sh" > "${PREF}_bridge.log" 2>&1 &
  sleep 4
  python3 registrador.py "$OUT/routes.json" "$PREF" > "${PREF}_registrador.log" 2>&1 &
  local REG=$!
  sleep 2
  timeout "$LIMITE" python3 multi_follower.py "$OUT/routes.json" 2>&1 | tee "${PREF}_follower.log" | grep --line-buffered -E "conclu|Iniciando|recarregando"
  sleep 3
  kill -INT $REG 2>/dev/null
  wait $REG 2>/dev/null
  limpar
  echo "=== fim: $(wc -l < "${PREF}_odom.csv") linhas de odometria gravadas ==="
}

# 0) Tempo de pré-processamento medido nesta máquina (~1 min)
python3 tempo_preprocessamento.py 5 | tee resultados/tempo_preprocessamento.txt

# 1) Teste sem margem de segurança (cenário 1, sem recarga): mostra o VANT preso no obstáculo
rodar cenario_1/out_margem0 resultados/c1_margem0

# 2) Cenários completos (com margem e recargas), REPS repetições
for r in $(seq 1 "$REPS"); do
  for c in 1 2 3; do
    rodar "cenario_$c/out" "resultados/c${c}_r${r}"
  done
done

echo ""
echo "Todos os experimentos terminaram. Calculando as métricas..."
python3 analisar_resultados.py resultados | tee resultados/resumo.txt
python3 figuras.py resultados
echo "Pronto. Me envie a pasta resultados/ compactada (sem os arquivos .log, se ficar grande)."
