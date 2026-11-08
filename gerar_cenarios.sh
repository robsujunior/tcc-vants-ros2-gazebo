#!/bin/bash
# Gera routes.json, world.sdf e bridge.sh dos três cenários (e do teste sem margem),
# a partir dos arquivos do solver e dos obstáculos reconstruídos.
# Uso (dentro da pasta do projeto):  bash gerar_cenarios.sh
cd "$(dirname "$0")"
gerar() {  # gerar <pasta> <entrada> <escala> <aceleração> <saída> [opções extras]
  local D=$1 IN=$2 S=$3 A=$4 OUT=$5; shift 5
  python3 build_scenario.py --input "$D/$IN" --paths-dir "$D" --obstacles "$D/obstacles.json" \
      --out "$D/$OUT" --scale "$S" --speedup "$A" "$@"
}
gerar cenario_1 D_T30_R6_preprocessing.txt 0.01 20 out
gerar cenario_1 D_T30_R6_preprocessing.txt 0.01 20 out_margem0 --margin-sim 0 --sem-recarga
gerar cenario_2 C_T20_R3_preprocessed.txt   0.02 10 out
gerar cenario_3 C_T50_R9_preprocessed.txt   0.02 10 out
