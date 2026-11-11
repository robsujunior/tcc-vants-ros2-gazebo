# Rotas de cobertura para múltiplos VANTs com desvio de obstáculos: simulação em ROS 2 e Gazebo

Código e dados do Trabalho de Conclusão de Curso de Robson da Lomba Campos Junior
(Sistemas de Informação, UFVJM, 2026). O projeto converte as rotas de um *solver* de
roteamento de múltiplos VANTs com recarga em pontos de passagem, gera automaticamente o
mundo de simulação do Gazebo e executa a missão de todos os VANTs ao mesmo tempo, com as
paradas de recarga, registrando a odometria para a análise.

## Requisitos

- Ubuntu 22.04, ROS 2 Humble e Gazebo Fortress, com o pacote `ros_gz` (`ros-humble-ros-gz`)
- Python 3.10 e as bibliotecas de `requirements.txt` (`pip install -r requirements.txt`)

As análises, as figuras e a simulação cinemática não precisam do ROS 2 nem do Gazebo.

## Estrutura

| Arquivo ou pasta | Conteúdo |
| --- | --- |
| `cenario_1/`, `cenario_2/`, `cenario_3/` | Arquivo de entrada do *solver* (`.txt`), rotas (`path_N.txt`), solução (`sol_1.txt`), obstáculos reconstruídos (`obstacles.json`) e arquivos gerados para a simulação (`out/`) |
| `cenarios.py` | Escala e aceleração do tempo de cada cenário |
| `build_scenario.py` | Gerador das rotas (`routes.json`), do mundo (`world.sdf`) e da ponte (`bridge.sh`) |
| `gerar_cenarios.sh` | Gera os arquivos dos três cenários e do teste sem margem |
| `follower_logic.py` | Controlador de seguimento de rota (Algoritmo 3 do TCC) |
| `multi_follower.py` | Nó ROS 2 que comanda todos os VANTs |
| `registrador.py` | Nó ROS 2 que grava a odometria e os eventos em CSV |
| `rodar_experimentos.sh` | Executa todos os experimentos no Gazebo, sem interface gráfica |
| `dry_run.py` | Simulação cinemática, sem Gazebo |
| `analisar_resultados.py` | Métricas das execuções no Gazebo |
| `analise_autonomia.py`, `analise_margem.py`, `analise_frequencia.py`, `tempo_preprocessamento.py` | Análises complementares do capítulo 6 |
| `figuras.py`, `figura_margem0.py`, `tabela_consolidada.py` | Figuras e tabelas do TCC |
| `ferramentas/preprocessor.py` | Pré-processador de desvios, com a margem de segurança como parâmetro e a gravação dos polígonos originais na seção `@obstacles` da saída (ver Origem dos arquivos) |
| `resultados/` | Odometria, eventos e métricas das execuções usadas no TCC |
| `figuras/` | Figuras do TCC e fontes dos diagramas (`.dot`) |

## Como reproduzir

1. Gerar os arquivos da simulação (opcional; já estão em `cenario_N/out/`):

       bash gerar_cenarios.sh

2. Executar os experimentos no Gazebo (cerca de 2 h 30 min: teste sem margem e três
   repetições de cada cenário). Os registros e as métricas vão para `resultados/`:

       bash rodar_experimentos.sh          # 3 repetições
       bash rodar_experimentos.sh 2        # 2 repetições
       JANELA=1 bash rodar_experimentos.sh # com a janela do Gazebo

3. Recalcular as métricas, as análises e as figuras a partir dos registros:

       python3 analisar_resultados.py resultados
       python3 tabela_consolidada.py resultados
       python3 analise_autonomia.py
       python3 analise_margem.py
       python3 analise_frequencia.py
       python3 tempo_preprocessamento.py 5
       python3 figuras.py resultados
       python3 figura_margem0.py

## Origem dos arquivos

Os arquivos de entrada, as rotas e as soluções de cada cenário foram gerados pelo *solver* de
Santin (2021), e o pré-processador de desvios foi disponibilizado pelo grupo de pesquisa ao
qual o *solver* está vinculado, com as modificações descritas no capítulo 5 do TCC. Os
obstáculos foram reconstruídos a partir das figuras dos cenários (seção 5.5). Os demais
arquivos foram desenvolvidos neste trabalho.

SANTIN, Rafael. *Roteamento e localização de estações de recarga em tarefas de cobertura de
área com múltiplos veículos aéreos não tripulados*. 2021. Tese (Doutorado em Engenharia
Elétrica) – Escola de Engenharia, Universidade Federal de Minas Gerais, Belo Horizonte, 2021.
