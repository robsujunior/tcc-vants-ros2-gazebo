"""Configuração dos três cenários usados no TCC (mesmos parâmetros das simulações já rodadas)."""
CENARIOS = {
    1: {"dir": "cenario_1", "input": "D_T30_R6_preprocessing.txt", "scale": 0.01, "speedup": 20},
    2: {"dir": "cenario_2", "input": "C_T20_R3_preprocessed.txt", "scale": 0.02, "speedup": 10},
    3: {"dir": "cenario_3", "input": "C_T50_R9_preprocessed.txt", "scale": 0.02, "speedup": 10},
}
DRONE_SIZE = 1.0
ROBOT_OFFSET = 1
