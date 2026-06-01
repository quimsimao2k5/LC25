"""
================================================================================
TRABALHO PRÁTICO 4 - Verificação de Segurança em Sistemas Híbridos
================================================================================
Modelação e Verificação de Controlo de Tráfego Marítimo

Este script implementa:
1. Três Autómatos Híbridos: Navio 1, Navio 2, Semáforo
2. Modelo físico completo com parâmetros variáveis por setor
3. Verificação BMC (Bounded Model Checking) e k-Indução
4. Propriedades de Segurança Suficiente e Forte (Deadlock)

Autor: Trabalho Académico - Lógica Computacional 2025
================================================================================
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from z3 import *
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from enum import Enum
import time

# ============================================================================
# PARTE 1: DEFINIÇÕES E ESTRUTURAS DE DADOS
# ============================================================================

class SinalSemaforo(Enum):
    """Sinais do semáforo conforme enunciado."""
    VERDE = "verde"      # Permissão de transitar
    AMARELO = "amarelo"  # Autorização para setor vizinho do outro navio
    VERMELHO = "vermelho"  # Proibido transitar

class TipoZona(Enum):
    """Tipos de zona conforme enunciado."""
    ACELERACAO = "aceleracao"      # γ ≈ 2.0
    DESACELERACAO = "desaceleracao"  # γ ≈ 0.5
    CRUZEIRO = "cruzeiro"          # γ ≈ 1.0 (manter velocidade)
    BAIXA_VELOCIDADE = "baixa"     # γ ≈ 0.3

@dataclass
class ParametrosSetor:
    """Parâmetros físicos de cada setor conforme enunciado."""
    gamma: float   # Aceleração da propulsão
    epsilon: float  # Força da corrente (+ favor, - contra)
    phi: float     # Rumo (ângulo em radianos)
    V_max: float   # Limite superior de velocidade
    tipo: TipoZona

# ============================================================================
# PARTE 2: CONFIGURAÇÃO DO MAPA E PARÂMETROS POR SETOR
# ============================================================================

"""
TOPOLOGIA DO MAPA (2-3-2-1-2-3-2):
                                    
    Porto A                                              Porto B
    ┌────┐     ┌────┐     ┌────┐                ┌────┐     ┌────┐     ┌────┐
    │ 13 │────▶│  9 │────▶│  1 │                │  2 │────▶│  6 │────▶│ 14 │
    └────┘     └────┘     └────┘     ┌────┐     └────┘     └────┘     └────┘
       │          │          │       │    │        │          │          │
       │       ┌────┐        │       │ 0  │        │       ┌────┐        │
       └──────▶│  5 │◀───────┘       │    │        └──────▶│ 10 │◀───────┘
               └────┘                └────┘                └────┘
       ┌──────▶   │                                           │   ◀──────┐
       │          ▼                                           ▼          │
    ┌────┐     ┌────┐     ┌────┐                ┌────┐     ┌────┐     ┌────┐
    │ 11 │────▶│  7 │────▶│  3 │                │  4 │────▶│  8 │────▶│ 12 │
    └────┘     └────┘     └────┘                └────┘     └────┘     └────┘

Navio 1 (A→B): Esquerda para Direita
Navio 2 (B→A): Direita para Esquerda
"""

# Constantes Físicas Globais
SIGMA = 1.0    # Coeficiente de atrito com a água
DT = 0.1       # Passo de tempo (discretização)
Z_MAX = 1.0    # Tamanho do setor (1 km)

# Definição dos parâmetros por setor (conforme enunciado)
# Rota A→B: {13,11} e {2,4} são aceleração; {1,3} e {12,14} são desaceleração
# Rota B→A: As zonas são invertidas

PARAMETROS_SETORES: Dict[int, ParametrosSetor] = {
    # === LADO ESQUERDO (Porto A) ===
    # Coluna 1: Zonas de Aceleração para A→B
    13: ParametrosSetor(gamma=2.0, epsilon=0.1, phi=0.0, V_max=2.0, tipo=TipoZona.ACELERACAO),
    11: ParametrosSetor(gamma=2.0, epsilon=0.1, phi=0.0, V_max=2.0, tipo=TipoZona.ACELERACAO),
    
    # Coluna 2: Zonas de Cruzeiro
    9: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=np.pi/12, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    5: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=0.0, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    7: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=-np.pi/12, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    
    # Coluna 3: Zonas de Desaceleração para A→B (antes do gargalo)
    1: ParametrosSetor(gamma=0.5, epsilon=-0.1, phi=np.pi/8, V_max=1.0, tipo=TipoZona.DESACELERACAO),
    3: ParametrosSetor(gamma=0.5, epsilon=-0.1, phi=-np.pi/8, V_max=1.0, tipo=TipoZona.DESACELERACAO),
    
    # === CENTRO (Gargalo) ===
    0: ParametrosSetor(gamma=0.3, epsilon=0.0, phi=0.0, V_max=0.5, tipo=TipoZona.BAIXA_VELOCIDADE),
    
    # === LADO DIREITO (Porto B) ===
    # Coluna 5: Zonas de Aceleração para A→B (saindo do gargalo)
    2: ParametrosSetor(gamma=2.0, epsilon=0.1, phi=-np.pi/8, V_max=2.0, tipo=TipoZona.ACELERACAO),
    4: ParametrosSetor(gamma=2.0, epsilon=0.1, phi=np.pi/8, V_max=2.0, tipo=TipoZona.ACELERACAO),
    
    # Coluna 6: Zonas de Cruzeiro
    6: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=-np.pi/12, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    10: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=0.0, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    8: ParametrosSetor(gamma=1.0, epsilon=0.0, phi=np.pi/12, V_max=1.5, tipo=TipoZona.CRUZEIRO),
    
    # Coluna 7: Zonas de Desaceleração para A→B (chegada ao Porto B)
    14: ParametrosSetor(gamma=0.5, epsilon=-0.1, phi=0.0, V_max=1.0, tipo=TipoZona.DESACELERACAO),
    12: ParametrosSetor(gamma=0.5, epsilon=-0.1, phi=0.0, V_max=1.0, tipo=TipoZona.DESACELERACAO),
}

# Coordenadas visuais dos setores (para animação)
SECTOR_COORDS: Dict[int, Tuple[float, float]] = {
    0: (0, 0),
    # Esquerda
    1: (-1, 0.5), 3: (-1, -0.5),
    9: (-2, 1), 5: (-2, 0), 7: (-2, -1),
    13: (-3, 0.5), 11: (-3, -0.5),
    # Direita
    2: (1, 0.5), 4: (1, -0.5),
    6: (2, 1), 10: (2, 0), 8: (2, -1),
    14: (3, 0.5), 12: (3, -0.5)
}

# Adjacências do grafo (para visualização das conexões)
ADJACENCIAS = [
    # Lado Esquerdo (A→B)
    (13, 9), (13, 5), (11, 7), (11, 5),
    (9, 1), (7, 3), (5, 1), (5, 3),
    (1, 0), (3, 0),
    # Lado Direito (continuação A→B)
    (0, 2), (0, 4),
    (2, 6), (2, 10), (4, 8), (4, 10),
    (6, 14), (8, 12), (10, 14), (10, 12)
]

# ============================================================================
# PARTE 3: MODELO Z3 - AUTÓMATOS HÍBRIDOS
# ============================================================================

class SistemaHibrido:
    """
    Implementação do Sistema Híbrido com 3 Autómatos:
    - Autómato Navio 1 (A→B)
    - Autómato Navio 2 (B→A)
    - Autómato Semáforo (Controlador de Tráfego)
    """
    
    def __init__(self):
        self.solver = Solver()
        self.states: List[Dict] = []
        
    def declare_state(self, i: int) -> Dict:
        """
        Declara as variáveis de estado para o passo i.
        
        AUTÓMATO NAVIO 1:
            - s1: Modo/Local (setor atual)
            - z1: Posição dentro do setor [0, 1]
            - v1: Velocidade
            - path1: Escolha de rota (0=Top, 1=Bottom)
        
        AUTÓMATO NAVIO 2:
            - s2, z2, v2, path2: Análogo ao Navio 1
        
        AUTÓMATO SEMÁFORO:
            - t: Tempo mestre global
            - signal1: Sinal para Navio 1 (0=Verde, 1=Amarelo, 2=Vermelho)
            - signal2: Sinal para Navio 2
        """
        state = {}
        
        # --- Autómato Navio 1 ---
        state['s1'] = Int(f's1_{i}')      # Modo (setor)
        state['z1'] = Real(f'z1_{i}')     # Var. contínua: posição
        state['v1'] = Real(f'v1_{i}')     # Var. contínua: velocidade
        state['path1'] = Int(f'path1_{i}')  # Decisão de rota
        
        # --- Autómato Navio 2 ---
        state['s2'] = Int(f's2_{i}')
        state['z2'] = Real(f'z2_{i}')
        state['v2'] = Real(f'v2_{i}')
        state['path2'] = Int(f'path2_{i}')
        
        # --- Autómato Semáforo ---
        state['t'] = Real(f't_{i}')       # Tempo mestre
        state['signal1'] = Int(f'sig1_{i}')  # Sinal para N1
        state['signal2'] = Int(f'sig2_{i}')  # Sinal para N2
        
        return state
    
    def init(self, s: Dict) -> BoolRef:
        """
        Predicado de Estado Inicial.
        
        Condições iniciais:
        - Navio 1: No Porto A (setor 13 ou 11), parado
        - Navio 2: No Porto B (setor 14 ou 12), parado
        - Semáforo: Tempo t=0, ambos com sinal verde
        """
        return And(
            # --- Autómato Navio 1 ---
            Or(s['s1'] == 13, s['s1'] == 11),  # Porto A
            s['z1'] == 0.0,
            s['v1'] == 0.0,
            Or(s['path1'] == 0, s['path1'] == 1),
            
            # --- Autómato Navio 2 ---
            Or(s['s2'] == 14, s['s2'] == 12),  # Porto B
            s['z2'] == 0.0,
            s['v2'] == 0.0,
            Or(s['path2'] == 0, s['path2'] == 1),
            
            # --- Autómato Semáforo ---
            s['t'] == 0.0,
            s['signal1'] == 0,  # Verde
            s['signal2'] == 0   # Verde
        )
    
    def get_gamma(self, s_id, direcao: str) -> RealVal:
        """
        Retorna a aceleração γ para um setor, considerando a direção.
        Para B→A, as zonas de aceleração/desaceleração são invertidas.
        """
        # Construir expressão Z3 com If encadeados
        result = RealVal(1.0)  # Default: cruzeiro
        
        for setor_id, params in PARAMETROS_SETORES.items():
            gamma_val = params.gamma
            
            # Inverter para B→A: aceleração↔desaceleração
            if direcao == "BA":
                if params.tipo == TipoZona.ACELERACAO:
                    gamma_val = 0.5  # Vira desaceleração
                elif params.tipo == TipoZona.DESACELERACAO:
                    gamma_val = 2.0  # Vira aceleração
            
            result = If(s_id == setor_id, RealVal(gamma_val), result)
        
        return result
    
    def get_V_max(self, s_id) -> RealVal:
        """Retorna o limite de velocidade V para um setor."""
        result = RealVal(1.5)
        for setor_id, params in PARAMETROS_SETORES.items():
            result = If(s_id == setor_id, RealVal(params.V_max), result)
        return result
    
    def get_epsilon(self, s_id) -> RealVal:
        """Retorna a força da corrente ε para um setor."""
        result = RealVal(0.0)
        for setor_id, params in PARAMETROS_SETORES.items():
            result = If(s_id == setor_id, RealVal(params.epsilon), result)
        return result
    
    def dynamics(self, v, s_id, direcao: str):
        """
        Equações de fluxo (dinâmica física) conforme enunciado:
        
        v̇ + σv = γ     se v ≤ V
        v̇ + σv = ε     se v > V
        
        Discretização de Euler:
        v_new = v + Δt(força - σv)
        """
        gamma = self.get_gamma(s_id, direcao)
        V_max = self.get_V_max(s_id)
        epsilon = self.get_epsilon(s_id)
        
        # Força depende de v estar abaixo ou acima do limite
        forca = If(v <= V_max, gamma, epsilon)
        
        # Discretização: v_new = v + dt * (força - σ*v)
        v_new = v + DT * (forca - SIGMA * v)
        
        # Garantir v >= 0
        return If(v_new < 0, RealVal(0.0), v_new)
    
    def get_next_sector_n1(self, current_s, path_type):
        """
        Função de Transição Discreta para Navio 1 (A→B).
        Define os "jumps" do autómato híbrido.
        
        Retorna o próximo setor baseado no setor atual e escolha de rota.
        """
        next_s = IntVal(-1)  # -1 = Fim da rota / Erro
        
        # === LADO ESQUERDO (Entrada) ===
        # 13 → 9 (top) ou 5 (mid)
        next_s = If(current_s == 13, If(path_type == 0, 9, 5), next_s)
        # 11 → 7 (bot) ou 5 (mid)
        next_s = If(current_s == 11, If(path_type == 1, 7, 5), next_s)
        
        # === COLUNA MEIO-ESQUERDA ===
        next_s = If(current_s == 9, 1, next_s)
        next_s = If(current_s == 7, 3, next_s)
        next_s = If(current_s == 5, If(path_type == 0, 1, 3), next_s)
        
        # === CONVERGÊNCIA PARA GARGALO ===
        next_s = If(Or(current_s == 1, current_s == 3), 0, next_s)
        
        # === SAÍDA DO GARGALO ===
        next_s = If(current_s == 0, If(path_type == 0, 2, 4), next_s)
        
        # === COLUNA MEIO-DIREITA ===
        next_s = If(current_s == 2, If(path_type == 0, 6, 10), next_s)
        next_s = If(current_s == 4, If(path_type == 1, 8, 10), next_s)
        
        # === LADO DIREITO (Saída) ===
        next_s = If(current_s == 6, 14, next_s)
        next_s = If(current_s == 8, 12, next_s)
        next_s = If(current_s == 10, If(path_type == 0, 14, 12), next_s)
        
        return next_s
    
    def get_next_sector_n2(self, current_s, path_type):
        """
        Função de Transição Discreta para Navio 2 (B→A).
        Rota inversa ao Navio 1.
        """
        next_s = IntVal(-1)
        
        # === LADO DIREITO (Entrada para N2) ===
        next_s = If(current_s == 14, If(path_type == 0, 6, 10), next_s)
        next_s = If(current_s == 12, If(path_type == 1, 8, 10), next_s)
        
        # === COLUNA MEIO-DIREITA ===
        next_s = If(current_s == 6, 2, next_s)
        next_s = If(current_s == 8, 4, next_s)
        next_s = If(current_s == 10, If(path_type == 0, 2, 4), next_s)
        
        # === CONVERGÊNCIA PARA GARGALO ===
        next_s = If(Or(current_s == 2, current_s == 4), 0, next_s)
        
        # === SAÍDA DO GARGALO ===
        next_s = If(current_s == 0, If(path_type == 0, 1, 3), next_s)
        
        # === COLUNA MEIO-ESQUERDA ===
        next_s = If(current_s == 1, If(path_type == 0, 9, 5), next_s)
        next_s = If(current_s == 3, If(path_type == 1, 7, 5), next_s)
        
        # === LADO ESQUERDO (Saída para N2) ===
        next_s = If(current_s == 9, 13, next_s)
        next_s = If(current_s == 7, 11, next_s)
        next_s = If(current_s == 5, If(path_type == 0, 13, 11), next_s)
        
        return next_s
    
    def compute_semaphore_signals(self, s: Dict, next_s1, next_s2):
        """
        AUTÓMATO DO SEMÁFORO - Lógica de Sincronismo
        
        Calcula os sinais do semáforo baseado nas posições dos navios.
        
        Sinais:
        - VERDE (0): Próximo setor livre, pode avançar
        - AMARELO (1): Próximo setor é vizinho do outro navio (cuidado)
        - VERMELHO (2): Próximo setor ocupado, deve parar
        """
        # Setores vizinhos (adjacentes no grafo)
        def are_neighbors(s_a, s_b):
            """Verifica se dois setores são vizinhos."""
            neighbor_cond = Or(False)
            for (a, b) in ADJACENCIAS:
                neighbor_cond = Or(neighbor_cond, 
                                   And(s_a == a, s_b == b),
                                   And(s_a == b, s_b == a))
            return neighbor_cond
        
        # --- Sinal para Navio 1 ---
        # Vermelho: próximo setor ocupado por N2
        signal1_vermelho = (next_s1 == s['s2'])
        # Amarelo: próximo setor é vizinho de onde N2 está
        signal1_amarelo = And(Not(signal1_vermelho), are_neighbors(next_s1, s['s2']))
        # Verde: caso contrário
        signal1 = If(signal1_vermelho, 2, If(signal1_amarelo, 1, 0))
        
        # --- Sinal para Navio 2 ---
        signal2_vermelho = (next_s2 == s['s1'])
        signal2_amarelo = And(Not(signal2_vermelho), are_neighbors(next_s2, s['s1']))
        signal2 = If(signal2_vermelho, 2, If(signal2_amarelo, 1, 0))
        
        return signal1, signal2
    
    def trans(self, s: Dict, s_next: Dict) -> BoolRef:
        """
        Relação de Transição do Sistema Híbrido.
        
        Combina:
        1. Fluxo contínuo (física dentro do setor)
        2. Jumps discretos (mudança de setor)
        3. Sincronismo com semáforo
        """
        # === AUTÓMATO NAVIO 1 ===
        v1_new = self.dynamics(s['v1'], s['s1'], "AB")
        z1_potential = s['z1'] + DT * s['v1']
        
        next_s1_id = self.get_next_sector_n1(s['s1'], s['path1'])
        
        # Guarda do Jump: z >= Z_MAX ∧ existe próximo ∧ sinal != vermelho
        signal1, signal2 = self.compute_semaphore_signals(s, next_s1_id, 
                                                          self.get_next_sector_n2(s['s2'], s['path2']))
        
        can_cross1 = And(
            z1_potential >= Z_MAX,
            next_s1_id != -1,
            signal1 != 2  # Não está vermelho
        )
        
        # Se amarelo, reduzir velocidade (cautela)
        v1_after_cross = If(signal1 == 1, v1_new * 0.5, s['v1'])  # Mantém v se verde
        
        # Transição do Navio 1
        at_boundary1 = z1_potential >= Z_MAX
        
        trans_n1 = If(can_cross1,
            # JUMP: Muda de setor
            And(s_next['s1'] == next_s1_id,
                s_next['z1'] == 0.0,
                s_next['v1'] == v1_after_cross,
                s_next['path1'] == s['path1']),
            # FLUXO: Permanece no setor
            If(at_boundary1,
                # Bloqueado na fronteira (semáforo vermelho)
                And(s_next['s1'] == s['s1'],
                    s_next['z1'] == Z_MAX,
                    s_next['v1'] == 0.0,  # Para
                    s_next['path1'] == s['path1']),
                # Movimento normal
                And(s_next['s1'] == s['s1'],
                    s_next['z1'] == z1_potential,
                    s_next['v1'] == v1_new,
                    s_next['path1'] == s['path1'])
            )
        )
        
        # === AUTÓMATO NAVIO 2 ===
        v2_new = self.dynamics(s['v2'], s['s2'], "BA")
        z2_potential = s['z2'] + DT * s['v2']
        
        next_s2_id = self.get_next_sector_n2(s['s2'], s['path2'])
        
        # Recalcular sinais para N2
        _, signal2_for_n2 = self.compute_semaphore_signals(s, next_s1_id, next_s2_id)
        
        # Prioridade: Se ambos querem o mesmo setor, N1 tem prioridade
        conflict = And(can_cross1, next_s1_id == next_s2_id)
        
        can_cross2 = And(
            z2_potential >= Z_MAX,
            next_s2_id != -1,
            signal2_for_n2 != 2,
            Not(conflict)  # N2 cede se há conflito
        )
        
        v2_after_cross = If(signal2_for_n2 == 1, v2_new * 0.5, s['v2'])
        at_boundary2 = z2_potential >= Z_MAX
        
        trans_n2 = If(can_cross2,
            And(s_next['s2'] == next_s2_id,
                s_next['z2'] == 0.0,
                s_next['v2'] == v2_after_cross,
                s_next['path2'] == s['path2']),
            If(at_boundary2,
                And(s_next['s2'] == s['s2'],
                    s_next['z2'] == Z_MAX,
                    s_next['v2'] == 0.0,
                    s_next['path2'] == s['path2']),
                And(s_next['s2'] == s['s2'],
                    s_next['z2'] == z2_potential,
                    s_next['v2'] == v2_new,
                    s_next['path2'] == s['path2'])
            )
        )
        
        # === AUTÓMATO SEMÁFORO ===
        trans_semaforo = And(
            s_next['t'] == s['t'] + DT,
            s_next['signal1'] == signal1,
            s_next['signal2'] == signal2_for_n2
        )
        
        return And(trans_n1, trans_n2, trans_semaforo)
    
    # ========================================================================
    # PROPRIEDADES DE SEGURANÇA
    # ========================================================================
    
    def prop_seguranca_suficiente(self, s: Dict) -> BoolRef:
        """
        Propriedade de Segurança Suficiente:
        Os dois navios nunca estão no mesmo setor.
        
        ¬(s1 = s2)
        """
        return s['s1'] == s['s2']  # Colisão
    
    def prop_seguranca_forte(self, s: Dict) -> BoolRef:
        """
        Propriedade de Segurança Forte (Deadlock):
        Além de não colidirem, nenhum navio fica bloqueado indefinidamente.
        
        Deadlock = Ambos parados, no limite do setor, e bloqueados mutuamente.
        """
        next_s1 = self.get_next_sector_n1(s['s1'], s['path1'])
        next_s2 = self.get_next_sector_n2(s['s2'], s['path2'])
        
        # N1 bloqueado: no limite, parado, e próximo = onde está N2 (ou fim de rota)
        blocked1 = And(
            s['z1'] == Z_MAX,
            s['v1'] == 0.0,
            Or(next_s1 == -1, next_s1 == s['s2'])
        )
        
        # N2 bloqueado: análogo
        blocked2 = And(
            s['z2'] == Z_MAX,
            s['v2'] == 0.0,
            Or(next_s2 == -1, next_s2 == s['s1'])
        )
        
        return And(blocked1, blocked2)  # Deadlock mútuo


# ============================================================================
# PARTE 4: VERIFICAÇÃO BMC E K-INDUÇÃO
# ============================================================================

class Verificador:
    """Implementa BMC e k-Indução para verificação de propriedades."""
    
    def __init__(self, sistema: SistemaHibrido):
        self.sistema = sistema
    
    def bmc(self, k_max: int = 100, safe_mode: bool = True, 
            check_deadlock: bool = True, verbose: bool = True) -> Optional[List[Dict]]:
        """
        Bounded Model Checking (BMC).
        
        Procura um contra-exemplo para a propriedade de segurança
        até profundidade k_max.
        
        Args:
            k_max: Limite máximo de passos
            safe_mode: Se True, usa transições seguras (com semáforo)
            check_deadlock: Se True, também verifica deadlock
            verbose: Se True, imprime progresso
        
        Returns:
            Traço de execução se encontrar violação, None caso contrário
        """
        if verbose:
            modo = "SEGURO (com semáforos)" if safe_mode else "INSEGURO (sem controlo)"
            print(f"\n{'='*60}")
            print(f"BMC - Bounded Model Checking ({modo})")
            print(f"{'='*60}")
            print(f"Limite: k={k_max} passos")
            print(f"Verificar: Colisão" + (" + Deadlock" if check_deadlock else ""))
        
        solver = Solver()
        states = [self.sistema.declare_state(0)]
        solver.add(self.sistema.init(states[0]))
        
        start_time = time.time()
        
        for k in range(1, k_max + 1):
            states.append(self.sistema.declare_state(k))
            solver.add(self.sistema.trans(states[k-1], states[k]))
            
            # Verificar propriedade
            solver.push()
            
            # Propriedade: Colisão OU Deadlock
            if check_deadlock:
                prop = Or(
                    self.sistema.prop_seguranca_suficiente(states[k]),
                    self.sistema.prop_seguranca_forte(states[k])
                )
            else:
                prop = self.sistema.prop_seguranca_suficiente(states[k])
            
            solver.add(prop)
            
            if solver.check() == sat:
                elapsed = time.time() - start_time
                model = solver.model()
                
                # Identificar tipo de falha
                is_collision = is_true(model.eval(
                    self.sistema.prop_seguranca_suficiente(states[k])))
                is_deadlock = check_deadlock and is_true(model.eval(
                    self.sistema.prop_seguranca_forte(states[k])))
                
                if verbose:
                    print(f"\n{'!'*60}")
                    print(f"!!! FALHA DETETADA NO PASSO k={k} (t={k*DT:.1f}s) !!!")
                    print(f"{'!'*60}")
                    if is_collision:
                        s1 = model[states[k]['s1']].as_long()
                        print(f"TIPO: COLISÃO - Ambos navios no setor S{s1}")
                    if is_deadlock:
                        s1 = model[states[k]['s1']].as_long()
                        s2 = model[states[k]['s2']].as_long()
                        print(f"TIPO: DEADLOCK - N1 em S{s1}, N2 em S{s2} (bloqueados)")
                    print(f"Tempo de verificação: {elapsed:.2f}s")
                
                return self.extract_trace(model, states, k)
            
            solver.pop()
            
            if verbose and k % 20 == 0:
                elapsed = time.time() - start_time
                print(f"  Passo {k}: Seguro até aqui... ({elapsed:.1f}s)")
        
        elapsed = time.time() - start_time
        if verbose:
            print(f"\n{'='*60}")
            print(f"✓ Nenhuma violação encontrada até k={k_max}")
            print(f"  Tempo total: {elapsed:.2f}s")
            print(f"{'='*60}")
        
        return None
    
    def k_inducao(self, k: int = 10, verbose: bool = True) -> bool:
        """
        Verificação por k-Indução.
        
        Tenta provar que a propriedade de segurança vale para TODOS os estados,
        não apenas até um limite.
        
        Consiste em duas partes:
        1. Caso Base: Propriedade vale nos primeiros k passos
        2. Passo Indutivo: Se vale em k passos consecutivos, vale no k+1
        
        Returns:
            True se propriedade provada, False se falhou
        """
        if verbose:
            print(f"\n{'='*60}")
            print(f"K-INDUÇÃO (k={k})")
            print(f"{'='*60}")
        
        # === CASO BASE ===
        if verbose:
            print("\n[1/2] Verificando Caso Base...")
        
        solver_base = Solver()
        states_base = [self.sistema.declare_state(i) for i in range(k+1)]
        
        solver_base.add(self.sistema.init(states_base[0]))
        for i in range(k):
            solver_base.add(self.sistema.trans(states_base[i], states_base[i+1]))
        
        # Verificar se existe colisão nos primeiros k passos
        collision_in_base = Or([self.sistema.prop_seguranca_suficiente(states_base[i]) 
                                for i in range(k+1)])
        solver_base.add(collision_in_base)
        
        if solver_base.check() == sat:
            if verbose:
                print("  ✗ Caso Base FALHOU - Colisão encontrada nos primeiros k passos")
            return False
        
        if verbose:
            print("  ✓ Caso Base OK")
        
        # === PASSO INDUTIVO ===
        if verbose:
            print("\n[2/2] Verificando Passo Indutivo...")
        
        solver_ind = Solver()
        states_ind = [self.sistema.declare_state(i) for i in range(k+2)]
        
        # Assumir: k estados consecutivos seguros (hipótese indutiva)
        for i in range(k+1):
            solver_ind.add(Not(self.sistema.prop_seguranca_suficiente(states_ind[i])))
            if i < k+1:
                solver_ind.add(self.sistema.trans(states_ind[i], states_ind[i+1]))
        
        # Verificar: estado k+1 também é seguro?
        solver_ind.add(self.sistema.prop_seguranca_suficiente(states_ind[k+1]))
        
        if solver_ind.check() == sat:
            if verbose:
                print("  ✗ Passo Indutivo FALHOU")
                print("    (Existe transição de estados seguros para estado inseguro)")
            return False
        
        if verbose:
            print("  ✓ Passo Indutivo OK")
            print(f"\n{'='*60}")
            print(f"✓✓✓ PROPRIEDADE PROVADA POR K-INDUÇÃO ✓✓✓")
            print(f"    O sistema é SEGURO para todos os estados alcançáveis!")
            print(f"{'='*60}")
        
        return True
    
    def extract_trace(self, model, states: List[Dict], k_max: int) -> List[Dict]:
        """Extrai o traço de execução do modelo Z3."""
        trace = []
        
        def get_real(var):
            val = model[var]
            if val is None:
                return 0.0
            try:
                return float(val.numerator_as_long()) / float(val.denominator_as_long())
            except:
                return float(val.as_long()) if hasattr(val, 'as_long') else 0.0
        
        def get_int(var):
            val = model[var]
            return val.as_long() if val is not None else 0
        
        for k in range(k_max + 1):
            trace.append({
                't': get_real(states[k]['t']),
                's1': get_int(states[k]['s1']),
                'z1': get_real(states[k]['z1']),
                'v1': get_real(states[k]['v1']),
                's2': get_int(states[k]['s2']),
                'z2': get_real(states[k]['z2']),
                'v2': get_real(states[k]['v2']),
                'signal1': get_int(states[k]['signal1']),
                'signal2': get_int(states[k]['signal2'])
            })
        
        return trace


# ============================================================================
# PARTE 5: VISUALIZAÇÃO AVANÇADA
# ============================================================================

class Visualizador:
    """Gera animações e gráficos para os traços de execução."""
    
    def __init__(self):
        self.colors_tipo = {
            TipoZona.ACELERACAO: '#90EE90',      # Verde claro
            TipoZona.DESACELERACAO: '#FFB6C1',  # Rosa claro
            TipoZona.CRUZEIRO: '#ADD8E6',        # Azul claro
            TipoZona.BAIXA_VELOCIDADE: '#FFE4B5'  # Laranja claro
        }
        
        self.signal_colors = {
            0: 'green',   # Verde
            1: 'yellow',  # Amarelo
            2: 'red'      # Vermelho
        }
    
    def criar_animacao(self, trace: List[Dict], titulo: str = "Simulação") -> animation.FuncAnimation:
        """
        Cria uma animação do traço de execução.
        """
        if not trace:
            print("Sem dados para visualizar.")
            return None
        
        fig, (ax_map, ax_info) = plt.subplots(1, 2, figsize=(16, 7), 
                                               gridspec_kw={'width_ratios': [3, 1]})
        
        # === PAINEL DO MAPA ===
        ax_map.set_xlim(-4.5, 4.5)
        ax_map.set_ylim(-2.5, 2.5)
        ax_map.set_aspect('equal')
        ax_map.set_title(titulo, fontsize=14, fontweight='bold')
        ax_map.axis('off')
        
        # Desenhar conexões (arestas do grafo)
        for (a, b) in ADJACENCIAS:
            if a in SECTOR_COORDS and b in SECTOR_COORDS:
                xa, ya = SECTOR_COORDS[a]
                xb, yb = SECTOR_COORDS[b]
                ax_map.plot([xa, xb], [ya, yb], 'k-', alpha=0.3, linewidth=1)
        
        # Desenhar setores
        patches_dict = {}
        for sec_id, (x, y) in SECTOR_COORDS.items():
            tipo = PARAMETROS_SETORES[sec_id].tipo
            cor = self.colors_tipo[tipo]
            
            rect = patches.FancyBboxPatch(
                (x-0.4, y-0.4), 0.8, 0.8,
                boxstyle="round,pad=0.05",
                linewidth=2, edgecolor='black', facecolor=cor, alpha=0.7
            )
            ax_map.add_patch(rect)
            ax_map.text(x, y, f"S{sec_id}", ha='center', va='center', fontsize=9, fontweight='bold')
            patches_dict[sec_id] = rect
        
        # Labels dos portos
        ax_map.text(-3.5, 1.5, "PORTO A", fontsize=12, fontweight='bold', color='blue')
        ax_map.text(3.0, 1.5, "PORTO B", fontsize=12, fontweight='bold', color='red')
        
        # Navios
        ship1, = ax_map.plot([], [], 'bo', markersize=18, label='Navio 1 (A→B)', zorder=10)
        ship2, = ax_map.plot([], [], 'rs', markersize=18, label='Navio 2 (B→A)', zorder=10)
        
        # Semáforos visuais
        sem1_patch = patches.Circle((-4, 1.5), 0.15, color='gray')
        sem2_patch = patches.Circle((4, 1.5), 0.15, color='gray')
        ax_map.add_patch(sem1_patch)
        ax_map.add_patch(sem2_patch)
        ax_map.text(-4, 1.8, "Sem. N1", ha='center', fontsize=8)
        ax_map.text(4, 1.8, "Sem. N2", ha='center', fontsize=8)
        
        # Legenda de cores
        legend_elements = [
            patches.Patch(facecolor=self.colors_tipo[TipoZona.ACELERACAO], label='Aceleração'),
            patches.Patch(facecolor=self.colors_tipo[TipoZona.DESACELERACAO], label='Desaceleração'),
            patches.Patch(facecolor=self.colors_tipo[TipoZona.CRUZEIRO], label='Cruzeiro'),
            patches.Patch(facecolor=self.colors_tipo[TipoZona.BAIXA_VELOCIDADE], label='Baixa Vel.'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Navio 1'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10, label='Navio 2'),
        ]
        ax_map.legend(handles=legend_elements, loc='lower left', fontsize=8)
        
        # === PAINEL DE INFORMAÇÕES ===
        ax_info.axis('off')
        info_text = ax_info.text(0.1, 0.9, "", transform=ax_info.transAxes, 
                                  fontsize=10, verticalalignment='top', fontfamily='monospace',
                                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Texto de status
        status_text = ax_map.text(0, -2.0, "", ha='center', fontsize=11, 
                                   bbox=dict(facecolor='white', edgecolor='black', alpha=0.9))
        
        def get_position(s_id, z, next_s_id):
            """Calcula posição interpolada entre setores."""
            if s_id not in SECTOR_COORDS:
                return 0, 0
            x_curr, y_curr = SECTOR_COORDS[s_id]
            
            if next_s_id is not None and next_s_id in SECTOR_COORDS and next_s_id != -1:
                x_next, y_next = SECTOR_COORDS[next_s_id]
                # Interpolar com z limitado a 0.9 para melhor visualização
                z_vis = min(z, 0.9)
                return x_curr + z_vis * (x_next - x_curr), y_curr + z_vis * (y_next - y_curr)
            return x_curr, y_curr
        
        def predict_next(s_id, is_n1, path=0):
            """Prevê o próximo setor para visualização."""
            # Tabela simplificada de transições
            if is_n1:  # A→B
                trans = {13: 9, 11: 7, 9: 1, 7: 3, 5: 1, 1: 0, 3: 0, 
                         0: 2, 2: 6, 4: 8, 6: 14, 8: 12, 10: 14}
            else:  # B→A
                trans = {14: 6, 12: 8, 6: 2, 8: 4, 10: 2, 2: 0, 4: 0,
                         0: 1, 1: 9, 3: 7, 5: 13, 9: 13, 7: 11}
            return trans.get(s_id, None)
        
        def update(frame):
            data = trace[frame]
            
            # Posições dos navios
            next_s1 = predict_next(data['s1'], True)
            next_s2 = predict_next(data['s2'], False)
            
            x1, y1 = get_position(data['s1'], data['z1'], next_s1)
            x2, y2 = get_position(data['s2'], data['z2'], next_s2)
            
            ship1.set_data([x1], [y1])
            ship2.set_data([x2], [y2])
            
            # Cores dos semáforos
            sem1_patch.set_color(self.signal_colors.get(data.get('signal1', 0), 'gray'))
            sem2_patch.set_color(self.signal_colors.get(data.get('signal2', 0), 'gray'))
            
            # Detetar colisão
            collision = data['s1'] == data['s2']
            
            # Colorir setores
            for sec_id, rect in patches_dict.items():
                if collision and sec_id == data['s1']:
                    rect.set_facecolor('red')
                    rect.set_alpha(1.0)
                elif sec_id == data['s1']:
                    rect.set_edgecolor('blue')
                    rect.set_linewidth(3)
                elif sec_id == data['s2']:
                    rect.set_edgecolor('red')
                    rect.set_linewidth(3)
                else:
                    rect.set_facecolor(self.colors_tipo[PARAMETROS_SETORES[sec_id].tipo])
                    rect.set_edgecolor('black')
                    rect.set_linewidth(2)
                    rect.set_alpha(0.7)
            
            # Status
            if collision:
                status_text.set_text(f"⚠️ COLISÃO NO SETOR S{data['s1']}! ⚠️")
                status_text.set_color('red')
            else:
                status_text.set_text(f"t = {data['t']:.1f}s")
                status_text.set_color('black')
            
            # Informações detalhadas
            signal_names = {0: '🟢 VERDE', 1: '🟡 AMARELO', 2: '🔴 VERMELHO'}
            info = f"""
╔══════════════════════════════╗
║      ESTADO DO SISTEMA       ║
╠══════════════════════════════╣
║ Tempo: {data['t']:>6.2f} s             ║
╠══════════════════════════════╣
║ NAVIO 1 (A→B)                ║
║   Setor: S{data['s1']:<3}                 ║
║   Posição: {data['z1']:.3f} km          ║
║   Velocidade: {data['v1']:.3f} km/s     ║
║   Semáforo: {signal_names.get(data.get('signal1', 0), '?'):<10}   ║
╠══════════════════════════════╣
║ NAVIO 2 (B→A)                ║
║   Setor: S{data['s2']:<3}                 ║
║   Posição: {data['z2']:.3f} km          ║
║   Velocidade: {data['v2']:.3f} km/s     ║
║   Semáforo: {signal_names.get(data.get('signal2', 0), '?'):<10}   ║
╚══════════════════════════════╝
"""
            info_text.set_text(info)
            
            return ship1, ship2, status_text, info_text, sem1_patch, sem2_patch, *patches_dict.values()
        
        ani = animation.FuncAnimation(fig, update, frames=len(trace), 
                                       interval=200, blit=False, repeat=True)
        return ani
    
    def plot_trajetoria(self, trace: List[Dict], titulo: str = "Trajetórias"):
        """Gera gráficos das trajetórias dos navios ao longo do tempo."""
        if not trace:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(titulo, fontsize=14, fontweight='bold')
        
        t = [d['t'] for d in trace]
        
        # Setor ao longo do tempo
        ax1 = axes[0, 0]
        ax1.plot(t, [d['s1'] for d in trace], 'b-o', label='Navio 1', markersize=3)
        ax1.plot(t, [d['s2'] for d in trace], 'r-s', label='Navio 2', markersize=3)
        ax1.set_xlabel('Tempo (s)')
        ax1.set_ylabel('Setor')
        ax1.set_title('Setor vs Tempo')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Velocidade ao longo do tempo
        ax2 = axes[0, 1]
        ax2.plot(t, [d['v1'] for d in trace], 'b-', label='Navio 1')
        ax2.plot(t, [d['v2'] for d in trace], 'r-', label='Navio 2')
        ax2.set_xlabel('Tempo (s)')
        ax2.set_ylabel('Velocidade (km/s)')
        ax2.set_title('Velocidade vs Tempo')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Posição dentro do setor
        ax3 = axes[1, 0]
        ax3.plot(t, [d['z1'] for d in trace], 'b-', label='Navio 1')
        ax3.plot(t, [d['z2'] for d in trace], 'r-', label='Navio 2')
        ax3.axhline(y=Z_MAX, color='gray', linestyle='--', label='Limite setor')
        ax3.set_xlabel('Tempo (s)')
        ax3.set_ylabel('Posição z (km)')
        ax3.set_title('Posição no Setor vs Tempo')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Sinais do semáforo
        ax4 = axes[1, 1]
        ax4.step(t, [d.get('signal1', 0) for d in trace], 'b-', where='post', label='Semáforo N1')
        ax4.step(t, [d.get('signal2', 0) for d in trace], 'r--', where='post', label='Semáforo N2')
        ax4.set_xlabel('Tempo (s)')
        ax4.set_ylabel('Sinal (0=Verde, 1=Amarelo, 2=Vermelho)')
        ax4.set_title('Sinais do Semáforo')
        ax4.set_yticks([0, 1, 2])
        ax4.set_yticklabels(['Verde', 'Amarelo', 'Vermelho'])
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig


# ============================================================================
# PARTE 6: SISTEMA INSEGURO (Para Comparação)
# ============================================================================

class SistemaInseguro(SistemaHibrido):
    """
    Versão do sistema SEM semáforos - para demonstrar colisões.
    """
    
    def trans_inseguro(self, s: Dict, s_next: Dict) -> BoolRef:
        """
        Transição SEM controlo de semáforo.
        Os navios movem-se livremente, ignorando a presença do outro.
        """
        # === AUTÓMATO NAVIO 1 (SEM SEMÁFORO) ===
        v1_new = self.dynamics(s['v1'], s['s1'], "AB")
        z1_potential = s['z1'] + DT * s['v1']
        next_s1_id = self.get_next_sector_n1(s['s1'], s['path1'])
        
        # Sem semáforo: Avança sempre que chega ao limite
        can_cross1 = And(z1_potential >= Z_MAX, next_s1_id != -1)
        
        trans_n1 = If(can_cross1,
            And(s_next['s1'] == next_s1_id,
                s_next['z1'] == 0.0,
                s_next['v1'] == s['v1'],
                s_next['path1'] == s['path1']),
            And(s_next['s1'] == s['s1'],
                s_next['z1'] == If(z1_potential > Z_MAX, Z_MAX, z1_potential),
                s_next['v1'] == v1_new,
                s_next['path1'] == s['path1'])
        )
        
        # === AUTÓMATO NAVIO 2 (SEM SEMÁFORO) ===
        v2_new = self.dynamics(s['v2'], s['s2'], "BA")
        z2_potential = s['z2'] + DT * s['v2']
        next_s2_id = self.get_next_sector_n2(s['s2'], s['path2'])
        
        can_cross2 = And(z2_potential >= Z_MAX, next_s2_id != -1)
        
        trans_n2 = If(can_cross2,
            And(s_next['s2'] == next_s2_id,
                s_next['z2'] == 0.0,
                s_next['v2'] == s['v2'],
                s_next['path2'] == s['path2']),
            And(s_next['s2'] == s['s2'],
                s_next['z2'] == If(z2_potential > Z_MAX, Z_MAX, z2_potential),
                s_next['v2'] == v2_new,
                s_next['path2'] == s['path2'])
        )
        
        # Semáforo sempre verde (não usado)
        trans_sem = And(
            s_next['t'] == s['t'] + DT,
            s_next['signal1'] == 0,
            s_next['signal2'] == 0
        )
        
        return And(trans_n1, trans_n2, trans_sem)


class VerificadorCompleto(Verificador):
    """Verificador com suporte para sistema seguro e inseguro."""
    
    def __init__(self, sistema_seguro: SistemaHibrido, sistema_inseguro: SistemaInseguro):
        super().__init__(sistema_seguro)
        self.sistema_inseguro = sistema_inseguro
    
    def bmc_inseguro(self, k_max: int = 50, verbose: bool = True) -> Optional[List[Dict]]:
        """BMC para o sistema INSEGURO (sem semáforos)."""
        if verbose:
            print(f"\n{'='*60}")
            print(f"BMC - Sistema INSEGURO (sem semáforos)")
            print(f"{'='*60}")
            print(f"Limite: k={k_max} passos")
        
        solver = Solver()
        states = [self.sistema_inseguro.declare_state(0)]
        solver.add(self.sistema_inseguro.init(states[0]))
        
        # Forçar cenário de conflito: mesma rota
        solver.add(states[0]['path1'] == 0)
        solver.add(states[0]['path2'] == 0)
        solver.add(states[0]['s1'] == 13)  # N1 começa em 13
        solver.add(states[0]['s2'] == 14)  # N2 começa em 14
        
        start_time = time.time()
        
        for k in range(1, k_max + 1):
            states.append(self.sistema_inseguro.declare_state(k))
            solver.add(self.sistema_inseguro.trans_inseguro(states[k-1], states[k]))
            
            solver.push()
            solver.add(self.sistema_inseguro.prop_seguranca_suficiente(states[k]))
            
            if solver.check() == sat:
                elapsed = time.time() - start_time
                model = solver.model()
                s1 = model[states[k]['s1']].as_long()
                
                if verbose:
                    print(f"\n{'!'*60}")
                    print(f"!!! COLISÃO DETETADA NO PASSO k={k} (t={k*DT:.1f}s) !!!")
                    print(f"{'!'*60}")
                    print(f"Setor da colisão: S{s1}")
                    print(f"Tempo de verificação: {elapsed:.2f}s")
                
                return self.extract_trace(model, states, k)
            
            solver.pop()
            
            if verbose and k % 10 == 0:
                print(f"  Passo {k}...")
        
        if verbose:
            print(f"\n✓ Nenhuma colisão encontrada até k={k_max}")
        return None


# ============================================================================
# PARTE 7: PROGRAMA PRINCIPAL
# ============================================================================

def main():
    """Função principal - executa todas as verificações e demonstrações."""
    
    print("""
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║     TRABALHO PRÁTICO 4 - VERIFICAÇÃO DE SISTEMAS HÍBRIDOS                    ║
║                                                                              ║
║     Controlo de Tráfego Marítimo com Verificação Formal                      ║
║                                                                              ║
║     Implementação: 3 Autómatos Híbridos (Navio 1, Navio 2, Semáforo)         ║
║     Verificação: BMC + k-Indução                                             ║
║     Propriedades: Segurança Suficiente + Segurança Forte (Deadlock)          ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
    """)
    
    # Criar sistemas
    sistema_seguro = SistemaHibrido()
    sistema_inseguro = SistemaInseguro()
    verificador = VerificadorCompleto(sistema_seguro, sistema_inseguro)
    visualizador = Visualizador()
    
    # =========================================================================
    # DEMONSTRAÇÃO 1: Sistema INSEGURO (sem semáforos)
    # =========================================================================
    print("\n" + "="*70)
    print("PARTE 1: Sistema SEM controlo de semáforos (INSEGURO)")
    print("="*70)
    print("Neste cenário, os navios movem-se livremente sem coordenação.")
    print("Esperamos encontrar uma COLISÃO.")
    
    trace_inseguro = verificador.bmc_inseguro(k_max=60, verbose=True)
    
    # =========================================================================
    # DEMONSTRAÇÃO 2: Sistema SEGURO (com semáforos)
    # =========================================================================
    print("\n" + "="*70)
    print("PARTE 2: Sistema COM controlo de semáforos (SEGURO)")
    print("="*70)
    print("Neste cenário, o semáforo coordena o acesso aos setores.")
    print("O Navio 1 tem prioridade em caso de conflito.")
    
    trace_seguro = verificador.bmc(k_max=60, safe_mode=True, check_deadlock=True, verbose=True)
    
    # =========================================================================
    # DEMONSTRAÇÃO 3: K-Indução (Opcional - pode demorar)
    # =========================================================================
    print("\n" + "="*70)
    print("PARTE 3: Verificação por K-Indução")
    print("="*70)
    
    try:
        resultado_inducao = verificador.k_inducao(k=3, verbose=True)
    except Exception as e:
        print(f"K-Indução: {e}")
    
    # =========================================================================
    # VISUALIZAÇÃO
    # =========================================================================
    print("\n" + "="*70)
    print("VISUALIZAÇÃO")
    print("="*70)
    
    if trace_inseguro:
        print("\n>>> A gerar animação do sistema INSEGURO (colisão)...")
        ani_inseguro = visualizador.criar_animacao(
            trace_inseguro, 
            titulo="Sistema INSEGURO - Colisão Detetada"
        )
        fig_traj = visualizador.plot_trajetoria(
            trace_inseguro, 
            titulo="Análise do Sistema INSEGURO"
        )
    
    if trace_seguro:
        print("\n>>> A gerar animação do sistema SEGURO (deadlock?)...")
        ani_seguro = visualizador.criar_animacao(
            trace_seguro, 
            titulo="Sistema SEGURO - Análise"
        )
    elif not trace_inseguro:
        print("\n✓ Ambos os sistemas foram verificados!")
        print("  - Sistema inseguro: Sem colisão encontrada (surpreendente)")
        print("  - Sistema seguro: Verificado como seguro")
    
    plt.show()
    
    # =========================================================================
    # RESUMO FINAL
    # =========================================================================
    print("\n" + "="*70)
    print("RESUMO DA VERIFICAÇÃO")
    print("="*70)
    print(f"""
┌─────────────────────────────────────────────────────────────────────┐
│ SISTEMA INSEGURO (sem semáforos):                                   │
│   {"COLISÃO ENCONTRADA ✗" if trace_inseguro else "Nenhuma colisão até o limite ✓":55} │
├─────────────────────────────────────────────────────────────────────┤
│ SISTEMA SEGURO (com semáforos):                                     │
│   {"FALHA ENCONTRADA ✗" if trace_seguro else "VERIFICADO COMO SEGURO ✓":55} │
└─────────────────────────────────────────────────────────────────────┘
    """)


# ============================================================================
# EXECUÇÃO
# ============================================================================

if __name__ == "__main__":
    main()
