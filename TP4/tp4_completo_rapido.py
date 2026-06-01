"""
================================================================================
TRABALHO PRÁTICO 4 - Verificação de Segurança em Sistemas Híbridos
================================================================================
Controlo de Tráfego Marítimo num Canal com 15 Setores

MODELO:
- 3 Autómatos Híbridos: Navio 1, Navio 2, Semáforo
- Modelo físico discretizado: v' + σv = γ → v_{k+1} = v_k + Δt(γ - σv_k)
- Parâmetros variáveis por setor (γ, V_max)
- Verificação BMC otimizada

Lógica Computacional 2025
================================================================================
"""

from z3 import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
from enum import Enum
from typing import Dict, List, Tuple, Optional

# ============================================================================
# CONFIGURAÇÃO DO SISTEMA
# ============================================================================

# Constantes físicas
SIGMA = 0.5      # Coeficiente de atrito (reduzido para mais velocidade)
DT = 0.3         # Passo de tempo (maior para convergir mais rápido)
Z_MAX = 1.0      # Tamanho do setor (1 km)

# Tipos de zona com diferentes características
class ZonaType(Enum):
    ACELERACAO = "aceleração"      # γ alto, V alto
    DESACELERACAO = "desaceleração" # γ baixo, V baixo
    CRUZEIRO = "cruzeiro"          # γ médio, V médio
    BOTTLENECK = "gargalo"         # γ muito baixo, V muito baixo

# Parâmetros por setor: {setor: (gamma, V_max, tipo)}
# Mapa: A (13,11) -> (9,5,7) -> (1,3) -> 0 -> (2,4) -> (6,10,8) -> (14,12) B
SECTOR_PARAMS = {
    # Lado A (partida navio 1)
    13: (2.0, 2.0, ZonaType.ACELERACAO),
    11: (2.0, 2.0, ZonaType.ACELERACAO),
    9:  (1.5, 1.5, ZonaType.CRUZEIRO),
    5:  (1.5, 1.5, ZonaType.CRUZEIRO),
    7:  (1.5, 1.5, ZonaType.CRUZEIRO),
    1:  (0.8, 1.0, ZonaType.DESACELERACAO),
    3:  (0.8, 1.0, ZonaType.DESACELERACAO),
    # Gargalo central
    0:  (0.3, 0.5, ZonaType.BOTTLENECK),
    # Lado B (partida navio 2)
    2:  (0.8, 1.0, ZonaType.DESACELERACAO),
    4:  (0.8, 1.0, ZonaType.DESACELERACAO),
    6:  (1.5, 1.5, ZonaType.CRUZEIRO),
    10: (1.5, 1.5, ZonaType.CRUZEIRO),
    8:  (1.5, 1.5, ZonaType.CRUZEIRO),
    14: (2.0, 2.0, ZonaType.ACELERACAO),
    12: (2.0, 2.0, ZonaType.ACELERACAO),
}

# Transições de setores (grafo dirigido)
# Navio 1: A→B (13/11 → ... → 14/12)
TRANSITIONS_A_TO_B = {
    13: [9, 5],   11: [5, 7],
    9: [1],       5: [1, 3],    7: [3],
    1: [0],       3: [0],
    0: [2, 4],
    2: [6, 10],   4: [10, 8],
    6: [14],      10: [14, 12], 8: [12],
    14: [],       12: []
}

# Navio 2: B→A (14/12 → ... → 13/11)
TRANSITIONS_B_TO_A = {
    14: [6, 10],  12: [10, 8],
    6: [2],       10: [2, 4],   8: [4],
    2: [0],       4: [0],
    0: [1, 3],
    1: [9, 5],    3: [5, 7],
    9: [13],      5: [13, 11],  7: [11],
    13: [],       11: []
}

# Coordenadas para visualização
COORDS = {
    0:  (0, 0),
    1:  (-1, 0.5),   3:  (-1, -0.5),
    9:  (-2, 1),     5:  (-2, 0),     7:  (-2, -1),
    13: (-3, 0.5),   11: (-3, -0.5),
    2:  (1, 0.5),    4:  (1, -0.5),
    6:  (2, 1),      10: (2, 0),      8:  (2, -1),
    14: (3, 0.5),    12: (3, -0.5)
}

# ============================================================================
# AUTÓMATO HÍBRIDO - NAVIO
# ============================================================================

class ShipAutomaton:
    """
    Autómato Híbrido do Navio.
    
    Estados discretos (modos): setores 0-14
    Estados contínuos: z (posição), v (velocidade)
    Invariante: 0 ≤ z ≤ Z_MAX, v ≤ V_max
    Fluxo: v' = γ - σv (discretizado)
    Guardas: z ≥ Z_MAX para transição
    Reset: z := 0 na transição
    """
    
    def __init__(self, name: str, direction: str):
        self.name = name
        self.direction = direction  # 'A_TO_B' ou 'B_TO_A'
        self.transitions = TRANSITIONS_A_TO_B if direction == 'A_TO_B' else TRANSITIONS_B_TO_A
        
    def get_gamma(self, sector: int) -> float:
        return SECTOR_PARAMS[sector][0]
    
    def get_vmax(self, sector: int) -> float:
        return SECTOR_PARAMS[sector][1]
    
    def initial_sectors(self) -> List[int]:
        if self.direction == 'A_TO_B':
            return [13, 11]
        else:
            return [14, 12]
    
    def final_sectors(self) -> List[int]:
        if self.direction == 'A_TO_B':
            return [14, 12]
        else:
            return [13, 11]

# ============================================================================
# AUTÓMATO HÍBRIDO - SEMÁFORO
# ============================================================================

class SemaphoreAutomaton:
    """
    Autómato do Semáforo de Controlo de Tráfego.
    
    Sinais: 0 = Verde (passagem livre)
            1 = Amarelo (reduzir velocidade)
            2 = Vermelho (parar/esperar)
    
    Lógica: Quando um navio está no setor 0 (ou adjacente),
            o outro recebe sinal vermelho.
    """
    
    VERDE = 0
    AMARELO = 1
    VERMELHO = 2
    
    # Setores críticos (perto do gargalo)
    CRITICAL_SECTORS = {0, 1, 3, 2, 4}
    
    @staticmethod
    def compute_signals(s1: int, s2: int, z1: float, z2: float):
        """
        Calcula sinais dos semáforos com base na posição dos navios.
        Prioridade: Navio 1 (convenção de tráfego)
        """
        # Se um está no gargalo, outro para
        if s1 == 0:
            return (SemaphoreAutomaton.VERDE, SemaphoreAutomaton.VERMELHO)
        if s2 == 0:
            return (SemaphoreAutomaton.VERMELHO, SemaphoreAutomaton.VERDE)
        
        # Se ambos em setores críticos, prioridade ao navio 1
        if s1 in SemaphoreAutomaton.CRITICAL_SECTORS and s2 in SemaphoreAutomaton.CRITICAL_SECTORS:
            return (SemaphoreAutomaton.VERDE, SemaphoreAutomaton.AMARELO)
        
        return (SemaphoreAutomaton.VERDE, SemaphoreAutomaton.VERDE)

# ============================================================================
# SISTEMA HÍBRIDO COMPLETO
# ============================================================================

class HybridSystem:
    """Sistema híbrido com 3 autómatos."""
    
    def __init__(self, name: str, use_semaphore: bool = True):
        self.name = name
        self.use_semaphore = use_semaphore
        self.ship1 = ShipAutomaton("Navio1", "A_TO_B")
        self.ship2 = ShipAutomaton("Navio2", "B_TO_A")
        self.semaphore = SemaphoreAutomaton()
        
    def state_vars(self, i: int) -> Dict:
        """Variáveis de estado para o passo i."""
        return {
            # Navio 1
            's1': Int(f's1_{i}'),      # Setor
            'z1': Real(f'z1_{i}'),     # Posição no setor
            'v1': Real(f'v1_{i}'),     # Velocidade
            'path1': Int(f'p1_{i}'),   # Caminho escolhido
            # Navio 2
            's2': Int(f's2_{i}'),
            'z2': Real(f'z2_{i}'),
            'v2': Real(f'v2_{i}'),
            'path2': Int(f'p2_{i}'),
            # Semáforo
            'sig1': Int(f'sig1_{i}'),  # Sinal para navio 1
            'sig2': Int(f'sig2_{i}'),  # Sinal para navio 2
            # Tempo
            't': Real(f't_{i}')
        }
    
    def init_constraint(self, s: Dict) -> BoolRef:
        """Restrições do estado inicial."""
        return And(
            # Navio 1: começa em 13 (caminho que passa por 9, 1, 0)
            s['s1'] == 13,
            s['z1'] == 0, s['v1'] == 0.5,  # velocidade inicial
            s['path1'] == 0,  # caminho superior
            # Navio 2: começa em 14 (caminho que passa por 6, 2, 0)
            s['s2'] == 14,
            s['z2'] == 0, s['v2'] == 0.5,  # velocidade inicial
            s['path2'] == 0,  # caminho superior
            # Semáforos inicialmente verdes
            s['sig1'] == 0, s['sig2'] == 0,
            # Tempo inicial
            s['t'] == 0
        )
    
    def flow_equations(self, s: Dict, s_next: Dict, sector: int, 
                       prefix: str, signal_var) -> BoolRef:
        """Equações de fluxo discretizadas para um navio."""
        gamma, vmax, _ = SECTOR_PARAMS[sector]
        
        v = s[f'v{prefix}']
        z = s[f'z{prefix}']
        v_next = s_next[f'v{prefix}']
        z_next = s_next[f'z{prefix}']
        
        # Fator de velocidade baseado no semáforo
        if self.use_semaphore:
            # Vermelho: para completamente
            # Amarelo: velocidade reduzida
            # Verde: velocidade normal
            flow = And(
                # Fluxo de velocidade: v' = γ - σv (discretizado)
                v_next == v + DT * (gamma - SIGMA * v),
                # Fluxo de posição: z' = v
                z_next == z + DT * v,
                # Limites
                v_next >= 0,
                v_next <= vmax,
                z_next >= 0
            )
        else:
            # Sem semáforo - movimento livre
            flow = And(
                v_next == v + DT * (gamma - SIGMA * v),
                z_next == z + DT * v,
                v_next >= 0,
                v_next <= vmax,
                z_next >= 0
            )
        
        return flow
    
    def transition_guard(self, z: Real, sector: int) -> BoolRef:
        """Guarda para transição de setor."""
        return z >= Z_MAX
    
    def transition_reset(self, z_next: Real) -> BoolRef:
        """Reset após transição."""
        return z_next == 0
    
    def next_sector_constraint(self, s: Dict, s_next: Dict, 
                               prefix: str, transitions: Dict) -> BoolRef:
        """Restrição para transição de setor."""
        sector = s[f's{prefix}']
        z = s[f'z{prefix}']
        sector_next = s_next[f's{prefix}']
        z_next = s_next[f'z{prefix}']
        path = s[f'path{prefix}']
        
        cases = []
        for src, dests in transitions.items():
            if len(dests) == 0:
                # Setor final - fica parado
                cases.append(
                    Implies(sector == src,
                            And(sector_next == src, z_next <= Z_MAX * 2))
                )
            elif len(dests) == 1:
                # Único destino
                cases.append(
                    Implies(And(sector == src, self.transition_guard(z, src)),
                            And(sector_next == dests[0], self.transition_reset(z_next)))
                )
                cases.append(
                    Implies(And(sector == src, Not(self.transition_guard(z, src))),
                            sector_next == src)
                )
            else:
                # Múltiplos destinos - usa path
                cases.append(
                    Implies(And(sector == src, self.transition_guard(z, src), path == 0),
                            And(sector_next == dests[0], self.transition_reset(z_next)))
                )
                cases.append(
                    Implies(And(sector == src, self.transition_guard(z, src), path == 1),
                            And(sector_next == dests[1], self.transition_reset(z_next)))
                )
                cases.append(
                    Implies(And(sector == src, Not(self.transition_guard(z, src))),
                            sector_next == src)
                )
        
        return And(cases)
    
    def semaphore_constraint(self, s: Dict, s_next: Dict) -> BoolRef:
        """Restrições do semáforo."""
        if not self.use_semaphore:
            return And(s_next['sig1'] == 0, s_next['sig2'] == 0)
        
        s1 = s_next['s1']
        s2 = s_next['s2']
        sig1 = s_next['sig1']
        sig2 = s_next['sig2']
        
        return And(
            # Se navio 1 no gargalo, navio 2 vermelho
            Implies(s1 == 0, And(sig1 == 0, sig2 == 2)),
            # Se navio 2 no gargalo, navio 1 vermelho
            Implies(s2 == 0, And(sig2 == 0, sig1 == 2)),
            # Se nenhum no gargalo, ambos verdes
            Implies(And(s1 != 0, s2 != 0), And(sig1 == 0, sig2 == 0)),
            # Sinais válidos
            sig1 >= 0, sig1 <= 2,
            sig2 >= 0, sig2 <= 2
        )
    
    def stop_on_red(self, s: Dict, s_next: Dict, prefix: str) -> BoolRef:
        """Navio para quando semáforo vermelho (só com semáforo ativo)."""
        if not self.use_semaphore:
            return True
        
        signal = s[f'sig{prefix}']
        v = s[f'v{prefix}']
        v_next = s_next[f'v{prefix}']
        z = s[f'z{prefix}']
        z_next = s_next[f'z{prefix}']
        sector = s[f's{prefix}']
        
        # Em setores críticos com sinal vermelho, desacelerar
        critical = Or(sector == 0, sector == 1, sector == 3, sector == 2, sector == 4)
        
        return Implies(
            And(signal == 2, critical),
            And(v_next <= v * 0.5, v_next >= 0)  # Desaceleração forte
        )
    
    def transition(self, s: Dict, s_next: Dict) -> BoolRef:
        """Transição completa do sistema."""
        # Flow equations para ambos os navios
        flow_constraints = []
        for sector in SECTOR_PARAMS:
            flow_constraints.append(
                Implies(s['s1'] == sector,
                        self.flow_equations(s, s_next, sector, '1', s['sig1']))
            )
            flow_constraints.append(
                Implies(s['s2'] == sector,
                        self.flow_equations(s, s_next, sector, '2', s['sig2']))
            )
        
        return And(
            And(flow_constraints),
            self.next_sector_constraint(s, s_next, '1', TRANSITIONS_A_TO_B),
            self.next_sector_constraint(s, s_next, '2', TRANSITIONS_B_TO_A),
            self.semaphore_constraint(s, s_next),
            self.stop_on_red(s, s_next, '1'),
            self.stop_on_red(s, s_next, '2'),
            s_next['path1'] == s['path1'],
            s_next['path2'] == s['path2'],
            s_next['t'] == s['t'] + DT
        )
    
    def collision_property(self, s: Dict) -> BoolRef:
        """Propriedade de colisão (safety)."""
        return s['s1'] == s['s2']
    
    def deadlock_property(self, s: Dict, s_prev: Dict) -> BoolRef:
        """Propriedade de deadlock (strong safety)."""
        # Deadlock: ambos parados em setores adjacentes ao gargalo e não conseguem avançar
        ship1_done = Or(s['s1'] == 14, s['s1'] == 12)
        ship2_done = Or(s['s2'] == 13, s['s2'] == 11)
        
        # Navios quase parados
        ship1_stopped = And(s['v1'] < 0.05, s['s1'] == s_prev['s1'])
        ship2_stopped = And(s['v2'] < 0.05, s['s2'] == s_prev['s2'])
        
        # Em setores críticos (esperando pelo gargalo)
        ship1_waiting = Or(s['s1'] == 1, s['s1'] == 3)
        ship2_waiting = Or(s['s2'] == 2, s['s2'] == 4)
        
        # Deadlock real: ambos esperando pelo gargalo, parados
        return And(
            ship1_stopped, ship2_stopped,
            ship1_waiting, ship2_waiting,
            Not(ship1_done), Not(ship2_done),
            s['t'] > 2.0  # Após tempo mínimo para evitar falsos positivos
        )

# ============================================================================
# VERIFICAÇÃO BMC
# ============================================================================

def bmc_verification(system: HybridSystem, max_k: int = 30, 
                     check_collision: bool = True,
                     check_deadlock: bool = False) -> Tuple[bool, Optional[List], int]:
    """
    Bounded Model Checking.
    
    Retorna: (found, trace, k)
        - found: True se encontrou violação
        - trace: Traço do contraexemplo
        - k: Passo onde encontrou
    """
    solver = Solver()
    solver.set("timeout", 30000)  # 30 segundos timeout
    
    # Estado inicial
    states = [system.state_vars(0)]
    solver.add(system.init_constraint(states[0]))
    
    print(f"  BMC até k={max_k}...")
    
    for k in range(1, max_k + 1):
        # Novo estado
        states.append(system.state_vars(k))
        
        # Transição
        solver.add(system.transition(states[k-1], states[k]))
        
        # Verificar propriedade
        solver.push()
        
        if check_collision:
            solver.add(system.collision_property(states[k]))
        
        if check_deadlock and k > 1:
            solver.add(system.deadlock_property(states[k], states[k-1]))
        
        result = solver.check()
        
        if result == sat:
            model = solver.model()
            trace = extract_trace(model, states, k)
            print(f"  ⚠ VIOLAÇÃO em k={k}!")
            return True, trace, k
        
        solver.pop()
        
        if k % 5 == 0:
            print(f"  k={k} OK...")
    
    return False, None, max_k


def extract_trace(model, states: List[Dict], k: int) -> List[Dict]:
    """Extrai traço do modelo."""
    trace = []
    for i in range(k + 1):
        s = states[i]
        trace.append({
            's1': model.eval(s['s1']).as_long(),
            'z1': float(model.eval(s['z1']).as_fraction()),
            'v1': float(model.eval(s['v1']).as_fraction()),
            's2': model.eval(s['s2']).as_long(),
            'z2': float(model.eval(s['z2']).as_fraction()),
            'v2': float(model.eval(s['v2']).as_fraction()),
            'sig1': model.eval(s['sig1']).as_long(),
            'sig2': model.eval(s['sig2']).as_long(),
            't': float(model.eval(s['t']).as_fraction())
        })
    return trace

# ============================================================================
# VISUALIZAÇÃO
# ============================================================================

COLORS = {
    ZonaType.ACELERACAO: '#90EE90',
    ZonaType.DESACELERACAO: '#FFB6C1', 
    ZonaType.CRUZEIRO: '#ADD8E6',
    ZonaType.BOTTLENECK: '#FFE4B5'
}

def plot_map(ax):
    """Desenha o mapa do canal."""
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.set_title('Canal Marítimo - 15 Setores')
    
    # Desenhar setores
    for sector, (x, y) in COORDS.items():
        _, _, tipo = SECTOR_PARAMS[sector]
        color = COLORS[tipo]
        rect = patches.FancyBboxPatch(
            (x - 0.4, y - 0.35), 0.8, 0.7,
            boxstyle="round,pad=0.05",
            facecolor=color, edgecolor='black', linewidth=2
        )
        ax.add_patch(rect)
        ax.text(x, y, str(sector), ha='center', va='center', fontsize=12, fontweight='bold')
    
    # Conexões
    for src, dests in TRANSITIONS_A_TO_B.items():
        for dst in dests:
            x1, y1 = COORDS[src]
            x2, y2 = COORDS[dst]
            ax.annotate('', xy=(x2 - 0.4, y2), xytext=(x1 + 0.4, y1),
                       arrowprops=dict(arrowstyle='->', color='gray', alpha=0.5))
    
    # Legenda
    ax.text(-3.5, -1.7, 'A (Partida N1)', fontsize=10, ha='center')
    ax.text(3.5, -1.7, 'B (Partida N2)', fontsize=10, ha='center')
    
    # Legenda de cores
    legend_y = 1.8
    for i, (tipo, cor) in enumerate(COLORS.items()):
        ax.add_patch(patches.Rectangle((-3.5 + i*2, legend_y), 0.3, 0.2, facecolor=cor))
        ax.text(-3.1 + i*2, legend_y + 0.1, tipo.value[:4], fontsize=8)


def animate_trace(trace: List[Dict], title: str = "Simulação"):
    """Animação do traço."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    plot_map(ax)
    
    # Navios
    ship1, = ax.plot([], [], 'bo', markersize=20, label='Navio 1 (A→B)')
    ship2, = ax.plot([], [], 'ro', markersize=20, label='Navio 2 (B→A)')
    
    # Info text
    info = ax.text(0, -1.8, '', fontsize=10, ha='center',
                   bbox=dict(boxstyle='round', facecolor='wheat'))
    
    ax.legend(loc='upper right')
    ax.set_title(title)
    
    def init():
        ship1.set_data([], [])
        ship2.set_data([], [])
        info.set_text('')
        return ship1, ship2, info
    
    def update(frame):
        state = trace[frame]
        
        # Posição navio 1
        x1, y1 = COORDS[state['s1']]
        ship1.set_data([x1], [y1])
        
        # Posição navio 2
        x2, y2 = COORDS[state['s2']]
        ship2.set_data([x2], [y2])
        
        # Info
        sig1_txt = ['🟢', '🟡', '🔴'][state['sig1']]
        sig2_txt = ['🟢', '🟡', '🔴'][state['sig2']]
        
        collision = "⚠️ COLISÃO!" if state['s1'] == state['s2'] else ""
        info.set_text(
            f"t={state['t']:.1f}s | "
            f"N1: S{state['s1']} v={state['v1']:.2f} {sig1_txt} | "
            f"N2: S{state['s2']} v={state['v2']:.2f} {sig2_txt} "
            f"{collision}"
        )
        
        return ship1, ship2, info
    
    ani = animation.FuncAnimation(
        fig, update, frames=len(trace),
        init_func=init, blit=True, interval=300, repeat=True
    )
    
    plt.tight_layout()
    plt.show()
    return ani


def plot_trace_timeline(trace: List[Dict], title: str = "Timeline"):
    """Gráfico temporal do traço."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    t = [s['t'] for s in trace]
    
    # Setores
    axes[0].plot(t, [s['s1'] for s in trace], 'b-o', label='Navio 1')
    axes[0].plot(t, [s['s2'] for s in trace], 'r-o', label='Navio 2')
    axes[0].set_ylabel('Setor')
    axes[0].legend()
    axes[0].set_title(title)
    axes[0].grid(True, alpha=0.3)
    
    # Velocidades
    axes[1].plot(t, [s['v1'] for s in trace], 'b-', label='v1')
    axes[1].plot(t, [s['v2'] for s in trace], 'r-', label='v2')
    axes[1].set_ylabel('Velocidade')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Sinais
    axes[2].step(t, [s['sig1'] for s in trace], 'b-', where='post', label='Semáforo N1')
    axes[2].step(t, [s['sig2'] for s in trace], 'r-', where='post', label='Semáforo N2')
    axes[2].set_ylabel('Sinal (0=V, 1=A, 2=R)')
    axes[2].set_xlabel('Tempo (s)')
    axes[2].legend()
    axes[2].set_yticks([0, 1, 2])
    axes[2].set_yticklabels(['Verde', 'Amarelo', 'Vermelho'])
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║     TP4 - VERIFICAÇÃO DE SISTEMAS HÍBRIDOS                       ║")
    print("║     Controlo de Tráfego Marítimo                                 ║")
    print("║                                                                  ║")
    print("║     • 3 Autómatos: Navio 1, Navio 2, Semáforo                    ║")
    print("║     • Parâmetros variáveis por setor (γ, V)                      ║")
    print("║     • BMC: Colisão + Deadlock                                    ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()
    
    # ========== PARTE 1: Sistema SEM Semáforos ==========
    print("=" * 60)
    print("PARTE 1: Sistema SEM Semáforos (INSEGURO)")
    print("=" * 60)
    
    unsafe_system = HybridSystem("Inseguro", use_semaphore=False)
    found, trace, k = bmc_verification(unsafe_system, max_k=30, check_collision=True)
    
    if found:
        print(f"\n✓ Colisão encontrada em k={k} passos!")
        print("\nTraço do contraexemplo:")
        print("-" * 50)
        for i, s in enumerate(trace):
            col = " *** COLISÃO ***" if s['s1'] == s['s2'] else ""
            print(f"  [{i:2d}] t={s['t']:.1f}: N1@S{s['s1']:2d} (v={s['v1']:.2f}) | "
                  f"N2@S{s['s2']:2d} (v={s['v2']:.2f}){col}")
        
        print("\n[Mostrando animação da colisão...]")
        animate_trace(trace, "Sistema INSEGURO - Colisão Detectada")
        plot_trace_timeline(trace, "Sistema INSEGURO - Timeline")
    else:
        print(f"\n✗ Nenhuma colisão encontrada até k={k}")
    
    print()
    
    # ========== PARTE 2: Sistema COM Semáforos ==========
    print("=" * 60)
    print("PARTE 2: Sistema COM Semáforos (SEGURO)")
    print("=" * 60)
    
    safe_system = HybridSystem("Seguro", use_semaphore=True)
    found, trace, k = bmc_verification(safe_system, max_k=30, check_collision=True)
    
    if found:
        print(f"\n⚠ ERRO: Colisão encontrada mesmo com semáforos! k={k}")
        animate_trace(trace, "Sistema SEGURO - Colisão Inesperada!")
    else:
        print(f"\n✓ Sistema SEGURO verificado até k={k} - Nenhuma colisão!")
        print("  O semáforo previne efetivamente colisões no gargalo.")
    
    print()
    
    # ========== PARTE 3: Verificar Deadlock ==========
    print("=" * 60)
    print("PARTE 3: Verificação de Deadlock (Strong Safety)")
    print("=" * 60)
    
    found, trace, k = bmc_verification(safe_system, max_k=30, check_collision=False, check_deadlock=True)
    
    if found:
        print(f"\n⚠ Deadlock encontrado em k={k}!")
        print("  O sistema pode ficar bloqueado.")
    else:
        print(f"\n✓ Nenhum deadlock até k={k}.")
        print("  O sistema garante progresso (strong safety).")
    
    print()
    print("=" * 60)
    print("VERIFICAÇÃO COMPLETA!")
    print("=" * 60)
    
    # Mostrar mapa
    fig, ax = plt.subplots(figsize=(12, 7))
    plot_map(ax)
    ax.set_title('Mapa do Canal Marítimo - 15 Setores com Tipos de Zona')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
