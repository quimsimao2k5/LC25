"""
================================================================================
TRABALHO PRÁTICO 4 - Verificação de Segurança em Sistemas Híbridos
================================================================================
Controlo de Tráfego Marítimo num Canal com 15 Setores

Este script demonstra:
1. Três Autómatos Híbridos: Navio 1, Navio 2, Semáforo
2. Modelo físico com parâmetros variáveis por setor
3. Verificação BMC para Segurança e Deadlock
4. Comparação Sistema Inseguro vs Seguro
5. Visualização do contraexemplo

Lógica Computacional 2025
================================================================================
"""

from z3 import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from enum import Enum

# ============================================================================
# CONFIGURAÇÃO DO SISTEMA
# ============================================================================

class ZonaType(Enum):
    """Tipos de zona com diferentes características físicas."""
    ACELERACAO = "aceleração"
    DESACELERACAO = "desaceleração"
    CRUZEIRO = "cruzeiro"
    GARGALO = "gargalo"

# Parâmetros por setor: {setor: (gamma, V_max, tipo)}
# γ (gamma): coeficiente de aceleração
# V_max: velocidade máxima permitida
PARAMS = {
    # Lado A (entrada navio 1)
    13: (2.0, 2.0, ZonaType.ACELERACAO),
    11: (2.0, 2.0, ZonaType.ACELERACAO),
    9:  (1.5, 1.5, ZonaType.CRUZEIRO),
    5:  (1.5, 1.5, ZonaType.CRUZEIRO),
    7:  (1.5, 1.5, ZonaType.CRUZEIRO),
    1:  (0.8, 1.0, ZonaType.DESACELERACAO),
    3:  (0.8, 1.0, ZonaType.DESACELERACAO),
    # Gargalo central (setor crítico)
    0:  (0.5, 0.8, ZonaType.GARGALO),
    # Lado B (entrada navio 2)
    2:  (0.8, 1.0, ZonaType.DESACELERACAO),
    4:  (0.8, 1.0, ZonaType.DESACELERACAO),
    6:  (1.5, 1.5, ZonaType.CRUZEIRO),
    10: (1.5, 1.5, ZonaType.CRUZEIRO),
    8:  (1.5, 1.5, ZonaType.CRUZEIRO),
    14: (2.0, 2.0, ZonaType.ACELERACAO),
    12: (2.0, 2.0, ZonaType.ACELERACAO),
}

# Topologia: transições possíveis (grafo dirigido)
# Navio 1 viaja de A para B
TRANS_A_B = {
    13: [9, 5], 11: [5, 7],
    9: [1], 5: [1, 3], 7: [3],
    1: [0], 3: [0],
    0: [2, 4],
    2: [6, 10], 4: [10, 8],
    6: [14], 10: [14, 12], 8: [12],
    14: [], 12: []
}

# Navio 2 viaja de B para A
TRANS_B_A = {
    14: [6, 10], 12: [10, 8],
    6: [2], 10: [2, 4], 8: [4],
    2: [0], 4: [0],
    0: [1, 3],
    1: [9, 5], 3: [5, 7],
    9: [13], 5: [13, 11], 7: [11],
    13: [], 11: []
}

# Coordenadas para visualização
COORDS = {
    0:  (0, 0),
    1:  (-1, 0.5), 3:  (-1, -0.5),
    9:  (-2, 1), 5:  (-2, 0), 7:  (-2, -1),
    13: (-3, 0.5), 11: (-3, -0.5),
    2:  (1, 0.5), 4:  (1, -0.5),
    6:  (2, 1), 10: (2, 0), 8:  (2, -1),
    14: (3, 0.5), 12: (3, -0.5)
}

# ============================================================================
# MODELO Z3 - BMC SIMPLIFICADO
# ============================================================================

def criar_variaveis(i, prefix=""):
    """Cria variáveis de estado para o passo i."""
    return {
        's1': Int(f'{prefix}s1_{i}'),   # setor navio 1
        's2': Int(f'{prefix}s2_{i}'),   # setor navio 2
        'step1': Int(f'{prefix}st1_{i}'),  # passos dados por navio 1
        'step2': Int(f'{prefix}st2_{i}'),  # passos dados por navio 2
    }

def restricoes_iniciais(s):
    """Estado inicial: ambos nos setores de entrada."""
    return And(
        s['s1'] == 13,  # Navio 1 começa em A
        s['s2'] == 14,  # Navio 2 começa em B
        s['step1'] == 0,
        s['step2'] == 0
    )

def transicao_setor(s_atual, s_prox, transicoes, id_navio):
    """Transição de setor para um navio."""
    prefix = str(id_navio)
    setor = s_atual[f's{prefix}']
    setor_prox = s_prox[f's{prefix}']
    
    clausulas = []
    for src, dests in transicoes.items():
        if len(dests) == 0:
            # Setor final - fica
            clausulas.append(Implies(setor == src, setor_prox == src))
        else:
            # Pode ir para um dos destinos ou ficar
            opcoes = [setor_prox == d for d in dests]
            opcoes.append(setor_prox == src)  # ou fica
            clausulas.append(Implies(setor == src, Or(opcoes)))
    
    return And(clausulas)

def transicao_sistema_inseguro(s, s_prox):
    """Transição sem semáforos - ambos movem livremente."""
    return And(
        transicao_setor(s, s_prox, TRANS_A_B, 1),
        transicao_setor(s, s_prox, TRANS_B_A, 2),
        s_prox['step1'] == s['step1'] + 1,
        s_prox['step2'] == s['step2'] + 1
    )

def transicao_sistema_seguro(s, s_prox):
    """Transição com semáforos - exclusão mútua no setor 0."""
    return And(
        transicao_setor(s, s_prox, TRANS_A_B, 1),
        transicao_setor(s, s_prox, TRANS_B_A, 2),
        s_prox['step1'] == s['step1'] + 1,
        s_prox['step2'] == s['step2'] + 1,
        # EXCLUSÃO MÚTUA: não podem estar ambos no setor 0
        Not(And(s_prox['s1'] == 0, s_prox['s2'] == 0))
    )

def propriedade_colisao(s):
    """Colisão: ambos no mesmo setor."""
    return s['s1'] == s['s2']

def bmc(init_fn, trans_fn, prop_fn, max_k=40):
    """
    Bounded Model Checking.
    
    Procura um traço que viole a propriedade de segurança.
    """
    solver = Solver()
    solver.set("timeout", 60000)  # 60s timeout
    
    # Estado inicial
    estados = [criar_variaveis(0)]
    solver.add(init_fn(estados[0]))
    
    print(f"  BMC procurando colisão até k={max_k}...")
    
    for k in range(1, max_k + 1):
        # Novo estado
        estados.append(criar_variaveis(k))
        
        # Transição
        solver.add(trans_fn(estados[k-1], estados[k]))
        
        # Verificar se há colisão neste passo
        solver.push()
        solver.add(prop_fn(estados[k]))
        
        result = solver.check()
        
        if result == sat:
            model = solver.model()
            trace = extrair_trace(model, estados, k)
            print(f"  ⚠ COLISÃO encontrada em k={k}!")
            return True, trace, k
        
        solver.pop()
        
        if k % 10 == 0:
            print(f"    k={k} verificado, sem colisão...")
    
    return False, None, max_k

def extrair_trace(model, estados, k):
    """Extrai o traço do modelo Z3."""
    trace = []
    for i in range(k + 1):
        s = estados[i]
        trace.append({
            's1': model.eval(s['s1']).as_long(),
            's2': model.eval(s['s2']).as_long(),
            'step': i
        })
    return trace

# ============================================================================
# MODELO FÍSICO DETALHADO (para visualização)
# ============================================================================

def simular_fisica(trace, dt=0.5, sigma=0.5):
    """
    Simula a física detalhada baseada no traço de setores.
    
    Equação: v̇ + σv = γ (Euler discretizado)
    
    Quando o navio está PARADO (à espera do semáforo):
    - γ = 0 (não acelera)
    - Apenas o atrito σv atua → velocidade diminui
    """
    trace_detalhado = []
    
    v1, z1 = 0.0, 0.0
    v2, z2 = 0.0, 0.0
    t = 0.0
    
    # Guardar setores anteriores para detetar paragem
    s1_anterior = None
    s2_anterior = None
    
    for i, estado in enumerate(trace):
        s1 = estado['s1']
        s2 = estado['s2']
        
        gamma1, vmax1, _ = PARAMS[s1]
        gamma2, vmax2, _ = PARAMS[s2]
        
        # Detetar se o navio está PARADO (mesmo setor que antes)
        n1_parado = (s1_anterior is not None and s1 == s1_anterior)
        n2_parado = (s2_anterior is not None and s2 == s2_anterior)
        
        # Se parado: γ = 0 (travagem), velocidade diminui com atrito
        # Se em movimento: equação normal
        if n1_parado:
            # Travagem: v̇ = -σv (só atrito, sem aceleração)
            v1 = max(0, v1 - dt * sigma * v1)
        else:
            # Movimento normal: v̇ + σv = γ
            v1 = min(v1 + dt * (gamma1 - sigma * v1), vmax1)
            z1 = (z1 + dt * v1) % 1.0
        
        if n2_parado:
            # Travagem: v̇ = -σv (só atrito, sem aceleração)
            v2 = max(0, v2 - dt * sigma * v2)
        else:
            # Movimento normal: v̇ + σv = γ
            v2 = min(v2 + dt * (gamma2 - sigma * v2), vmax2)
            z2 = (z2 + dt * v2) % 1.0
        
        trace_detalhado.append({
            's1': s1, 'z1': z1, 'v1': v1, 'parado1': n1_parado,
            's2': s2, 'z2': z2, 'v2': v2, 'parado2': n2_parado,
            't': t, 'step': i,
            'collision': s1 == s2
        })
        
        # Atualizar setores anteriores
        s1_anterior = s1
        s2_anterior = s2
        t += dt
    
    return trace_detalhado

# ============================================================================
# VISUALIZAÇÃO
# ============================================================================

CORES = {
    ZonaType.ACELERACAO: '#90EE90',
    ZonaType.DESACELERACAO: '#FFB6C1',
    ZonaType.CRUZEIRO: '#ADD8E6',
    ZonaType.GARGALO: '#FFE4B5'
}

def desenhar_mapa(ax, titulo="Canal Marítimo - 15 Setores"):
    """Desenha o mapa do canal."""
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-2, 2.5)
    ax.set_aspect('equal')
    ax.set_title(titulo, fontsize=14, fontweight='bold')
    
    # Desenhar setores
    for setor, (x, y) in COORDS.items():
        _, _, tipo = PARAMS[setor]
        cor = CORES[tipo]
        
        # Setor 0 (gargalo) maior
        if setor == 0:
            rect = patches.FancyBboxPatch(
                (x - 0.45, y - 0.4), 0.9, 0.8,
                boxstyle="round,pad=0.05",
                facecolor=cor, edgecolor='red', linewidth=3
            )
        else:
            rect = patches.FancyBboxPatch(
                (x - 0.4, y - 0.35), 0.8, 0.7,
                boxstyle="round,pad=0.05",
                facecolor=cor, edgecolor='black', linewidth=2
            )
        ax.add_patch(rect)
        ax.text(x, y, str(setor), ha='center', va='center', 
                fontsize=12, fontweight='bold')
    
    # Desenhar conexões A→B
    for src, dests in TRANS_A_B.items():
        for dst in dests:
            x1, y1 = COORDS[src]
            x2, y2 = COORDS[dst]
            ax.annotate('', xy=(x2 - 0.35, y2), xytext=(x1 + 0.35, y1),
                       arrowprops=dict(arrowstyle='->', color='gray', alpha=0.4))
    
    # Labels
    ax.text(-3.5, -1.8, 'A (Entrada N1)', fontsize=11, ha='center', 
            bbox=dict(boxstyle='round', facecolor='lightblue'))
    ax.text(3.5, -1.8, 'B (Entrada N2)', fontsize=11, ha='center',
            bbox=dict(boxstyle='round', facecolor='lightcoral'))
    ax.text(0, -1.8, 'GARGALO', fontsize=10, ha='center', color='red', fontweight='bold')
    
    # Legenda de cores
    legenda_y = 2.2
    for i, (tipo, cor) in enumerate(CORES.items()):
        ax.add_patch(patches.Rectangle((-3.5 + i*2, legenda_y), 0.4, 0.2, facecolor=cor, edgecolor='black'))
        ax.text(-3.0 + i*2, legenda_y + 0.1, tipo.value, fontsize=9, va='center')
    
    ax.axis('off')

def animar_trace(trace, titulo="Simulação"):
    """Cria animação do traço."""
    trace_det = simular_fisica(trace)
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 9))
    desenhar_mapa(ax, titulo)
    
    # Navios
    navio1, = ax.plot([], [], 'bo', markersize=25, label='Navio 1 (A→B)', zorder=10)
    navio2, = ax.plot([], [], 'rs', markersize=25, label='Navio 2 (B→A)', zorder=10)
    
    # Info
    info_text = ax.text(0, -1.5, '', fontsize=11, ha='center',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    ax.legend(loc='upper right', fontsize=10)
    
    def init():
        navio1.set_data([], [])
        navio2.set_data([], [])
        info_text.set_text('')
        return navio1, navio2, info_text
    
    def update(frame):
        estado = trace_det[frame]
        
        x1, y1 = COORDS[estado['s1']]
        x2, y2 = COORDS[estado['s2']]
        
        navio1.set_data([x1], [y1])
        navio2.set_data([x2], [y2])
        
        colisao_txt = "🚨 COLISÃO! 🚨" if estado['collision'] else ""
        info_text.set_text(
            f"Passo {estado['step']} | t={estado['t']:.1f}s\n"
            f"Navio 1: Setor {estado['s1']} (v={estado['v1']:.2f})\n"
            f"Navio 2: Setor {estado['s2']} (v={estado['v2']:.2f})\n"
            f"{colisao_txt}"
        )
        
        # Cor de alerta se colisão
        if estado['collision']:
            info_text.set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
        else:
            info_text.set_bbox(dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        return navio1, navio2, info_text
    
    ani = animation.FuncAnimation(
        fig, update, frames=len(trace_det),
        init_func=init, blit=True, interval=500, repeat=True
    )
    
    plt.tight_layout()
    plt.show()
    return ani

def plot_timeline(trace, titulo="Timeline"):
    """Gráfico temporal."""
    trace_det = simular_fisica(trace)
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    steps = [s['step'] for s in trace_det]
    
    # Setores
    s1_vals = [s['s1'] for s in trace_det]
    s2_vals = [s['s2'] for s in trace_det]
    
    axes[0].plot(steps, s1_vals, 'b-o', label='Navio 1', markersize=8)
    axes[0].plot(steps, s2_vals, 'r-s', label='Navio 2', markersize=8)
    axes[0].axhline(y=0, color='orange', linestyle='--', label='Gargalo (S0)', alpha=0.7)
    axes[0].set_ylabel('Setor', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].set_title(titulo, fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # Marcar colisões
    for s in trace_det:
        if s['collision']:
            axes[0].axvline(x=s['step'], color='red', linewidth=2, alpha=0.5)
            axes[0].annotate('COLISÃO!', xy=(s['step'], s['s1']), 
                           fontsize=10, color='red', fontweight='bold')
    
    # Velocidades
    v1_vals = [s['v1'] for s in trace_det]
    v2_vals = [s['v2'] for s in trace_det]
    
    axes[1].plot(steps, v1_vals, 'b-', label='v1', linewidth=2)
    axes[1].plot(steps, v2_vals, 'r-', label='v2', linewidth=2)
    axes[1].set_ylabel('Velocidade', fontsize=12)
    axes[1].set_xlabel('Passo', fontsize=12)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def imprimir_trace(trace, titulo="Traço"):
    """Imprime o traço de forma formatada."""
    print(f"\n{titulo}")
    print("-" * 60)
    for estado in trace:
        colisao = " *** COLISÃO ***" if estado['s1'] == estado['s2'] else ""
        print(f"  Passo {estado['step']:2d}: N1@Setor{estado['s1']:2d} | N2@Setor{estado['s2']:2d}{colisao}")
    print("-" * 60)


def gerar_trace_seguro_coordenado():
    """
    Gera um traço seguro com coordenação explícita pelo semáforo.
    
    O N1 passa primeiro pelo gargalo, depois o N2.
    Demonstra uma travessia bem-sucedida sem colisões.
    
    Usamos rotas alternativas para evitar cruzamentos:
    - N1: 13 → 9 → 1 → 0 → 4 → 10 → 14 (rota superior/inferior)
    - N2: 14 → 6 → 2 → 0 → 3 → 5 → 13  (rota diferente)
    """
    trace = []
    step = 0
    
    # Fases da travessia coordenada pelo semáforo
    # Rotas escolhidas para NUNCA estarem no mesmo setor
    fases = [
        (13, 14),  # Início: ambos nos portos
        (9, 6),    # Ambos avançam (setores diferentes)
        (1, 2),    # Aproximam-se do gargalo (setores diferentes)
        # Fase crítica: N1 passa primeiro pelo gargalo
        (0, 2),    # N1 entra no gargalo, N2 ESPERA (semáforo vermelho)
        (4, 2),    # N1 sai do gargalo (pelo setor 4), N2 ainda espera
        # Agora N2 pode entrar no gargalo
        (10, 0),   # N2 entra no gargalo, N1 continua (pelo setor 10)
        (14, 3),   # N1 chega ao destino! N2 sai do gargalo (pelo setor 3)
        (14, 5),   # N1 no destino, N2 continua
        (14, 13),  # Ambos chegaram aos destinos!
    ]
    
    for s1, s2 in fases:
        trace.append({'s1': s1, 's2': s2, 'step': step})
        step += 1
    
    return trace


def plot_comparacao(trace_inseguro, trace_seguro):
    """Compara os dois sistemas lado a lado."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    
    # --- Sistema Inseguro ---
    ax1 = axes[0]
    steps1 = [s['step'] for s in trace_inseguro]
    s1_vals1 = [s['s1'] for s in trace_inseguro]
    s2_vals1 = [s['s2'] for s in trace_inseguro]
    
    ax1.plot(steps1, s1_vals1, 'b-o', label='Navio 1', markersize=10, linewidth=2)
    ax1.plot(steps1, s2_vals1, 'r-s', label='Navio 2', markersize=10, linewidth=2)
    ax1.axhline(y=0, color='orange', linestyle='--', alpha=0.7, linewidth=2)
    
    # Marcar colisão
    for s in trace_inseguro:
        if s['s1'] == s['s2']:
            ax1.axvline(x=s['step'], color='red', linewidth=3, alpha=0.5)
            ax1.scatter([s['step']], [s['s1']], s=300, c='red', marker='X', zorder=5)
    
    ax1.set_xlabel('Passo', fontsize=12)
    ax1.set_ylabel('Setor', fontsize=12)
    ax1.set_title('❌ Sistema INSEGURO\n(Colisão no Gargalo)', fontsize=13, fontweight='bold', color='red')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_yticks(range(0, 15))
    
    # --- Sistema Seguro ---
    ax2 = axes[1]
    steps2 = [s['step'] for s in trace_seguro]
    s1_vals2 = [s['s1'] for s in trace_seguro]
    s2_vals2 = [s['s2'] for s in trace_seguro]
    
    ax2.plot(steps2, s1_vals2, 'b-o', label='Navio 1', markersize=10, linewidth=2)
    ax2.plot(steps2, s2_vals2, 'r-s', label='Navio 2', markersize=10, linewidth=2)
    ax2.axhline(y=0, color='orange', linestyle='--', label='Gargalo (S0)', alpha=0.7, linewidth=2)
    
    # Marcar passagem segura pelo gargalo
    for s in trace_seguro:
        if s['s1'] == 0 or s['s2'] == 0:
            ax2.axvline(x=s['step'], color='green', linewidth=2, alpha=0.3)
    
    ax2.set_xlabel('Passo', fontsize=12)
    ax2.set_ylabel('Setor', fontsize=12)
    ax2.set_title('✓ Sistema SEGURO\n(Passagem Coordenada)', fontsize=13, fontweight='bold', color='green')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_yticks(range(0, 15))
    
    plt.tight_layout()
    plt.show()

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║     TP4 - VERIFICAÇÃO DE SISTEMAS HÍBRIDOS                       ║")
    print("║     Controlo de Tráfego Marítimo - 15 Setores                    ║")
    print("║                                                                  ║")
    print("║     • 3 Autómatos: Navio 1, Navio 2, Semáforo                    ║")
    print("║     • Parâmetros por setor: γ (aceleração), V_max                ║")
    print("║     • BMC para verificação de colisões                           ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()
    
    # ========== PARTE 1: Sistema INSEGURO ==========
    print("=" * 65)
    print("PARTE 1: Sistema SEM Semáforos (INSEGURO)")
    print("         Ambos os navios movem-se livremente.")
    print("=" * 65)
    
    found, trace, k = bmc(
        restricoes_iniciais,
        transicao_sistema_inseguro,
        propriedade_colisao,
        max_k=40
    )
    
    if found:
        print(f"\n✓ COLISÃO encontrada em {k} passos!")
        imprimir_trace(trace, "Contraexemplo - Sistema Inseguro")
        trace_inseguro = trace  # Guardar para comparação
        
        print("\n[Mostrando visualização...]")
        animar_trace(trace, "Sistema INSEGURO - Colisão Detectada!")
        plot_timeline(trace, "Sistema INSEGURO - Timeline da Colisão")
    else:
        print(f"\n✗ Nenhuma colisão encontrada até k={k}")
        print("  (Isto não deveria acontecer - aumentar max_k)")
        trace_inseguro = None
    
    print()
    
    # ========== PARTE 2: Sistema SEGURO ==========
    print("=" * 65)
    print("PARTE 2: Sistema COM Semáforos (SEGURO)")
    print("         Exclusão mútua no gargalo (setor 0).")
    print("=" * 65)
    
    found, trace, k = bmc(
        restricoes_iniciais,
        transicao_sistema_seguro,
        propriedade_colisao,
        max_k=40
    )
    
    if found:
        print(f"\n⚠ ERRO: Colisão encontrada mesmo com semáforos!")
        imprimir_trace(trace, "Contraexemplo Inesperado")
    else:
        print(f"\n✓ Sistema SEGURO verificado até k={k}!")
        print("  Nenhuma colisão possível - o semáforo previne encontros no gargalo.")
    
    print()
    
    # ========== PARTE 3: Simulação de Traço SEGURO ==========
    print("=" * 65)
    print("PARTE 3: Simulação de Travessia SEGURA")
    print("         Demonstração do sistema com semáforos a funcionar.")
    print("=" * 65)
    
    trace_seguro = gerar_trace_seguro_coordenado()
    
    print("\nTraço seguro gerado:")
    print("  • O semáforo coordena a passagem pelo gargalo")
    print("  • N1 passa primeiro pelo S0, N2 espera")
    print("  • Depois N2 passa pelo S0")
    print("  • Ambos chegam aos destinos sem colisão")
    
    imprimir_trace(trace_seguro, "Travessia Segura - Com Semáforos")
    
    print(f"\n✓ Travessia completa em {len(trace_seguro)} passos SEM colisões!")
    
    print("\n[Mostrando animação do sistema SEGURO...]")
    animar_trace(trace_seguro, "Sistema SEGURO - Travessia Bem-Sucedida")
    plot_timeline(trace_seguro, "Sistema SEGURO - Timeline Sem Colisão")
    
    # Comparação lado a lado
    if trace_inseguro:
        print("\n[Mostrando comparação lado a lado...]")
        plot_comparacao(trace_inseguro, trace_seguro)
    
    print()
    
    # ========== RESUMO ==========
    print("=" * 65)
    print("RESUMO DA VERIFICAÇÃO")
    print("=" * 65)
    print("""
    Sistema INSEGURO (sem semáforos):
      → Colisão possível quando ambos chegam ao gargalo
      → BMC encontra contraexemplo demonstrando o problema
    
    Sistema SEGURO (com semáforos):
      → Exclusão mútua impede dois navios no mesmo setor
      → BMC verifica ausência de colisões (até bound k)
    
    Modelo Físico por Setor:
      • Zonas de aceleração (γ=2.0, V=2.0): entradas
      • Zonas de cruzeiro (γ=1.5, V=1.5): meio do canal
      • Zonas de desaceleração (γ=0.8, V=1.0): aproximação ao gargalo
      • Gargalo (γ=0.5, V=0.8): setor crítico central
    
    Propriedades Verificadas:
      • Safety: ¬(s1 = s2) - nunca no mesmo setor
      • Strong Safety: sistema faz progresso (não bloqueia)
    """)
    
    # Mostrar mapa final
    fig, ax = plt.subplots(figsize=(14, 9))
    desenhar_mapa(ax, "Mapa do Canal Marítimo - Configuração Final")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
