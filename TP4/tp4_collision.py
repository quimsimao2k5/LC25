import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from z3 import *
import sys

# ==========================================
# 1. CONFIGURAÇÃO E MODELO Z3
# ==========================================

def check_ships_collision():
    print("--- A iniciar Verificação BMC com Z3 ---")
    
    # Parâmetros Físicos
    SIGMA = 1.0   # Atrito
    GAMMA = 2.0   # Aceleração
    DT = 0.1      # Passo de tempo
    Z_MAX = 1.0   # Tamanho do setor (1 km)
    NUM_SECTORS = 15
    
    # Caminhos (Linear: 0 a 14)
    # Navio 1: 0 -> 14
    # Navio 2: 14 -> 0
    
    # --- Definição do Estado ---
    def declare_state(i):
        state = {}
        # Navio 1
        state['s1'] = Int(f's1_{i}') # Índice do setor (0..14)
        state['z1'] = Real(f'z1_{i}') # Posição dentro do setor (0.0..1.0)
        state['v1'] = Real(f'v1_{i}') # Velocidade
        # Navio 2
        state['s2'] = Int(f's2_{i}')
        state['z2'] = Real(f'z2_{i}')
        state['v2'] = Real(f'v2_{i}')
        return state

    # --- Estado Inicial ---
    def init(s):
        return And(
            s['s1'] == 0,  s['z1'] == 0.0, s['v1'] == 0.0,
            s['s2'] == 14, s['z2'] == 0.0, s['v2'] == 0.0
        )

    # --- Dinâmica Física (Euler) ---
    def dynamics(v, force):
        # v_next = v + dt * (F - sigma * v)
        return v + DT * (force - SIGMA * v)

    # --- Transição de Estado ---
    def trans(s, s_next):
        # --- Navio 1 (A -> B, s aumenta) ---
        # Força constante (simplificação: quer sempre acelerar)
        force1 = GAMMA 
        v1_new = dynamics(s['v1'], force1)
        z1_new = s['z1'] + DT * s['v1']
        
        # Lógica de Mudança de Setor
        # Se z >= 1.0 e ainda não chegou ao fim (14), avança setor e z=0
        crossing1 = And(s['z1'] >= Z_MAX, s['s1'] < NUM_SECTORS - 1)
        
        move1 = If(crossing1,
                   And(s_next['s1'] == s['s1'] + 1,
                       s_next['z1'] == 0.0,
                       s_next['v1'] == s['v1']), # Mantém v na transição
                   And(s_next['s1'] == s['s1'],
                       s_next['z1'] == z1_new,
                       s_next['v1'] == v1_new)
                  )

        # --- Navio 2 (B -> A, s diminui) ---
        force2 = GAMMA
        v2_new = dynamics(s['v2'], force2)
        z2_new = s['z2'] + DT * s['v2']
        
        # Se z >= 1.0 e ainda não chegou ao fim (0), recua setor e z=0
        crossing2 = And(s['z2'] >= Z_MAX, s['s2'] > 0)
        
        move2 = If(crossing2,
                   And(s_next['s2'] == s['s2'] - 1,
                       s_next['z2'] == 0.0,
                       s_next['v2'] == s['v2']),
                   And(s_next['s2'] == s['s2'],
                       s_next['z2'] == z2_new,
                       s_next['v2'] == v2_new)
                  )

        return And(move1, move2)

    # --- Propriedade de Segurança ---
    def collision(s):
        # Colisão se estiverem no mesmo setor
        return s['s1'] == s['s2']

    # --- Loop BMC ---
    solver = Solver()
    K_MAX = 100 # Limite máximo de passos para procurar
    
    states = [declare_state(0)]
    solver.add(init(states[0]))
    
    for k in range(1, K_MAX + 1):
        # Criar novo estado e transição
        states.append(declare_state(k))
        solver.add(trans(states[k-1], states[k]))
        
        # Verificar se há colisão neste passo
        solver.push()
        solver.add(collision(states[k])) # Queremos encontrar onde isto é VERDADE
        
        if solver.check() == sat:
            print(f"!!! COLISÃO DETETADA NO PASSO {k} (Tempo: {k*DT:.1f}s) !!!")
            model = solver.model()
            return extract_trace(model, states, k, DT)
        
        solver.pop()
        
        if k % 10 == 0:
            print(f"Passo {k}: Seguro...")

    print("Nenhuma colisão encontrada dentro do limite de passos.")
    return None

# ==========================================
# 2. EXTRAÇÃO DE DADOS
# ==========================================

def extract_trace(model, states, k_max, dt):
    trace = []
    for k in range(k_max + 1):
        # Extrair valores do modelo Z3 e converter para float/int python
        s1 = model[states[k]['s1']].as_long()
        z1_ref = model[states[k]['z1']]
        z1 = float(z1_ref.numerator_as_long()) / float(z1_ref.denominator_as_long()) if z1_ref is not None else 0.0
        
        s2 = model[states[k]['s2']].as_long()
        z2_ref = model[states[k]['z2']]
        z2 = float(z2_ref.numerator_as_long()) / float(z2_ref.denominator_as_long()) if z2_ref is not None else 0.0
        
        trace.append({
            't': k * dt,
            's1': s1, 'z1': z1,
            's2': s2, 'z2': z2
        })
    return trace

# ==========================================
# 3. VISUALIZAÇÃO E ANIMAÇÃO
# ==========================================

def visualizar_traco(trace):
    if not trace:
        return

    print("--- A gerar animação ---")
    
    # Configuração do Gráfico
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.set_xlim(-1, 16)
    ax.set_ylim(-1, 2)
    ax.set_aspect('equal')
    ax.set_title("Simulação de Tráfego Marítimo (Z3 Trace)")
    ax.set_xlabel("Setores (km)")
    ax.set_yticks([])
    
    # Desenhar os setores (quadrados)
    sectors = []
    for i in range(15):
        # Desenha retângulo para cada setor
        rect = patches.Rectangle((i, 0), 1, 1, linewidth=1, edgecolor='black', facecolor='lightblue', alpha=0.3)
        ax.add_patch(rect)
        ax.text(i + 0.5, -0.3, f"S{i}", ha='center')
        sectors.append(rect)

    # Elementos móveis (Navios)
    # Navio 1: Azul, Navio 2: Vermelho
    ship1_dot, = ax.plot([], [], 'bo', markersize=10, label='Navio 1 (A->B)')
    ship2_dot, = ax.plot([], [], 'ro', markersize=10, label='Navio 2 (B->A)')
    
    # Texto de estado
    status_text = ax.text(0.5, 1.5, "", ha='center', fontsize=10, bbox=dict(facecolor='white', alpha=0.8))
    
    ax.legend(loc='upper right')

    def init_anim():
        ship1_dot.set_data([], [])
        ship2_dot.set_data([], [])
        status_text.set_text("")
        return ship1_dot, ship2_dot, status_text

    def update(frame):
        data = trace[frame]
        
        # Calcular posição global X para plotar
        # Posição visual = índice do setor + z (progresso dentro do setor)
        pos1_x = data['s1'] + data['z1']
        pos2_x = data['s2'] + data['z2'] # Nota: visualmente z conta da esq para dir
        
        # Navio 1 (y=0.7), Navio 2 (y=0.3) para não se sobreporem visualmente até colidirem
        ship1_dot.set_data([pos1_x], [0.7])
        ship2_dot.set_data([pos2_x], [0.3])
        
        # Atualizar cor do setor se houver colisão
        if data['s1'] == data['s2']:
            sectors[data['s1']].set_facecolor('red')
            status_text.set_color('red')
            status_text.set_weight('bold')
        else:
            # Resetar cores (caso a animação faça loop)
            for s in sectors: s.set_facecolor('lightblue')
            status_text.set_color('black')

        # Texto informativo
        txt = (f"T={data['t']:.1f}s\n"
               f"N1: Setor {data['s1']} (z={data['z1']:.2f})\n"
               f"N2: Setor {data['s2']} (z={data['z2']:.2f})")
        status_text.set_text(txt)
        
        return ship1_dot, ship2_dot, status_text, *sectors

    # Criar animação
    ani = animation.FuncAnimation(fig, update, frames=len(trace), 
                                  init_func=init_anim, blit=False, interval=100, repeat=True)
    
    plt.show()

if __name__ == "__main__":
    trace_data = check_ships_collision()
    if trace_data:
        visualizar_traco(trace_data)
