import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from z3 import *
import sys

# ==========================================
# 1. CONFIGURAÇÃO E MODELO Z3 (COMPLEXO)
# ==========================================

def check_ships_collision_complex():
    print("--- A iniciar Verificação BMC (Sistema Seguro - Semáforos) ---")
    
    # Parâmetros Físicos
    SIGMA = 1.0   # Atrito
    GAMMA = 2.0   # Aceleração
    DT = 0.1      # Passo de tempo
    Z_MAX = 1.0   # Tamanho do setor (1 km)
    
    # --- MAPA (Grafo) ---
    # Via Ímpar: 0 -> 13 -> 11 -> 9 -> 7 -> 5 -> 3 -> 1
    # Via Par:   0 -> 2 -> 4 -> 6 -> 8 -> 10 -> 12 -> 14
    
    # Definir caminhos como listas ordenadas de setores
    PATH_ODD = [0, 13, 11, 9, 7, 5, 3, 1]
    PATH_EVEN = [0, 2, 4, 6, 8, 10, 12, 14]
    
    # Navio 1 (A->B): Pode escolher PATH_ODD ou PATH_EVEN
    # Navio 2 (B->A): Faz o caminho inverso (reverse(PATH_ODD) ou reverse(PATH_EVEN))

    # --- Definição do Estado ---
    def declare_state(i):
        state = {}
        # Navio 1
        state['s1'] = Int(f's1_{i}') # ID do setor atual
        state['z1'] = Real(f'z1_{i}') 
        state['v1'] = Real(f'v1_{i}')
        state['path1'] = Int(f'path1_{i}') # 0 = Odd, 1 = Even (Decisão de rota)
        
        # Navio 2
        state['s2'] = Int(f's2_{i}')
        state['z2'] = Real(f'z2_{i}')
        state['v2'] = Real(f'v2_{i}')
        state['path2'] = Int(f'path2_{i}') # 0 = Odd, 1 = Even
        return state

    # --- Estado Inicial ---
    def init(s):
        return And(
            s['s1'] == 0,  s['z1'] == 0.0, s['v1'] == 0.0, # N1 em s0
            # N2 começa no fim de uma das rotas (ex: s1 ou s14). 
            # Vamos deixar o solver escolher onde N2 começa ou fixar para teste.
            # Para colisão interessante, N2 começa no fim da rota ODD (s1)
            # s['s2'] == 1, s['z2'] == 0.0, s['v2'] == 0.0, # REMOVIDO
            s['z2'] == 0.0, s['v2'] == 0.0, # Mantém z e v
            
            # Inicialmente as rotas podem ser qualquer uma (serão decididas/mantidas)
            Or(s['path1'] == 0, s['path1'] == 1),
            Or(s['path2'] == 0, s['path2'] == 1)
        )

    # --- Dinâmica Física ---
    def dynamics(v, force):
        return v + DT * (force - SIGMA * v)

    # --- Função Auxiliar: Próximo Setor ---
    # Dado um setor atual e o tipo de rota, qual o próximo?
    def get_next_sector_n1(current_s, path_type):
        # Se path_type == 0 (Odd): 0->13->...->1
        # Se path_type == 1 (Even): 0->2->...->14
        
        # Codificar a lógica de transição do grafo no Z3
        # Retorna o ID do próximo setor
        
        # Lógica simplificada para Z3:
        # Se current=0 e path=0 -> next=13
        # Se current=0 e path=1 -> next=2
        # ...
        
        # Construção manual das transições
        next_s = -1 # Default (fim/erro)
        
        # Transições comuns (saindo de 0)
        next_s = If(current_s == 0, If(path_type == 0, 13, 2), next_s)
        
        # Transições Rota Ímpar (A->B)
        pairs_odd = [(13,11), (11,9), (9,7), (7,5), (5,3), (3,1)]
        for curr, nxt in pairs_odd:
            next_s = If(And(path_type == 0, current_s == curr), nxt, next_s)
            
        # Transições Rota Par (A->B)
        pairs_even = [(2,4), (4,6), (6,8), (8,10), (10,12), (12,14)]
        for curr, nxt in pairs_even:
            next_s = If(And(path_type == 1, current_s == curr), nxt, next_s)
            
        return next_s

    def get_next_sector_n2(current_s, path_type):
        # Navio 2 viaja B->A (Inverso)
        # Se path=0 (Odd): 1->3->...->13->0
        # Se path=1 (Even): 14->12->...->2->0
        
        next_s = -1
        
        # Chegada a 0
        next_s = If(Or(current_s == 13, current_s == 2), 0, next_s)
        
        # Rota Ímpar (B->A)
        pairs_odd_rev = [(1,3), (3,5), (5,7), (7,9), (9,11), (11,13)]
        for curr, nxt in pairs_odd_rev:
            next_s = If(And(path_type == 0, current_s == curr), nxt, next_s)
            
        # Rota Par (B->A)
        pairs_even_rev = [(14,12), (12,10), (10,8), (8,6), (6,4), (4,2)]
        for curr, nxt in pairs_even_rev:
            next_s = If(And(path_type == 1, current_s == curr), nxt, next_s)
            
        return next_s

    # --- Transição de Estado ---
    def trans(s, s_next):
        # --- Navio 1 ---
        force1 = GAMMA
        v1_new = dynamics(s['v1'], force1)
        z1_new = s['z1'] + DT * s['v1']
        
        # Determinar próximo setor potencial
        next_s1_id = get_next_sector_n1(s['s1'], s['path1'])
        
        # Condição de mudança: z >= 1.0 E existe próximo setor (next_s1_id != -1)
        crossing1 = And(s['z1'] >= Z_MAX, next_s1_id != -1)
        
        # SEMÁFORO / SEGURANÇA (ATIVADO)
        # Só entra no próximo setor se estiver livre (next_s1_id != s['s2'])
        can_enter1 = And(crossing1, next_s1_id != s['s2'])
        
        move1 = If(can_enter1,
                   And(s_next['s1'] == next_s1_id,
                       s_next['z1'] == 0.0,
                       s_next['v1'] == s['v1'],
                       s_next['path1'] == s['path1']), # Mantém a rota escolhida
                   And(s_next['s1'] == s['s1'],
                       s_next['z1'] == z1_new,
                       s_next['v1'] == v1_new,
                       s_next['path1'] == s['path1'])
                  )

        # --- Navio 2 ---
        force2 = GAMMA
        v2_new = dynamics(s['v2'], force2)
        z2_new = s['z2'] + DT * s['v2']
        
        next_s2_id = get_next_sector_n2(s['s2'], s['path2'])
        crossing2 = And(s['z2'] >= Z_MAX, next_s2_id != -1)
        
        # Segurança para Navio 2 (com Prioridade para Navio 1)
        # Não entra se N1 está lá.
        # E não entra se N1 também está a entrar no mesmo setor (Prioridade N1)
        conflict_entry = And(crossing1, next_s1_id == next_s2_id)
        can_enter2 = And(crossing2, next_s2_id != s['s1'], Not(conflict_entry))
        
        move2 = If(can_enter2,
                   And(s_next['s2'] == next_s2_id,
                       s_next['z2'] == 0.0,
                       s_next['v2'] == s['v2'],
                       s_next['path2'] == s['path2']),
                   And(s_next['s2'] == s['s2'],
                       s_next['z2'] == z2_new,
                       s_next['v2'] == v2_new,
                       s_next['path2'] == s['path2'])
                  )

        return And(move1, move2)

    # --- Propriedade de Segurança ---
    def collision(s):
        return s['s1'] == s['s2']

    # --- Loop BMC ---
    solver = Solver()
    K_MAX = 100
    
    states = [declare_state(0)]
    solver.add(init(states[0]))
    
    # Forçar cenário de conflito: Ambos escolhem a rota ÍMPAR (path=0)
    # Isso deve causar colisão frontal.
    solver.add(states[0]['path1'] == 0)
    solver.add(states[0]['path2'] == 0)
    
    # AJUSTE: Navio 2 começa em s11 (para encontrarem-se em s13)
    # Se começar em s13, eles trocam de lugar (s0<->s13) no mesmo passo
    solver.add(states[0]['s2'] == 11) 
    
    for k in range(1, K_MAX + 1):
        states.append(declare_state(k))
        solver.add(trans(states[k-1], states[k]))
        
        solver.push()
        solver.add(collision(states[k]))
        
        if solver.check() == sat:
            print(f"!!! COLISÃO DETETADA NO PASSO {k} (Tempo: {k*DT:.1f}s) !!!")
            return extract_trace(solver.model(), states, k, DT)
        
        solver.pop()
        if k % 10 == 0: print(f"Passo {k}...")

    print("Nenhuma colisão encontrada.")
    return None

def extract_trace(model, states, k_max, dt):
    trace = []
    for k in range(k_max + 1):
        s1 = model[states[k]['s1']].as_long()
        z1_ref = model[states[k]['z1']]
        z1 = float(z1_ref.numerator_as_long()) / float(z1_ref.denominator_as_long()) if z1_ref is not None else 0.0
        
        s2 = model[states[k]['s2']].as_long()
        z2_ref = model[states[k]['z2']]
        z2 = float(z2_ref.numerator_as_long()) / float(z2_ref.denominator_as_long()) if z2_ref is not None else 0.0
        
        trace.append({'t': k * dt, 's1': s1, 'z1': z1, 's2': s2, 'z2': z2})
    return trace

# ==========================================
# 2. VISUALIZAÇÃO (MAPA COMPLEXO)
# ==========================================

def visualizar_traco_complexo(trace):
    if not trace: return

    print("--- A gerar animação (Mapa Complexo) ---")
    
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(-2, 10)
    ax.set_ylim(-2, 4)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Coordenadas dos Setores para Visualização
    # s0 em (0, 1)
    # Via Ímpar (Cima): 13(1,2), 11(2,2)...
    # Via Par (Baixo): 2(1,0), 4(2,0)...
    
    sector_coords = {}
    sector_coords[0] = (0, 1)
    
    # Ímpares (y=2)
    odd_sectors = [13, 11, 9, 7, 5, 3, 1]
    for i, sec in enumerate(odd_sectors):
        sector_coords[sec] = (i + 1, 2)
        
    # Pares (y=0)
    even_sectors = [2, 4, 6, 8, 10, 12, 14]
    for i, sec in enumerate(even_sectors):
        sector_coords[sec] = (i + 1, 0)
        
    # Desenhar Mapa
    patches_dict = {}
    for sec_id, (x, y) in sector_coords.items():
        rect = patches.Rectangle((x, y), 1, 1, linewidth=1, edgecolor='black', facecolor='lightblue', alpha=0.3)
        ax.add_patch(rect)
        ax.text(x + 0.5, y + 0.5, f"S{sec_id}", ha='center', va='center')
        patches_dict[sec_id] = rect

    # Navios
    ship1_dot, = ax.plot([], [], 'bo', markersize=12, label='Navio 1')
    ship2_dot, = ax.plot([], [], 'ro', markersize=12, label='Navio 2')
    status_text = ax.text(4, 3.5, "", ha='center', fontsize=10, bbox=dict(facecolor='white'))

    def update(frame):
        data = trace[frame]
        
        # Calcular posições (x, y) baseadas no setor e z
        def get_pos(s, z):
            bx, by = sector_coords[s]
            # Se for rota A->B (s0 -> fim), z soma ao x
            # Se for rota B->A (fim -> s0), z subtrai ao x? 
            # Simplificação visual: z sempre soma da esquerda para a direita dentro do quadrado
            # Mas precisamos saber a direção real.
            # Assumindo visualização estática: z vai de 0 a 1 (Esq -> Dir)
            return bx + z, by + 0.5

        x1, y1 = get_pos(data['s1'], data['z1'])
        x2, y2 = get_pos(data['s2'], data['z2'])
        
        ship1_dot.set_data([x1], [y1])
        ship2_dot.set_data([x2], [y2])
        
        # Colisão
        if data['s1'] == data['s2']:
            patches_dict[data['s1']].set_facecolor('red')
            status_text.set_color('red')
        else:
            for p in patches_dict.values(): p.set_facecolor('lightblue')
            status_text.set_color('black')
            
        status_text.set_text(f"T={data['t']:.1f}s | N1: S{data['s1']} | N2: S{data['s2']}")
        return ship1_dot, ship2_dot, status_text, *patches_dict.values()

    ani = animation.FuncAnimation(fig, update, frames=len(trace), interval=100, blit=False)
    plt.show()

if __name__ == "__main__":
    trace = check_ships_collision_complex()
    if trace:
        visualizar_traco_complexo(trace)
