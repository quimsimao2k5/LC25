import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from z3 import *
import sys

# ==========================================
# 1. CONFIGURAÇÃO E MODELO Z3 (MAPA CRUZADO)
# ==========================================

def check_ships_collision_complex():
    print("--- A iniciar Verificação BMC (Mapa Cruzado) ---")
    
    # Parâmetros Físicos
    SIGMA = 1.0   # Atrito
    GAMMA = 2.0   # Aceleração
    DT = 0.1      # Passo de tempo
    Z_MAX = 1.0   # Tamanho do setor (1 km)
    
    # --- MAPA (Cruzamento) ---
    # Path 1 (Horizontal): 13 -> 9 -> 5 -> 1 -> 0 -> 2 -> 6 -> 10 -> 14
    # Path 2 (Vertical):   11 -> 7 -> 3 -> 0 -> 4 -> 8 -> 12
    
    # --- Definição do Estado ---
    def declare_state(i):
        state = {}
        # Navio 1
        state['s1'] = Int(f's1_{i}') # ID do setor atual
        state['z1'] = Real(f'z1_{i}') 
        state['v1'] = Real(f'v1_{i}')
        state['path1'] = Int(f'path1_{i}') # 0 = Path 1 (Horiz), 1 = Path 2 (Vert)
        
        # Navio 2
        state['s2'] = Int(f's2_{i}')
        state['z2'] = Real(f'z2_{i}')
        state['v2'] = Real(f'v2_{i}')
        state['path2'] = Int(f'path2_{i}') # 0 = Path 1 (Horiz), 1 = Path 2 (Vert)
        return state

    # --- Estado Inicial ---
    def init(s):
        return And(
            # Navio 1 (A->B): Começa em 13 (Path 1) ou 11 (Path 2)
            If(s['path1'] == 0, s['s1'] == 13, s['s1'] == 11),
            s['z1'] == 0.0, s['v1'] == 0.0,
            
            # Navio 2 (B->A): Começa em 14 (Path 1) ou 12 (Path 2)
            If(s['path2'] == 0, s['s2'] == 14, s['s2'] == 12),
            s['z2'] == 0.0, s['v2'] == 0.0,
            
            # Escolha de rotas (livre inicialmente)
            Or(s['path1'] == 0, s['path1'] == 1),
            Or(s['path2'] == 0, s['path2'] == 1)
        )

    # --- Dinâmica Física ---
    def dynamics(v, force):
        return v + DT * (force - SIGMA * v)

    # --- Função Auxiliar: Próximo Setor ---
    def get_next_sector_n1(current_s, path_type):
        # Navio 1 viaja A -> B (Esq -> Dir ou Cima -> Baixo)
        next_s = -1
        
        # Path 1 (Horizontal): 13->9->5->1->0->2->6->10->14
        pairs_p1 = [(13,9), (9,5), (5,1), (1,0), (0,2), (2,6), (6,10), (10,14)]
        for curr, nxt in pairs_p1:
            next_s = If(And(path_type == 0, current_s == curr), nxt, next_s)
            
        # Path 2 (Vertical): 11->7->3->0->4->8->12
        pairs_p2 = [(11,7), (7,3), (3,0), (0,4), (4,8), (8,12)]
        for curr, nxt in pairs_p2:
            next_s = If(And(path_type == 1, current_s == curr), nxt, next_s)
            
        return next_s

    def get_next_sector_n2(current_s, path_type):
        # Navio 2 viaja B -> A (Inverso)
        next_s = -1
        
        # Path 1 Inverso: 14->10->6->2->0->1->5->9->13
        pairs_p1_rev = [(14,10), (10,6), (6,2), (2,0), (0,1), (1,5), (5,9), (9,13)]
        for curr, nxt in pairs_p1_rev:
            next_s = If(And(path_type == 0, current_s == curr), nxt, next_s)
            
        # Path 2 Inverso: 12->8->4->0->3->7->11
        pairs_p2_rev = [(12,8), (8,4), (4,0), (0,3), (3,7), (7,11)]
        for curr, nxt in pairs_p2_rev:
            next_s = If(And(path_type == 1, current_s == curr), nxt, next_s)
            
        return next_s

    # --- Transição de Estado ---
    def trans(s, s_next):
        # --- Navio 1 ---
        force1 = GAMMA
        v1_new = dynamics(s['v1'], force1)
        z1_new = s['z1'] + DT * s['v1']
        
        next_s1_id = get_next_sector_n1(s['s1'], s['path1'])
        crossing1 = And(s['z1'] >= Z_MAX, next_s1_id != -1)
        
        move1 = If(crossing1,
                   And(s_next['s1'] == next_s1_id,
                       s_next['z1'] == 0.0,
                       s_next['v1'] == s['v1'],
                       s_next['path1'] == s['path1']),
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
        
        move2 = If(crossing2,
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
    
    # Forçar cenário de conflito: Ambos escolhem a rota 1 (Horizontal)
    # N1: 13->...->14
    # N2: 14->...->13
    # Devem colidir em algum lugar no meio (ex: s0, s1, s2)
    solver.add(states[0]['path1'] == 0)
    solver.add(states[0]['path2'] == 0)
    
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
# 2. VISUALIZAÇÃO (MAPA CRUZADO)
# ==========================================

def visualizar_traco_complexo(trace):
    if not trace: return

    print("--- A gerar animação (Mapa Cruzado) ---")
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-5, 5)
    ax.set_ylim(-4, 4)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Coordenadas dos Setores (Centrado em s0=(0,0))
    # Path 1 (Horizontal): 13(-4), 9(-3), 5(-2), 1(-1), 0(0), 2(1), 6(2), 10(3), 14(4)
    # Path 2 (Vertical):   11(3), 7(2), 3(1), 0(0), 4(-1), 8(-2), 12(-3)
    
    sector_coords = {
        0: (0, 0),
        # Horizontal Left
        1: (-1, 0), 5: (-2, 0), 9: (-3, 0), 13: (-4, 0),
        # Horizontal Right
        2: (1, 0), 6: (2, 0), 10: (3, 0), 14: (4, 0),
        # Vertical Up
        3: (0, 1), 7: (0, 2), 11: (0, 3),
        # Vertical Down
        4: (0, -1), 8: (0, -2), 12: (0, -3)
    }
    
    # Desenhar Mapa
    patches_dict = {}
    for sec_id, (x, y) in sector_coords.items():
        # Desenhar quadrado centrado em (x,y) -> canto inf esq (x-0.5, y-0.5)
        rect = patches.Rectangle((x-0.5, y-0.5), 1, 1, linewidth=1, edgecolor='black', facecolor='lightblue', alpha=0.3)
        ax.add_patch(rect)
        ax.text(x, y, f"S{sec_id}", ha='center', va='center')
        patches_dict[sec_id] = rect

    # Navios
    ship1_dot, = ax.plot([], [], 'bo', markersize=12, label='Navio 1')
    ship2_dot, = ax.plot([], [], 'ro', markersize=12, label='Navio 2')
    status_text = ax.text(0, 3.5, "", ha='center', fontsize=10, bbox=dict(facecolor='white'))

    def get_pos_interpolated(s_curr, z, s_next_id=None):
        if s_curr not in sector_coords: return 0, 0
        x_curr, y_curr = sector_coords[s_curr]
        
        if s_next_id is not None and s_next_id in sector_coords:
            x_next, y_next = sector_coords[s_next_id]
            # Interpolação linear
            x = x_curr + z * (x_next - x_curr)
            y = y_curr + z * (y_next - y_curr)
            return x, y
        else:
            # Se não soubermos o próximo, assumimos centro
            return x_curr, y_curr

    def update(frame):
        data = trace[frame]
        
        # Lógica de grafo para visualização
        def get_target(s, is_n1):
            # Simplificação: Assumir Path 1 ou 2 baseado na posição atual
            # Path 1: 13,9,5,1,0,2,6,10,14
            p1_seq = [13,9,5,1,0,2,6,10,14]
            # Path 2: 11,7,3,0,4,8,12
            p2_seq = [11,7,3,0,4,8,12]
            
            seq = None
            if s in p1_seq: seq = p1_seq
            elif s in p2_seq: seq = p2_seq
            
            if not seq: return None
            
            try:
                idx = seq.index(s)
                if is_n1: # Avança na lista
                    if idx < len(seq) - 1: return seq[idx+1]
                else: # Recua na lista
                    if idx > 0: return seq[idx-1]
            except: pass
            return None

        s1_target = get_target(data['s1'], True)
        s2_target = get_target(data['s2'], False)
        
        x1, y1 = get_pos_interpolated(data['s1'], data['z1'], s1_target)
        x2, y2 = get_pos_interpolated(data['s2'], data['z2'], s2_target)
        
        ship1_dot.set_data([x1], [y1])
        ship2_dot.set_data([x2], [y2])
        
        # Colisão
        if data['s1'] == data['s2']:
            if data['s1'] in patches_dict:
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
