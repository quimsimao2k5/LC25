import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from z3 import *
import sys

# ==========================================
# 1. CONFIGURAÇÃO E MODELO Z3 (MAPA FINAL)
# ==========================================

def check_ships_collision_complex():
    print("--- A iniciar Verificação BMC (Mapa Final) ---")
    
    # Parâmetros Físicos
    SIGMA = 1.0   # Atrito
    GAMMA = 2.0   # Aceleração
    DT = 0.1      # Passo de tempo
    Z_MAX = 1.0   # Tamanho do setor (1 km)

    # --- MAPA (Topologia 2-3-2-1-2-3-2) ---
    # Left (A->B):
    #   Col 1: 13, 11
    #   Col 2: 9, 5, 7
    #   Col 3: 1, 3
    #   Col 4: 0 (Bottleneck)
    # Right (B->A):
    #   Col 5: 2, 4
    #   Col 6: 6, 10, 8
    #   Col 7: 14, 12
    
    # --- Definição do Estado ---
    def declare_state(i):
        state = {}
        # Navio 1
        state['s1'] = Int(f's1_{i}') # ID do setor atual
        state['z1'] = Real(f'z1_{i}') 
        state['v1'] = Real(f'v1_{i}')
        state['path1'] = Int(f'path1_{i}') # 0 = Top/Main, 1 = Bottom/Alt
        
        # Navio 2
        state['s2'] = Int(f's2_{i}')
        state['z2'] = Real(f'z2_{i}')
        state['v2'] = Real(f'v2_{i}')
        state['path2'] = Int(f'path2_{i}') # 0 = Top/Main, 1 = Bottom/Alt
        return state

    # --- Estado Inicial ---
    def init(s):
        return And(
            # Navio 1 (A->B): Começa em 13 ou 11
            Or(s['s1'] == 13, s['s1'] == 11),
            s['z1'] == 0.0, s['v1'] == 0.0,
            
            # Navio 2 (B->A): Começa em 14 ou 12
            Or(s['s2'] == 14, s['s2'] == 12),
            s['z2'] == 0.0, s['v2'] == 0.0,
            
            # Escolha de rotas (livre inicialmente)
            Or(s['path1'] == 0, s['path1'] == 1),
            Or(s['path2'] == 0, s['path2'] == 1)
        )

    # --- Dinâmica Física ---
    def dynamics(v, force):
        return v + DT * (force - SIGMA * v)

    # --- Função Auxiliar: Força por Setor ---
    def get_force(s_id):
        # Setores de Aceleração (Início das rotas)
        # Left: 13, 11. Right: 14, 12.
        is_accel = Or(s_id == 13, s_id == 11, s_id == 14, s_id == 12)
        # Força de manutenção para não parar no meio (compensar atrito)
        # Se não for aceleração, dá força suficiente para manter v ~ 1.0 (se SIGMA=1.0, force=1.0)
        return If(is_accel, GAMMA, 1.0)

    # --- Função Auxiliar: Próximo Setor ---
    def get_next_sector_n1(current_s, path_type):
        # Navio 1 viaja A -> B (Left -> Right)
        next_s = -1
        
        # Transições Left Side
        # 13 -> 9 (Top) or 5 (Mid)
        next_s = If(current_s == 13, If(path_type == 0, 9, 5), next_s)
        # 11 -> 7 (Bot) or 5 (Mid)
        next_s = If(current_s == 11, If(path_type == 0, 7, 5), next_s)
        
        # 9 -> 1 (Top)
        next_s = If(current_s == 9, 1, next_s)
        # 7 -> 3 (Bot)
        next_s = If(current_s == 7, 3, next_s)
        # 5 -> 1 (Top) or 3 (Bot)
        next_s = If(current_s == 5, If(path_type == 0, 1, 3), next_s)
        
        # 1 -> 0, 3 -> 0
        next_s = If(Or(current_s == 1, current_s == 3), 0, next_s)
        
        # Transições Right Side (Saindo de 0)
        # 0 -> 2 (Top) or 4 (Bot)
        next_s = If(current_s == 0, If(path_type == 0, 2, 4), next_s)
        
        # 2 -> 6 (Top) or 10 (Mid)
        next_s = If(current_s == 2, If(path_type == 0, 6, 10), next_s)
        # 4 -> 8 (Bot) or 10 (Mid)
        next_s = If(current_s == 4, If(path_type == 0, 8, 10), next_s)
        
        # 6 -> 14, 8 -> 12
        next_s = If(current_s == 6, 14, next_s)
        next_s = If(current_s == 8, 12, next_s)
        # 10 -> 14 (Top) or 12 (Bot)
        next_s = If(current_s == 10, If(path_type == 0, 14, 12), next_s)
        
        return next_s

    def get_next_sector_n2(current_s, path_type):
        # Navio 2 viaja B -> A (Right -> Left) - Inverso
        next_s = -1
        
        # Transições Right Side (Entrando)
        # 14 -> 6 (Top) or 10 (Mid)
        next_s = If(current_s == 14, If(path_type == 0, 6, 10), next_s)
        # 12 -> 8 (Bot) or 10 (Mid)
        next_s = If(current_s == 12, If(path_type == 0, 8, 10), next_s)
        
        # 6 -> 2, 8 -> 4
        next_s = If(current_s == 6, 2, next_s)
        next_s = If(current_s == 8, 4, next_s)
        # 10 -> 2 (Top) or 4 (Bot)
        next_s = If(current_s == 10, If(path_type == 0, 2, 4), next_s)
        
        # 2 -> 0, 4 -> 0
        next_s = If(Or(current_s == 2, current_s == 4), 0, next_s)
        
        # Transições Left Side (Saindo de 0)
        # 0 -> 1 (Top) or 3 (Bot)
        next_s = If(current_s == 0, If(path_type == 0, 1, 3), next_s)
        
        # 1 -> 9 (Top) or 5 (Mid)
        next_s = If(current_s == 1, If(path_type == 0, 9, 5), next_s)
        # 3 -> 7 (Bot) or 5 (Mid)
        next_s = If(current_s == 3, If(path_type == 0, 7, 5), next_s)
        
        # 9 -> 13, 7 -> 11
        next_s = If(current_s == 9, 13, next_s)
        next_s = If(current_s == 7, 11, next_s)
        # 5 -> 13 (Top) or 11 (Bot)
        next_s = If(current_s == 5, If(path_type == 0, 13, 11), next_s)
        
        return next_s

    # --- Transição de Estado (VERSÃO SIMPLES - SEM PRIORIDADE) ---
    def trans(s, s_next):
        # --- Navio 1 ---
        force1 = get_force(s['s1'])
        v1_new = dynamics(s['v1'], force1)
        z1_potential = s['z1'] + DT * s['v1']
        
        next_s1_id = get_next_sector_n1(s['s1'], s['path1'])
        
        # Lógica de Semáforo INGÉNUA: O próximo setor está livre?
        is_next_free_n1 = (next_s1_id != s['s2'])
        
        crossing1 = And(z1_potential >= Z_MAX, next_s1_id != -1, is_next_free_n1)
        at_boundary_n1 = z1_potential >= Z_MAX
        
        move1 = If(crossing1,
                   # Cruzou: Novo setor, z=0
                   And(s_next['s1'] == next_s1_id,
                       s_next['z1'] == 0.0,
                       s_next['v1'] == v1_new,
                       s_next['path1'] == s['path1']),
                   # Não cruzou
                   If(at_boundary_n1,
                      # Bloqueado no limite -> Para
                      And(s_next['s1'] == s['s1'],
                          s_next['z1'] == Z_MAX,
                          s_next['v1'] == 0.0,
                          s_next['path1'] == s['path1']),
                      # Movimento normal dentro do setor
                      And(s_next['s1'] == s['s1'],
                          s_next['z1'] == z1_potential,
                          s_next['v1'] == v1_new,
                          s_next['path1'] == s['path1'])
                   )
                  )

        # --- Navio 2 (MESMA LÓGICA - SEM PRIORIDADE) ---
        force2 = get_force(s['s2'])
        v2_new = dynamics(s['v2'], force2)
        z2_potential = s['z2'] + DT * s['v2']
        
        next_s2_id = get_next_sector_n2(s['s2'], s['path2'])
        
        is_next_free_n2 = (next_s2_id != s['s1'])
        
        crossing2 = And(z2_potential >= Z_MAX, next_s2_id != -1, is_next_free_n2)
        at_boundary_n2 = z2_potential >= Z_MAX
        
        move2 = If(crossing2,
                   And(s_next['s2'] == next_s2_id,
                       s_next['z2'] == 0.0,
                       s_next['v2'] == v2_new,
                       s_next['path2'] == s['path2']),
                   If(at_boundary_n2,
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

        return And(move1, move2)

    # --- Propriedade de Segurança ---
    def collision(s):
        return s['s1'] == s['s2']
        
    def deadlock(s):
        # Verificar qual seria o próximo setor
        next_s1 = get_next_sector_n1(s['s1'], s['path1'])
        next_s2 = get_next_sector_n2(s['s2'], s['path2'])
        
        # CORREÇÃO 2: Só é Deadlock se estiver parado E BLOQUEADO
        # Bloqueado = Próximo setor ocupado (semáforo vermelho) OU fim de mapa
        blocked_1 = Or(next_s1 == -1, next_s1 == s['s2'])
        blocked_2 = Or(next_s2 == -1, next_s2 == s['s1'])

        # Deadlock = Ambos parados, no limite, E incapazes de avançar
        return And(s['v1'] == 0.0, s['z1'] == Z_MAX, blocked_1,
                   s['v2'] == 0.0, s['z2'] == Z_MAX, blocked_2)

    # --- Loop BMC ---
    solver = Solver()
    K_MAX = 100
    
    states = [declare_state(0)]
    solver.add(init(states[0]))
    
    # Forçar cenário de conflito: Ambos escolhem a rota Top/Main (path=0)
    # N1: 13 -> 9 -> 1 -> 0 -> 2 -> 6 -> 14
    # N2: 14 -> 6 -> 2 -> 0 -> 1 -> 9 -> 13
    # Devem colidir em algum lugar no meio (ex: s0, s1, s2)
    solver.add(states[0]['path1'] == 0)
    solver.add(states[0]['path2'] == 0)
    
    # Fixar início para garantir colisão rápida
    solver.add(states[0]['s1'] == 13)
    solver.add(states[0]['s2'] == 14)
    
    for k in range(1, K_MAX + 1):
        states.append(declare_state(k))
        solver.add(trans(states[k-1], states[k]))
        
        solver.push()
        # Verificar Colisão OU Deadlock
        solver.add(Or(collision(states[k]), deadlock(states[k])))
        
        if solver.check() == sat:
            print(f"!!! FALHA DETETADA NO PASSO {k} (Tempo: {k*DT:.1f}s) !!!")
            # Verificar qual foi
            m = solver.model()
            if is_true(m.eval(collision(states[k]))):
                print("Tipo: COLISÃO (Navios no mesmo setor)")
            if is_true(m.eval(deadlock(states[k]))):
                print("Tipo: DEADLOCK (Navios bloqueados mutuamente)")
                s1_val = m[states[k]['s1']].as_long()
                s2_val = m[states[k]['s2']].as_long()
                print(f"   Navio 1 em S{s1_val} (Bloqueado)")
                print(f"   Navio 2 em S{s2_val} (Bloqueado)")
                
            return extract_trace(solver.model(), states, k, DT)
        
        solver.pop()
        if k % 10 == 0: print(f"Passo {k}...")

    print("Nenhuma falha (Colisão ou Deadlock) encontrada.")
    return None

def extract_trace(model, states, k_max, dt):
    trace = []
    for k in range(k_max + 1):
        s1 = model[states[k]['s1']].as_long()
        z1_ref = model[states[k]['z1']]
        z1 = float(z1_ref.numerator_as_long()) / float(z1_ref.denominator_as_long()) if z1_ref is not None else 0.0
        v1_ref = model[states[k]['v1']]
        v1 = float(v1_ref.numerator_as_long()) / float(v1_ref.denominator_as_long()) if v1_ref is not None else 0.0
        
        s2 = model[states[k]['s2']].as_long()
        z2_ref = model[states[k]['z2']]
        z2 = float(z2_ref.numerator_as_long()) / float(z2_ref.denominator_as_long()) if z2_ref is not None else 0.0
        v2_ref = model[states[k]['v2']]
        v2 = float(v2_ref.numerator_as_long()) / float(v2_ref.denominator_as_long()) if v2_ref is not None else 0.0
        
        trace.append({'t': k * dt, 's1': s1, 'z1': z1, 'v1': v1, 's2': s2, 'z2': z2, 'v2': v2})
    return trace

# ==========================================
# 2. VISUALIZAÇÃO (MAPA FINAL)
# ==========================================

def visualizar_traco_complexo(trace):
    if not trace: return

    print("--- A gerar animação (Mapa Final) ---")
    
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Coordenadas dos Setores (Centrado em s0=(0,0))
    # Left:
    #   Col -1: 1 (y=1), 3 (y=-1)
    #   Col -2: 9 (y=2), 5 (y=0), 7 (y=-2)
    #   Col -3: 13 (y=1.5), 11 (y=-1.5)
    # Right:
    #   Col 1: 2 (y=1), 4 (y=-1)
    #   Col 2: 6 (y=2), 10 (y=0), 8 (y=-2)
    #   Col 3: 14 (y=1.5), 12 (y=-1.5)
    
    sector_coords = {
        0: (0, 0),
        # Left
        1: (-1, 0.5), 3: (-1, -0.5),
        9: (-2, 1), 5: (-2, 0), 7: (-2, -1),
        13: (-3, 0.5), 11: (-3, -0.5),
        # Right
        2: (1, 0.5), 4: (1, -0.5),
        6: (2, 1), 10: (2, 0), 8: (2, -1),
        14: (3, 0.5), 12: (3, -0.5)
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
    status_text = ax.text(0, 2.5, "", ha='center', fontsize=10, bbox=dict(facecolor='white'))

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
        
        # Lógica de grafo para visualização (Inferir próximo setor)
        # Precisamos saber o PATH para saber o próximo setor
        # Mas o trace não tem path. Vamos tentar inferir pelo movimento.
        
        # Helper para prever próximo setor baseado na lógica do Z3
        def predict_next(s, is_n1):
            # Simplificação: Assumir Path 0 (Top/Main) para visualização se ambíguo
            # Ou tentar ver se o navio está num setor "Bottom"
            path_guess = 0
            if s in [11, 7, 3, 4, 8, 12]: path_guess = 1 # Bottom sectors
            
            # Usar a lógica do Z3 (reimplementada aqui simplificada)
            if is_n1: # A -> B
                if s == 13: return 9 if path_guess==0 else 5
                if s == 11: return 7 if path_guess==0 else 5 # Wait, path 1 is bottom?
                # Se path=1 (Bottom), 11->7. Se path=0 (Top), 11->5? No, logic was:
                # 11 -> 7 (Bot) or 5 (Mid). Let's assume Path 1 is Bottom.
                if s == 9: return 1
                if s == 7: return 3
                if s == 5: return 1 # Assume Top for Mid
                if s == 1: return 0
                if s == 3: return 0
                if s == 0: return 2 # Assume Top
                if s == 2: return 6
                if s == 4: return 8
                if s == 6: return 14
                if s == 8: return 12
                if s == 10: return 14
            else: # B -> A
                if s == 14: return 6
                if s == 12: return 8
                if s == 6: return 2
                if s == 8: return 4
                if s == 10: return 2
                if s == 2: return 0
                if s == 4: return 0
                if s == 0: return 1
                if s == 1: return 9
                if s == 3: return 7
                if s == 5: return 13
                if s == 9: return 13
                if s == 7: return 11
            return None

        s1_target = predict_next(data['s1'], True)
        s2_target = predict_next(data['s2'], False)
        
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

    ani = animation.FuncAnimation(fig, update, frames=len(trace), interval=300, blit=False)
    plt.show()

if __name__ == "__main__":
    trace = check_ships_collision_complex()
    if trace:
        visualizar_traco_complexo(trace)
