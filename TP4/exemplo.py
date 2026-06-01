from z3 import *

def check_ships_collision():
    # --- Configurações do Problema ---
    # Caminhos dos navios (Sequência de setores ID 0-14)
    # Exemplo base: Navio 1 vai de 0 a 4, Navio 2 vem de 4 a 0 (cruzam-se no meio)
    path1 = [0, 1, 2, 3, 4] 
    path2 = [4, 3, 2, 1, 0] 
    
    SIGMA = 1.0   # Coeficiente de atrito
    GAMMA = 2.0   # Aceleração
    DT = 0.1      # Passo de tempo (delta t) para discretização
    Z_MAX = 1.0   # Tamanho do setor (1 km)

    # --- Definição do Estado ---
    def declare_state(i):
        state = {}
        # Navio 1
        state['s1_idx'] = Int(f's1_idx_{i}') # Índice no array path1 (não o ID do setor diretamente)
        state['z1']     = Real(f'z1_{i}')    # Posição dentro do setor
        state['v1']     = Real(f'v1_{i}')    # Velocidade
        # Navio 2
        state['s2_idx'] = Int(f's2_idx_{i}')
        state['z2']     = Real(f'z2_{i}')
        state['v2']     = Real(f'v2_{i}')
        return state

    # --- Estado Inicial ---
    def init(s):
        return And(
            s['s1_idx'] == 0, s['z1'] == 0.0, s['v1'] == 0.0,
            s['s2_idx'] == 0, s['z2'] == 0.0, s['v2'] == 0.0
        )

    # --- Lógica de Dinâmica (Flow) ---
    def dynamics(v, force):
        # Discretização de Euler: v_next = v + dt * (F - sigma * v)
        return v + DT * (force - SIGMA * v)

    # --- Transição de Estado ---
    def trans(s, s_next):
        # Determinar IDs reais dos setores atuais e próximos potenciais
        # Nota: Usamos If do Z3 para mapear índice -> ID do setor se os caminhos fossem complexos
        # Aqui simplificamos assumindo lógica direta sobre os índices para a verificação
        
        # --- Dinâmica Navio 1 ---
        # Força: Se velocidade baixa, acelera (GAMMA), senão inércia (0)
        force1 = If(s['v1'] < 1.5, GAMMA, 0.0) 
        v1_new = dynamics(s['v1'], force1)
        z1_new = s['z1'] + DT * s['v1']
        
        # Lógica de Mudança de Setor (Navio 1)
        # Condição: Atingiu limite (z >= 1.0) E não chegou ao fim do caminho
        crossing1 = And(s['z1'] >= Z_MAX, s['s1_idx'] < len(path1) - 1)
        
        # SEMÁFORO (Guarda): Só entra no próximo setor se o Navio 2 não estiver lá
        # O próximo setor do Navio 1 é path1[s['s1_idx'] + 1]
        # O setor atual do Navio 2 é path2[s['s2_idx']]
        next_sec_id_1 = path1[0] # Placeholder para lógica de lista Z3, ver abaixo simplificação
        
        # Para Z3 puro, é difícil indexar listas Python dinamicamente nas clausulas.
        # Vamos codificar a colisão explicitamente nas propriedades ou usar lógica fixa.
        # Aqui: A transição ocorre se z >= max.
        
        # Atualização Navio 1
        ship1_move = If(crossing1,
                        And(s_next['s1_idx'] == s['s1_idx'] + 1, # Avança setor
                            s_next['z1'] == 0.0,                 # Reinicia z
                            s_next['v1'] == s['v1']),            # Mantém v
                        And(s_next['s1_idx'] == s['s1_idx'],     # Mantém setor
                            s_next['z1'] == z1_new,              # Avança z
                            s_next['v1'] == v1_new)              # Atualiza v
                       )

        # --- Dinâmica Navio 2 ---
        force2 = If(s['v2'] < 1.5, GAMMA, 0.0)
        v2_new = dynamics(s['v2'], force2)
        z2_new = s['z2'] + DT * s['v2']
        
        crossing2 = And(s['z2'] >= Z_MAX, s['s2_idx'] < len(path2) - 1)
        
        ship2_move = If(crossing2,
                        And(s_next['s2_idx'] == s['s2_idx'] + 1,
                            s_next['z2'] == 0.0,
                            s_next['v2'] == s['v2']),
                        And(s_next['s2_idx'] == s['s2_idx'],
                            s_next['z2'] == z2_new,
                            s_next['v2'] == v2_new)
                       )

        return And(ship1_move, ship2_move)

    # --- Propriedade de Segurança (Safety) ---
    def property_safety(s):
        # Não podem estar no mesmo setor ao mesmo tempo
        # Temos de recuperar o ID real do setor a partir dos índices
        
        # Construção da lógica de 'Select' manual para Z3
        # Se s1_idx == 0 então id = path1[0], etc.
        def get_sector_id(path, idx_var):
            expr = -1 # Valor default inválido
            for i, sector_id in enumerate(path):
                expr = If(idx_var == i, sector_id, expr)
            return expr

        sec1_id = get_sector_id(path1, s['s1_idx'])
        sec2_id = get_sector_id(path2, s['s2_idx'])
        
        # Safe se setores diferentes OU se já terminaram o caminho (opcional)
        return sec1_id != sec2_id

    # --- Loop de Verificação BMC ---
    print("A iniciar verificação BMC...")
    solver = Solver()
    
    # K é o horizonte de tempo (número de passos discretos)
    K = 20 
    
    states = [declare_state(i) for i in range(K + 1)]
    
    # Adicionar estado inicial
    solver.add(init(states[0]))
    
    # Desenrolar transições e verificar propriedade a cada passo
    for k in range(K):
        # Adicionar transição k -> k+1
        solver.add(trans(states[k], states[k+1]))
        
        # Verificar se a propriedade falha no estado k (ou k+1)
        # BMC procura um contra-exemplo (Not Safety)
        solver.push()
        solver.add(Not(property_safety(states[k+1])))
        
        result = solver.check()
        if result == sat:
            print(f"Colisão detetada no passo {k+1}!")
            model = solver.model()
            # Imprimir detalhes do contra-exemplo
            idx1 = model[states[k+1]['s1_idx']].as_long()
            idx2 = model[states[k+1]['s2_idx']].as_long()
            print(f"Navio 1 no índice {idx1} (Setor {path1[idx1]})")
            print(f"Navio 2 no índice {idx2} (Setor {path2[idx2]})")
            return
        else:
            print(f"Passo {k+1}: Seguro.")
        solver.pop()
        
    print(f"Nenhuma colisão encontrada até ao passo {K}.")

if __name__ == "__main__":
    check_ships_collision()

# Lógica de Semáforo:
# Calcular próximo setor
next_sec1 = get_sector_id(path1, s['s1_idx'] + 1)
current_sec2 = get_sector_id(path2, s['s2_idx'])

# Só permite crossing se atingiu o limite E o próximo setor está livre
safe_to_enter = (next_sec1 != current_sec2)
crossing1 = And(s['z1'] >= Z_MAX, s['s1_idx'] < len(path1) - 1, safe_to_enter)