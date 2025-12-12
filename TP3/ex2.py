from pysmt.shortcuts import *
from pysmt.typing import INT

# --- 1. Definição do Sistema (SFOTS com Inteiros Matemáticos) ---

def declare(i):
    state = {}
    # Usamos INT para provar a propriedade matemática sem erros de overflow
    state['a']  = Symbol(f"a_{i}", INT)
    state['b']  = Symbol(f"b_{i}", INT)
    state['r']  = Symbol(f"r_{i}", INT)
    state['rp'] = Symbol(f"rp_{i}", INT) # r'
    state['s']  = Symbol(f"s_{i}", INT)
    state['sp'] = Symbol(f"sp_{i}", INT) # s'
    state['t']  = Symbol(f"t_{i}", INT)
    state['tp'] = Symbol(f"tp_{i}", INT) # t'
    return state

def init(state):
    # Inputs > 0 e inicialização conforme o enunciado
    # r=a, r'=b, s=1, s'=0, t=0, t'=1
    return And(
        state['a'] > Int(0),
        state['b'] > Int(0),
        Equals(state['r'], state['a']),
        Equals(state['rp'], state['b']),
        Equals(state['s'], Int(1)),
        Equals(state['sp'], Int(0)),
        Equals(state['t'], Int(0)),
        Equals(state['tp'], Int(1))
    )

def trans(curr, prox):
    # O loop ocorre enquanto r' != 0
    cond_loop = Not(Equals(curr['rp'], Int(0)))
    
    # Cálculos Matemáticos (q = r div r')
    q = Div(curr['r'], curr['rp']) 
    
    # Atualizações conforme o código do Problema 1:
    # r_new = r'
    # r'_new = r - q * r'
    # s_new = s'
    # s'_new = s - q * s'
    # t_new = t'
    # t'_new = t - q * t'
    
    evolution = And(
        Equals(prox['r'],  curr['rp']),
        Equals(prox['rp'], curr['r'] - (q * curr['rp'])),
        
        Equals(prox['s'],  curr['sp']),
        Equals(prox['sp'], curr['s'] - (q * curr['sp'])),
        
        Equals(prox['t'],  curr['tp']),
        Equals(prox['tp'], curr['t'] - (q * curr['tp'])),
        
        # As constantes a e b mantêm-se
        Equals(prox['a'], curr['a']),
        Equals(prox['b'], curr['b'])
    )
    
    # A transição de estado só é válida se a guarda do loop for verdadeira
    return And(cond_loop, evolution)

def inv_bezout(state):
    # Invariante a provar: a*s + b*t = r
    term1 = state['a'] * state['s']
    term2 = state['b'] * state['t']
    return Equals(term1 + term2, state['r'])

# --- 2. Motor de k-Indução (Vindo da Ficha 8) ---

def kinduction_always(declare, init, trans, inv, k):
    print(f"--- A verificar com k={k} ---")
    with Solver(name="z3") as s:
        
        # === CASO BASE ===
        # Verificar se o invariante é válido nos primeiros k estados
        states = [declare(i) for i in range(k)]
        
        # 1. Estado inicial é válido
        s.add_assertion(init(states[0]))
        
        # 2. As k-1 transições são válidas
        for i in range(k-1):
            s.add_assertion(trans(states[i], states[i+1]))
        
        # 3. Verificar se o invariante falha em ALGUM destes estados
        condition_fail = Or([Not(inv(st)) for st in states])
        
        s.push()
        s.add_assertion(condition_fail)
        if s.solve():
            print(f"[FALHA] O Caso Base falhou para k={k}.")
            print("Contra-exemplo encontrado.")
            return False
        s.pop()
        
        # === PASSO INDUTIVO ===
        # Assumir que é válido em k estados consecutivos -> provar no k+1
        states_ind = [declare(i) for i in range(k+1)]
        
        # 1. Assumir transições válidas
        for i in range(k):
            s.add_assertion(trans(states_ind[i], states_ind[i+1]))
            
        # 2. Hipótese de Indução: Invariante válido de 0 até k-1
        for i in range(k):
            s.add_assertion(inv(states_ind[i]))
            
        # 3. Tentar provar que falha no estado k
        s.add_assertion(Not(inv(states_ind[k])))
        
        if s.solve():
            print(f"[FALHA] Passo Indutivo falhou para k={k}.")
            return False
        else:
            print(f"[SUCESSO] Invariante provado por {k}-indução!")
            return True

# --- 3. Execução ---
# Tenta provar com k=1 (Indução Simples)
# Se falhar, tenta aumentar o k manualmente.
kinduction_always(declare, init, trans, inv_bezout, k=3)

# --- Alínea C: Verificação de Terminação (Look-ahead) ---

def variante(state):
    # O nosso variante é o valor de r' (rp). 
    # No algoritmo de Euclides, o resto diminui estritamente em cada passo.
    return state['rp']

def k_lookahead(declare, trans, var, k):
    print(f"--- A verificar terminação com lookahead={k} ---")
    with Solver(name="z3") as s:
        # Criar k+1 estados (estado 0 até estado k)
        states = [declare(i) for i in range(k+1)]
        
        # Aplicar k transições consecutivas
        for i in range(k):
            s.add_assertion(trans(states[i], states[i+1]))
            
            # Propriedade 1: O variante deve ser sempre não-negativo
            # (No nosso caso, rp >= 0 é garantido pela lógica de inteiros positivos)
            s.add_assertion(var(states[i]) >= Int(0))

        nao_decresceu = GE(var(states[k]), var(states[0]))
        nao_terminou  = Not(Equals(var(states[k]), Int(0)))
        
        # Queremos ver se é possível esta situação má acontecer
        s.add_assertion(And(nao_decresceu, nao_terminou))
        
        if s.solve():
            print(f"[FALHA] Existe um caminho onde o variante não decresce em {k} passos.")
            print("Contra-exemplo:")
            # Opcional: imprimir valores para debug
            m = s.get_model()
            rp_0 = m.get_py_value(states[0]['rp'])
            rp_k = m.get_py_value(states[k]['rp'])
            print(f"rp_inicial = {rp_0}, rp_final = {rp_k}")
            return False
        else:
            print(f"[SUCESSO] O programa termina! O variante decresce sempre a cada {k} passo(s).")
            return True

# Executar
# k=1 deve ser suficiente porque r' diminui em todas as iterações
k_lookahead(declare, trans, variante, k=1)