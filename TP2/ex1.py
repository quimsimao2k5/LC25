import numpy as np

def termo_linear(a, x):
    """ Calcula <a.x> = (a_1 AND x_1) XOR (a_2 AND x_2) ... XOR (a_n AND x_n) """
    # (a & x) faz o AND elemento-a-elemento
    # .sum() conta quantos 'True' resultaram
    # % 2 faz o XOR de tudo (se a soma for ímpar -> 1, se for par -> 0)
    return (a & x).sum() % 2

def f_sem_falhas(p, x):
    """ Calcula f(p;x) = o ^ <a.x> ^ (<b.x> AND <c.x>) """
    o, a, b, c = p
    
    termo_a = termo_linear(a, x)
    termo_b = termo_linear(b, x)
    termo_c = termo_linear(c, x)
    
    # O operador ^ é o XOR em Python
    # O operador & é o AND em Python (para 0s e 1s)
    resultado = o ^ termo_a ^ (termo_b & termo_c)
    return resultado

def gerar_parametros_circuito(z_bits, s_master, n,k=32):
    """
    Gera os parâmetros (o_i, a_i, b_i, c_i) para os n sub-circuitos.
    Assume que z_bits é um array numpy booleano de comprimento n.
    """
    
    # 1. Inicializar o PRNG "Master" com a 's_master' [cite: 219-220]
    rng_master = np.random.default_rng(seed=s_master)

    # 2. Gerar as n 'sub-seeds' [cite: 221]
    sub_seeds = rng_master.integers(low=0, high=2**k, size=n)

    parametros_todos = []
    print(f"A gerar {n} sub-circuitos...")

    for i in range(n):
        # 3. Gerar a_i, b_i, c_i com a 'sub_seed' s_i
        rng_sub = np.random.default_rng(seed=sub_seeds[i])
        a_i = rng_sub.integers(0, 2, size=n, dtype=bool)
        b_i = rng_sub.integers(0, 2, size=n, dtype=bool)
        c_i = rng_sub.integers(0, 2, size=n, dtype=bool)

        # 4. Calcular o offset o_i [cite: 226]
        p0 = (0, a_i, b_i, c_i)
        o_i = f_sem_falhas(p0, z_bits) # Usa z_bits diretamente
        
        # 5. Guardar os parâmetros finais
        p_final = (o_i, a_i, b_i, c_i)
        parametros_todos.append(p_final)

    print("Parâmetros do circuito gerados com sucesso.")
    return parametros_todos

from pysmt.shortcuts import (
    Solver, Symbol, BOOL, INT, 
    And, Or, Not, Ite, Iff, Xor, Plus, Int, Bool, FALSE, GT
)
from pysmt.typing import BOOL

def construir_modelo_smt(parametros_todos, n):
    """
    Constrói o modelo SMT para o circuito n x n com falhas.
    """
    
    # --- 1. Definir Variáveis SMT Principais ---
    
    # x = Vetor de input de n bits
    # (Em pySMT é mais fácil usar n Bools ou um BV(n))
    # Vamos usar n Bools para facilitar os XORs
    x = [Symbol(f"x_{i}", BOOL) for i in range(n)]
    
    # y = Vetor de output de n bits
    y = [Symbol(f"y_{i}", BOOL) for i in range(n)]
    
    # --- 2. Definir Variáveis SMT de Falha ---
    # 3 falhas por 'and', n sub-circuitos
    # falhas[i][j] = falha do sub-circuito i, porta 'and' j (j=0,1,2)
    falhas = [
        [Symbol(f"falha_{i}_{j}", BOOL) for j in range(3)] 
        for i in range(n)
    ]
    
    lista_de_formulas = [] # Guarda a fórmula para cada y_i

    # --- 3. Construir a Lógica para cada Sub-circuito ---
    for i in range(n):
        o_i, a_i, b_i, c_i = parametros_todos[i]
        
        # Converte os parâmetros numpy (True/False) para SMT (TRUE(), FALSE())
        o_i_smt = Bool(bool(o_i))

        # --- Lógica do Sub-circuito i ---
        
        # a. Termos Lineares (Grandes XORs)
        def multi_xor(terms):
            if not terms:
                return FALSE()
            expr = terms[0]
            for t in terms[1:]:
                expr = Xor(expr, t)
            return expr
        # <a.x> = Xor( ... x_k ... ) para todo k onde a_i_smt[k] é TRUE
        termo_a = multi_xor([x[k] for k in range(n) if a_i[k]])
        termo_b = multi_xor([x[k] for k in range(n) if b_i[k]])
        termo_c = multi_xor([x[k] for k in range(n) if c_i[k]])

        # b. As 3 Portas 'and' (com redundância) [cite: 25, 64]
        and_ideal = And(termo_b, termo_c) # O resultado que devia dar

        # c. Aplicar as falhas
        # Se falha=True, inverte o resultado
        out_and = []
        for j in range(3):
            falha_ij = falhas[i][j]
            # Ite(cond, se_verdadeiro, se_falso)
            out_and_j = Ite(falha_ij, Not(and_ideal), and_ideal)
            out_and.append(out_and_j)
            
        # d. Porta de Maioria (maj_1) [cite: 25]
        # Maj(a,b,c) = (a&b) | (a&c) | (b&c)
        maj_out = Or(
            And(out_and[0], out_and[1]),
            And(out_and[0], out_and[2]),
            And(out_and[1], out_and[2])
        )
        
        # e. Fórmula final para y_i
        # y_i = o_i ^ <a.x> ^ maj_out [cite: 12, 25]
        formula_yi = Iff(y[i], multi_xor([o_i_smt, termo_a, maj_out]))
        lista_de_formulas.append(formula_yi)

    # A fórmula total é o AND de todas as fórmulas de sub-circuito
    formula_final = And(lista_de_formulas)
    
    print("Modelo SMT construído com sucesso.")
    
    # Devolvemos as "pontas" do modelo
    return x, y, falhas, formula_final

def encontrar_segredo_falso(n, x, y, falhas, formula_final, z_bits_reais):
    """
    Tenta encontrar um z' != z e um conjunto de falhas > 0
    tal que o output do circuito y seja 0^n.
    """
    
    with Solver(name="z3") as s:
        
        # --- Adicionar Restrições ---
        
        # Restrição D: A lógica do circuito tem de ser obedecida
        s.add_assertion(formula_final)
        
        # Restrição A: O output 'y' tem de ser 0^n (tudo False)
        target_output = And([Not(y[i]) for i in range(n)])
        s.add_assertion(target_output)
        
        # Restrição B: O input 'x' NÃO pode ser o segredo real 'z'
        z_real_smt = [Bool(bool(b)) for b in z_bits_reais]
        x_equals_z_real = And([Iff(x[i], z_real_smt[i]) for i in range(n)])
        s.add_assertion(Not(x_equals_z_real))
        
        # Restrição C: Pelo menos UMA falha tem de estar ativa
        todas_as_falhas = [falha_bit for sublist in falhas for falha_bit in sublist]
        pelo_menos_uma_falha = Or(todas_as_falhas)
        s.add_assertion(pelo_menos_uma_falha)

        # --- Verificar o Modelo ---
        
        print("\nA procurar um 'segredo falso' (z') com falhas...")
        if s.solve():
            print("--- SUCESSO! Encontrado z' e falhas ---")
            model = s.get_model()
            
            # 1. Obter o z' (que é o valor do input 'x')
            z_prime_vals = [model.get_py_value(x[i]) for i in range(n)]
            print(f"Segredo Falso (z'): {z_prime_vals}")
            
            # 2. Obter as falhas ativas
            falhas_ativas = []
            for i in range(n):
                for j in range(3):
                    if model.get_py_value(falhas[i][j]) == True:
                        falhas_ativas.append(f"falha_{i}_{j}")
            print(f"Falhas Ativas: {falhas_ativas}")
            
        else:
            print("--- FALHA (UNSAT) ---")
            print("Não foi possível encontrar um z' que satisfaça as condições.")


def maximizar_falhas(n, x, y, falhas, formula_final, z_bits_reais):
    """
    Tarefa 3: Maximiza o número de falhas que o circuito aguenta
    enquanto o output (com input z) continua a ser 0^n.
    Implementação por busca iterativa usando Solver (pySMT).
    """
    print("\n--- Tarefa 3: A maximizar falhas (busca iterativa) ---")

    all_faults = [f for sub in falhas for f in sub]
    falhas_int_terms = [Ite(f, Int(1), Int(0)) for f in all_faults]
    contagem_expr = Plus(falhas_int_terms) if falhas_int_terms else Int(0)

    # Construir as restrições fixas (base)
    z_real_smt = [Bool(bool(b)) for b in z_bits_reais]
    restricoes_base = [
        formula_final,
        And([Iff(x[i], z_real_smt[i]) for i in range(n)]),  # x = z real
        And([Not(y[i]) for i in range(n)])                 # y = 0^n
    ]

    with Solver(name="z3") as s:
        for r in restricoes_base:
            s.add_assertion(r)

        best_model = None
        best_count = -1

        # Iterar enquanto houver modelo que melhore o número de falhas
        while s.solve():
            m = s.get_model()
            # contar falhas no modelo actual
            cur_count = sum(1 for f in all_faults if m.get_py_value(f))
            best_count = cur_count
            best_model = m

            # exigir melhoria estrita: contagem > cur_count
            s.add_assertion(GT(contagem_expr, Int(cur_count)))

        if best_model is not None:
            print("--- SUCESSO! Solução de maximização encontrada ---")
            print(f"Número MÁXIMO de falhas que o circuito aguenta: {best_count}")
            falhas_ativas = []
            for idx, f in enumerate(all_faults):
                if best_model.get_py_value(f):
                    i = idx // 3
                    j = idx % 3
                    falhas_ativas.append(f"falha_{i}_{j}")
            print(f"Configuração de falhas que o circuito 'sobreviveu': {falhas_ativas}")
        else:
            print("--- FALHA (UNSAT) ---")
            print("Não foi possível encontrar uma solução (nem mesmo 0 falhas).")

# ====================================================================
# SCRIPT PRINCIPAL (Exemplo de Execução)
# ====================================================================

# 1. Definir parâmetros
N = 8      # Dimensão (n). Usar valores pequenos (4-8) para testar.
S_MASTER = 12345 # Master seed (inteiro)

# 2. Definir o Segredo Real 'z' (tem de ter 'N' bits)
# Vamos criar um z_bits = [True, False, False, True] (para N=4)
z_bits_reais = np.array([True, False, False, True, True, False, True, False])

# 3. Executar Tarefa 1.A
parametros = gerar_parametros_circuito(z_bits_reais, S_MASTER, N)

# 4. Executar Tarefa 1.B
x_smt, y_smt, falhas_smt, formula_smt = construir_modelo_smt(parametros, N)

# 5. Executar Tarefa 2
encontrar_segredo_falso(N, x_smt, y_smt, falhas_smt, formula_smt, z_bits_reais)

maximizar_falhas(N, x_smt, y_smt, falhas_smt, formula_smt, z_bits_reais)