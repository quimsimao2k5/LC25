from z3 import *

def exemplo_chc_que_funciona():
    print("--- Exemplo CHC: Multiplicação por Somas ---")
    
    fp = Fixedpoint()
    fp.set(engine='spacer') # O mesmo motor do teu trabalho

    # Configuração simples: 5 bits (Max signed: 15, Min: -16)
    nbits = 16
    
    # Variáveis: a (constante), b (contador), res (resultado acumulado)
    a, b, res = BitVecs('a b res', nbits)
    a_p, b_p, res_p = BitVecs('a_p b_p res_p', nbits)

    # Função Invariante que o Z3 tem de descobrir
    Inv = Function('Inv', BitVecSort(nbits), BitVecSort(nbits), BitVecSort(nbits), BoolSort())
    fp.register_relation(Inv)

    # Relação de Falha (Erro)
    Fail = Function('Fail', BoolSort())
    fp.register_relation(Fail)

    # ------------------------------------------------
    # REGRA 1: Init
    # res = 0, b > 0, a > 0
    # ------------------------------------------------
    init_cond = And(res == 0, b > 0, a > 0)
    fp.rule(ForAll([a, b, res], Implies(init_cond, Inv(a, b, res))))

    # ------------------------------------------------
    # REGRA 2: Transição (Loop)
    # while (b > 0): res = res + a; b = b - 1;
    # ------------------------------------------------
    # Cálculos estendidos para detetar overflow manualmente
    res_ext = SignExt(nbits, res)
    a_ext   = SignExt(nbits, a)
    new_res_ext = res_ext + a_ext # Soma em vez de divisão!
    
    # Limites para 5 bits
    limit_min = -(2**(nbits-1))
    limit_max = (2**(nbits-1)) - 1
    
    # Deteção de Overflow
    has_overflow = Or(new_res_ext < limit_min, new_res_ext > limit_max)
    
    # Passo válido (Sem overflow)
    trans_safe = And(
        b > 0,                # Condição do while
        Not(has_overflow),    # Não rebentou
        a_p == a,             # a não muda
        b_p == b - 1,         # decrementa contador
        res_p == Extract(nbits-1, 0, new_res_ext) # atualiza resultado
    )
    
    fp.rule(ForAll([a, b, res, a_p, b_p, res_p], Implies(And(Inv(a, b, res), trans_safe), Inv(a_p, b_p, res_p))))

    # ------------------------------------------------
    # REGRA 3: Erro (Fail)
    # Se houver overflow, falhámos.
    # ------------------------------------------------
    # O erro acontece se estivermos no loop (b > 0) e o overflow disparar
    condicao_erro = And(b > 0, has_overflow)
    
    fp.rule(ForAll([a, b, res], Implies(And(Inv(a, b, res), condicao_erro), Fail())))

    # ------------------------------------------------
    # QUERY
    # ------------------------------------------------
    print(f"A verificar para nbits={nbits}...")
    print("Se a*b > MAX, deve dar SAT (Erro)...\n")
    
    res = fp.query(Fail())
    
    if res == sat:
        print(">>> RESULTADO: SAT (Inseguro!)")
        print("O solver encontrou valores que causam overflow.")
        
        # --- HACK: Extrair valores do texto da prova ---
        ans = str(fp.get_answer())
        import re
        
        # Procura por padrões Inv(num, num, num)
        # O Z3 imprime os argumentos na ordem definida na função: Inv(a, b, res)
        matches = re.findall(r'Inv\((-?\d+),\s*(-?\d+),\s*(-?\d+)\)', ans)
        
        if matches:
            print("\n--- Valores Encontrados (Extraídos do Log) ---")
            # Geralmente o Z3 lista do fim para o início ou vice-versa, mas procuramos o estado inicial onde res=0
            
            found_init = False
            for val_a, val_b, val_res in matches:
                if int(val_res) == 0:
                    print(f" -> CONTRA-EXEMPLO INICIAL: a = {val_a}, b = {val_b}")
                    found_init = True
            
            if not found_init:
                # Se não acharmos res=0, mostramos o primeiro que aparecer
                val_a, val_b, val_res = matches[0]
                print(f" -> Um estado no traço de erro: a = {val_a}, b = {val_b}, res = {val_res}")
                
        else:
            print("\n(Não foi possível extrair os valores automaticamente. Tente ler o output acima)")
            print(ans)
            
    elif res == unsat:
        print(">>> RESULTADO: UNSAT (Seguro)")
    else:
        print("Unknown")

exemplo_chc_que_funciona()