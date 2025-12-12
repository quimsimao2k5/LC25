from pysmt.shortcuts import *
from pysmt.typing import BVType

# --- Configuração ---
nbits = 4
N_LOC = 3

# Constantes de Estados
LOC_INIT  = BV(0, N_LOC) # 000
LOC_GUARD = BV(1, N_LOC) # 001 (While condition)
LOC_BODY  = BV(2, N_LOC) # 010 (Inside loop)
LOC_STOP  = BV(3, N_LOC) # 011 (Fim)
LOC_ERROR = BV(4, N_LOC) # 100 (Erro)

# Limites para detetar overflow em 16 bits (Signed)
# Min: -32768, Max: 32767
MIN_16 = -8
MAX_16 = 7

def declare(i):
    state = {}
    state['loc'] = Symbol(f"loc_{i}", BVType(N_LOC))
    
    # Inputs constantes
    state['a'] = Symbol(f"a_{i}", BVType(nbits))
    state['b'] = Symbol(f"b_{i}", BVType(nbits))
    
    # Variáveis do algoritmo
    state['r']  = Symbol(f"r_{i}", BVType(nbits))
    state['rp'] = Symbol(f"rp_{i}", BVType(nbits)) # r'
    state['s']  = Symbol(f"s_{i}", BVType(nbits))
    state['sp'] = Symbol(f"sp_{i}", BVType(nbits)) # s'
    state['t']  = Symbol(f"t_{i}", BVType(nbits))
    state['tp'] = Symbol(f"tp_{i}", BVType(nbits)) # t'
    
    return state

def init(state):
    # Inputs > 0 (Signed Greater Than 0)
    cond_inputs = And(
        BVSGT(state['a'], BV(0, nbits)),
        BVSGT(state['b'], BV(0, nbits))
    )
    
    # Inicialização (Linha 3 do código)
    valores_iniciais = And(
        Equals(state['r'], state['a']),
        Equals(state['rp'], state['b']),
        Equals(state['s'], BV(1, nbits)),
        Equals(state['sp'], BV(0, nbits)),
        Equals(state['t'], BV(0, nbits)),
        Equals(state['tp'], BV(1, nbits))
    )
    
    return And(
        Equals(state['loc'], LOC_INIT),
        cond_inputs,
        valores_iniciais
    )

def trans(curr, prox):
    
    # === Auxiliar: Manter variáveis constantes ===
    # (a e b nunca mudam)
    keep_consts = And(
        Equals(prox['a'], curr['a']),
        Equals(prox['b'], curr['b'])
    )

    # === Auxiliar: Manter variáveis de dados (se não houver update) ===
    keep_data = And(
        Equals(prox['r'], curr['r']),
        Equals(prox['rp'], curr['rp']),
        Equals(prox['s'], curr['s']),
        Equals(prox['sp'], curr['sp']),
        Equals(prox['t'], curr['t']),
        Equals(prox['tp'], curr['tp'])
    )

    # ---------------------------------------------------
    # 1. INIT -> GUARD
    # Passa do setup inicial para a verificação do while
    # ---------------------------------------------------
    t_init_guard = And(
        Equals(curr['loc'], LOC_INIT),
        Equals(prox['loc'], LOC_GUARD),
        keep_data
    )

    # ---------------------------------------------------
    # 2. GUARD -> BODY ou STOP
    # Avalia: while r' != 0
    # ---------------------------------------------------
    cond_while = Not(Equals(curr['rp'], BV(0, nbits)))

    # Caso r' != 0: Entra no loop (BODY)
    t_guard_body = And(
        Equals(curr['loc'], LOC_GUARD),
        cond_while,
        Equals(prox['loc'], LOC_BODY),
        keep_data
    )

    # Caso r' == 0: Termina (STOP)
    t_guard_stop = And(
        Equals(curr['loc'], LOC_GUARD),
        Not(cond_while),
        Equals(prox['loc'], LOC_STOP),
        keep_data
    )

    # ---------------------------------------------------
    # 3. BODY -> GUARD (Cálculos com Sucesso)
    # 4. BODY -> ERROR (Se houver Overflow ou r=0)
    # ---------------------------------------------------
    
    # Cálculos em 16 bits (para o próximo estado se tudo correr bem)
    # q = r div r' (Divisão Signed)
    q = BVSDiv(curr['r'], curr['rp'])
    
    # r, r' = r', r - q*r' (equivalente a r % r')
    # Nota: r % r' é seguro em termos de magnitude, mas r=0 é erro segundo enunciado.
    next_r = curr['rp']
    next_rp = BVSRem(curr['r'], curr['rp']) # Resto da divisão
    
    # Cálculos manuais de OVERFLOW (usando extensão para 32 bits)
    # Precisamos calcular: s - q * s'  e  t - q * t'
    
    # Estender para 32 bits
    s_32  = BVSExt(curr['s'], nbits)
    sp_32 = BVSExt(curr['sp'], nbits)
    t_32  = BVSExt(curr['t'], nbits)
    tp_32 = BVSExt(curr['tp'], nbits)
    q_32  = BVSExt(q, nbits)
    
    # Calcular em 32 bits: new_val = val - (q * val_prime)
    term_s_32 = BVMul(q_32, sp_32)
    new_s_32  = BVSub(s_32, term_s_32)
    
    term_t_32 = BVMul(q_32, tp_32)
    new_t_32  = BVSub(t_32, term_t_32)
    
    # Verificar se cabe em 16 bits (Signed: -32768 a 32767)
    # Usamos BV(valor, 32) para comparar
    limit_min = BV(MIN_16, 32)
    limit_max = BV(MAX_16, 32)
    
    # Condições de Overflow
    ovf_s = Or(BVSLT(new_s_32, limit_min), BVSGT(new_s_32, limit_max))
    ovf_t = Or(BVSLT(new_t_32, limit_min), BVSGT(new_t_32, limit_max))
    
    # Condição r=0 (Enunciado: "Considere estado de erro quando r = 0")
    # Atenção: r aqui refere-se à variável r do estado, não ao r'.
    # Se r se tornar 0, é erro.
    r_is_zero = Equals(curr['r'], BV(0, nbits))
    
    # Agrupar condições de Erro
    # Se houver overflow OU r for 0
    is_error_condition = Or(ovf_s, ovf_t, r_is_zero)
    
    # --- Transição BODY -> ERROR ---
    t_body_error = And(
        Equals(curr['loc'], LOC_BODY),
        is_error_condition,
        Equals(prox['loc'], LOC_ERROR),
        keep_data # Não importa os valores no erro
    )

    # --- Transição BODY -> GUARD (Sucesso) ---
    # Só acontece se NÃO houver erro
    # Retiramos os 16 bits inferiores dos resultados de 32 bits para atualizar
    next_s = BVExtract(new_s_32, 15, 0)
    next_sp = curr['sp'] # s' no próximo estado é o s' antigo?
    # ESPERA! Olhando para o código Python da imagem:
    # r, r', s, s', t, t' = r', r - q*r', s', s - q*s', t', t - q*t'
    # Isto é atribuição múltipla simultânea!
    # O novo r é o antigo r' -> OK
    # O novo r' é r % r' -> OK
    # O novo s é o antigo s' -> CORREÇÃO AQUI
    # O novo s' é s - q*s' -> CORREÇÃO AQUI
    
    t_body_guard = And(
        Equals(curr['loc'], LOC_BODY),
        Not(is_error_condition),
        Equals(prox['loc'], LOC_GUARD),
        
        # Atualizações simultâneas (Cuidado com a ordem do enunciado)
        # Código: r, r', s, s', t, t' = r', r - ..., s', s - ..., t', t - ...
        
        Equals(prox['r'], curr['rp']),      # r_new = r'
        Equals(prox['rp'], next_rp),        # r'_new = resto
        
        Equals(prox['s'], curr['sp']),      # s_new = s'
        Equals(prox['sp'], next_s),         # s'_new = s - q*s' (calculado acima)
        
        Equals(prox['t'], curr['tp']),      # t_new = t'
        Equals(prox['tp'], BVExtract(new_t_32, 15, 0)) # t'_new = t - q*t'
    )
    
    # ---------------------------------------------------
    # 5. STOP -> STOP (Loop final)
    # 6. ERROR -> ERROR (Loop final)
    # ---------------------------------------------------
    t_stop_loop = And(
        Equals(curr['loc'], LOC_STOP),
        Equals(prox['loc'], LOC_STOP),
        keep_data
    )
    
    t_error_loop = And(
        Equals(curr['loc'], LOC_ERROR),
        Equals(prox['loc'], LOC_ERROR),
        keep_data
    )

    # Juntar tudo
    return And(
        keep_consts,
        Or(
            t_init_guard,
            t_guard_body,
            t_guard_stop,
            t_body_guard,
            t_body_error,
            t_stop_loop,
            t_error_loop
        )
    )

from z3 import *

def check_chc():
    print("--- A configurar Constraint Horn Clauses (CHC) com BitVecs ---")
    
    fp = Fixedpoint()
    
    # Configuração otimizada para BitVectors
    params = {
        "engine": "spacer",
        "print_statistics": True,
        "xform.slice": False,
        "xform.inline_linear": False,
        "xform.inline_eager": False,
        "spacer.p3.share_invariants": True 
    }
    fp.set(**params)

    # Começa com 4 bits para testar a lógica. Se funcionar, muda para 16.
    nbits = 4 
    
    # Limites Signed: -8 a 7 (para 4 bits)
    limit_val_min = -(2**(nbits-1))
    limit_val_max = (2**(nbits-1)) - 1
    
    print(f"A verificar para nbits={nbits} (Intervalo Signed: [{limit_val_min}, {limit_val_max}])")

    # Variáveis
    loc, a, b = BitVecs('loc a b', nbits)
    r, rp, s, sp, t, tp = BitVecs('r rp s sp t tp', nbits)
    
    loc_p, a_p, b_p = BitVecs('loc_p a_p b_p', nbits)
    r_p, rp_p, s_p, sp_p, t_p, tp_p = BitVecs('r_p rp_p s_p sp_p t_p tp_p', nbits)

    vars_state = [loc, a, b, r, rp, s, sp, t, tp]
    vars_state_p = [loc_p, a_p, b_p, r_p, rp_p, s_p, sp_p, t_p, tp_p]

    # Relação Invariante
    sorts = [BitVecSort(nbits)] * 9 + [BoolSort()]
    Inv = Function('Inv', *sorts)
    fp.register_relation(Inv)

    # Constantes
    LOC_INIT, LOC_GUARD, LOC_BODY, LOC_STOP, LOC_ERROR = [BitVecVal(i, nbits) for i in range(5)]

    # =========================================================
    # REGRA 1: INIT -> Inv
    # =========================================================
    init_cond = And(
        loc == LOC_INIT,
        a > 0, b > 0,
        r == a, rp == b,
        s == 1, sp == 0,
        t == 0, tp == 1
    )
    fp.rule(ForAll(vars_state, Implies(init_cond, Inv(*vars_state))))

    # =========================================================
    # REGRA 2: Inv AND Trans -> Inv'
    # =========================================================
    keep_consts = And(a_p == a, b_p == b)
    
    # Transições de controle simples
    t_simple = Or(
        And(loc == LOC_INIT, loc_p == LOC_GUARD, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp),
        And(loc == LOC_GUARD, rp == 0, loc_p == LOC_STOP, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp),
        And(loc == LOC_GUARD, rp != 0, loc_p == LOC_BODY, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp)
    )

    # --- Cálculos do Loop (BODY -> GUARD) ---
    # Extensão para detetar overflow (nbits * 2)
    ext_bits = nbits * 2
    s_ext, sp_ext = SignExt(nbits, s), SignExt(nbits, sp)
    t_ext, tp_ext = SignExt(nbits, t), SignExt(nbits, tp)
    r_ext, rp_ext = SignExt(nbits, r), SignExt(nbits, rp)
    
    # q = r / r' (Signed Division)
    # Proteção: se rp=0, q=0. O solver precisa disto para não falhar em caminhos inválidos.
    q_ext = If(rp_ext == 0, BitVecVal(0, ext_bits), r_ext / rp_ext)
    
    # new_s = s - q * s'
    new_s_ext = s_ext - (q_ext * sp_ext)
    # new_t = t - q * t'
    new_t_ext = t_ext - (q_ext * tp_ext)
    
    # Overflow Check
    l_min = BitVecVal(limit_val_min, ext_bits)
    l_max = BitVecVal(limit_val_max, ext_bits)
    
    ovf_s = Or(new_s_ext < l_min, new_s_ext > l_max)
    ovf_t = Or(new_t_ext < l_min, new_t_ext > l_max)
    r_is_zero = (r == 0)
    
    is_error = Or(ovf_s, ovf_t, r_is_zero)
    
    t_body_guard = And(
        loc == LOC_BODY,
        Not(is_error),
        loc_p == LOC_GUARD,
        r_p == rp,
        rp_p == SRem(r, rp), # Signed Remainder
        s_p == sp,
        sp_p == Extract(nbits-1, 0, new_s_ext),
        t_p == tp,
        tp_p == Extract(nbits-1, 0, new_t_ext)
    )

    trans_valida = And(keep_consts, Or(t_simple, t_body_guard))
    
    vars_all = vars_state + vars_state_p
    fp.rule(ForAll(vars_all, Implies(And(Inv(*vars_state), trans_valida), Inv(*vars_state_p))))

    # =========================================================
    # REGRA 3: Inv AND Error -> Fail
    # =========================================================
    condicao_erro = And(loc == LOC_BODY, is_error)
    
    Fail = Function('Fail', BoolSort())
    fp.register_relation(Fail)
    
    fp.rule(ForAll(vars_state, Implies(And(Inv(*vars_state), condicao_erro), Fail())))

    print("A verificar acessibilidade do erro...")
    res = fp.query(Fail())
    
    if res == unsat:
        print("RESULTADO: Seguro (UNSAT)")
        print("Invariante encontrado! O erro é inalcançável.")
    elif res == sat:
        print("RESULTADO: Inseguro (SAT)")
        print("Erro encontrado! O invariante não existe.")
        print("Caminho de erro:", fp.get_ground_sat_answer())
    else:
        print("RESULTADO: Desconhecido (Unknown)")
        print("Motivo: Timeout ou complexidade excessiva.")

#check_chc()

from z3 import *

def check_reachability_with_trace():
    print("\n--- Alínea C: Verificação de Acessibilidade do Erro (Trace) ---")
    
    fp = Fixedpoint()
    # Configurações para facilitar a descoberta de contra-exemplos
    fp.set(engine='spacer')
    fp.set('xform.slice', False)
    fp.set('xform.inline_linear', False)

    # 1. Definições (Usa nbits=4 ou 5 para garantir que encontras o erro rápido)
    nbits = 4 
    limit_val_min = -(2**(nbits-1))
    limit_val_max = (2**(nbits-1)) - 1
    
    print(f"Configuração: nbits={nbits}")

    # Variáveis
    loc, a, b = BitVecs('loc a b', nbits)
    r, rp, s, sp, t, tp = BitVecs('r rp s sp t tp', nbits)
    
    loc_p, a_p, b_p = BitVecs('loc_p a_p b_p', nbits)
    r_p, rp_p, s_p, sp_p, t_p, tp_p = BitVecs('r_p rp_p s_p sp_p t_p tp_p', nbits)

    # Função de Estado (Invariante potencial)
    sorts = [BitVecSort(nbits)] * 9 + [BoolSort()]
    Inv = Function('Inv', *sorts)
    fp.register_relation(Inv)

    # Constantes
    LOC_INIT, LOC_GUARD, LOC_BODY, LOC_STOP, LOC_ERROR = [BitVecVal(i, nbits) for i in range(5)]

    # -------------------------------------------------------
    # REGRA 1: INIT -> Inv
    # -------------------------------------------------------
    init_cond = And(
        loc == LOC_INIT,
        a > 0, b > 0,
        r == a, rp == b,
        s == 1, sp == 0,
        t == 0, tp == 1
    )
    vars_state = [loc, a, b, r, rp, s, sp, t, tp]
    fp.rule(ForAll(vars_state, Implies(init_cond, Inv(*vars_state))))

    # -------------------------------------------------------
    # REGRA 2: Inv & Trans -> Inv
    # -------------------------------------------------------
    keep_consts = And(a_p == a, b_p == b)
    
    # Transições básicas
    t_simple = Or(
        And(loc == LOC_INIT, loc_p == LOC_GUARD, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp),
        And(loc == LOC_GUARD, rp == 0, loc_p == LOC_STOP, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp),
        And(loc == LOC_GUARD, rp != 0, loc_p == LOC_BODY, r_p==r, rp_p==rp, s_p==s, sp_p==sp, t_p==t, tp_p==tp)
    )

    # Transição de Cálculo (BODY -> GUARD)
    ext_bits = nbits * 2
    s_ext, sp_ext = SignExt(nbits, s), SignExt(nbits, sp)
    t_ext, tp_ext = SignExt(nbits, t), SignExt(nbits, tp)
    r_ext, rp_ext = SignExt(nbits, r), SignExt(nbits, rp)
    
    q_ext = If(rp_ext == 0, BitVecVal(0, ext_bits), r_ext / rp_ext)
    
    new_s_ext = s_ext - (q_ext * sp_ext)
    new_t_ext = t_ext - (q_ext * tp_ext)
    
    l_min = BitVecVal(limit_val_min, ext_bits)
    l_max = BitVecVal(limit_val_max, ext_bits)
    
    ovf_s = Or(new_s_ext < l_min, new_s_ext > l_max)
    ovf_t = Or(new_t_ext < l_min, new_t_ext > l_max)
    r_is_zero = (r == 0)
    
    is_error = Or(ovf_s, ovf_t, r_is_zero)
    
    t_body_guard = And(
        loc == LOC_BODY,
        Not(is_error),
        loc_p == LOC_GUARD,
        r_p == rp, rp_p == SRem(r, rp),
        s_p == sp, sp_p == Extract(nbits-1, 0, new_s_ext),
        t_p == tp, tp_p == Extract(nbits-1, 0, new_t_ext)
    )

    trans_valida = And(keep_consts, Or(t_simple, t_body_guard))
    
    vars_state_p = [loc_p, a_p, b_p, r_p, rp_p, s_p, sp_p, t_p, tp_p]
    vars_all = vars_state + vars_state_p
    fp.rule(ForAll(vars_all, Implies(And(Inv(*vars_state), trans_valida), Inv(*vars_state_p))))

    # -------------------------------------------------------
    # REGRA 3: Inv & Erro -> Fail
    # -------------------------------------------------------
    condicao_erro_body = And(loc == LOC_BODY, is_error)
    
    Fail = Function('Fail', BoolSort())
    fp.register_relation(Fail)
    
    fp.rule(ForAll(vars_state, Implies(And(Inv(*vars_state), condicao_erro_body), Fail())))

    # -------------------------------------------------------
    # QUERY: Tentar atingir o erro
    # -------------------------------------------------------
    print("A verificar se o erro é atingível...")
    # Timeout de 100 segundos
    fp.set(timeout=100000)
    
    res = fp.query(Fail())

    if res == sat:
        print("\n[RESULTADO] O estado de erro é ATINGÍVEL! (SAT)")
        print("Isto confirma que o programa não é seguro.")
        print("\n--- Caminho para o Erro (Interpolantes/Trace) ---")
        # Isto imprime o "traço" lógico que prova o erro
        print(fp.get_ground_sat_answer())
    elif res == unsat:
        print("\n[RESULTADO] O estado de erro é INACESSÍVEL. (UNSAT)")
    else:
        print("\n[RESULTADO] Unknown (Timeout ou complexidade excessiva)")

check_reachability_with_trace()