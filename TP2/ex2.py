from pysmt.shortcuts import*# (BV,Symbol,Solver,And,Not,Or,Equals,BVULE,BVULT,BVAdd,BVSub,BVAnd,BVLShl,BVLShr,BVMul)
from pysmt.typing import BVType

nbits = 8

N_LOC=3

#constantes
LOC_INIT  = BV(0, N_LOC)
LOC_SKIP  = BV(1, N_LOC)
LOC_LEFT  = BV(2, N_LOC)
LOC_RIGHT = BV(3, N_LOC)
LOC_STOP  = BV(4, N_LOC)
LOC_ERROR = BV(5, N_LOC)

def declare(i):
    state = {}
    # 'loc' é uma VARIÁVEL BitVec de tamanho N_LOC
    state['loc'] = Symbol('loc'+str(i), BVType(N_LOC))
    
    # Variáveis do problema (todas BitVec de tamanho nbits) 
    state['x'] = Symbol('x'+str(i), BVType(nbits))
    state['y'] = Symbol('y'+str(i), BVType(nbits))
    state['z'] = Symbol('z'+str(i), BVType(nbits))
    state['a'] = Symbol('a'+str(i), BVType(nbits))
    state['b'] = Symbol('b'+str(i), BVType(nbits))
    return state

def init(state):
    # está no estado init, x == a, y == b, z = 0
    return And(
        Equals(
            state['loc'],
            LOC_INIT
        ),
        BVUGT(
            state['a'],
            BV(0,nbits)
        ),
        BVUGT(
            state['b'],
            BV(0,nbits)
        ),
        Equals(
            state['x'],
            state['a']
        ),
        Equals(
            state['y'],
            state['b']
        ),
        Equals(
            state['z'],
            BV(0,nbits)
        )
    )

def trans(curr, prox):
    
    cond_overflow_left = BVULT(BVAdd(curr['x'], curr['x']), curr['x'])

    cond_overflow_right = BVULT(BVAdd(curr['z'], curr['x']), curr['z'])

    # --- Transições ---
    
    trans_init_skip = And(
        Equals(curr['loc'], LOC_INIT),
        Equals(prox['loc'], LOC_SKIP),
        Equals(curr['x'], prox['x']),
        Equals(curr['y'], prox['y']),
        Equals(curr['z'], prox['z'])
    )

    trans_skip_stop = And(
        Equals(curr['loc'], LOC_SKIP),
        Equals(prox['loc'], LOC_STOP),
        Equals(curr['x'], prox['x']),
        Equals(curr['y'], prox['y']),
        Equals(curr['z'], prox['z']),
        Equals(curr['y'], BV(0, nbits))
    )

    trans_skip_right = And(
        Equals(curr['loc'], LOC_SKIP),
        Equals(prox['loc'], LOC_RIGHT),
        Equals(prox['x'], curr['x']),
        Equals(prox['y'], curr['y']),
        Equals(curr['z'], prox['z']),
        Not(Equals(curr['y'], BV(0, nbits))),
        Equals(BVAnd(curr['y'], BV(1, nbits)), BV(1, nbits))
    )
 
    trans_skip_left = And(
        Equals(curr['loc'], LOC_SKIP),
        Equals(prox['loc'], LOC_LEFT),
        Equals(prox['x'], curr['x']),
        Equals(prox['y'], curr['y']),
        Equals(curr['z'], prox['z']),
        Not(Equals(curr['y'], BV(0, nbits))),
        Equals(BVAnd(curr['y'], BV(1, nbits)), BV(0, nbits))
    )

    # CORRIGIDO: Adicionado Not(cond_overflow_right)
    trans_right_skip = And(
        Equals(curr['loc'], LOC_RIGHT),
        Equals(prox['loc'], LOC_SKIP),
        Not(cond_overflow_right),  # <--- CORREÇÃO
        Equals(curr['x'], prox['x']),
        Equals(BVSub(curr['y'], BV(1, nbits)), prox['y']),
        Equals(BVAdd(curr['z'], curr['x']), prox['z'])
    )

    # CORRIGIDO: Adicionado Not(cond_overflow_left)
    trans_left_skip = And(
        Equals(curr['loc'], LOC_LEFT),
        Equals(prox['loc'], LOC_SKIP),
        Not(cond_overflow_left),
        Equals(prox['x'], BVLShl(curr['x'], BV(1, nbits))),
        Equals(prox['y'], BVLShr(curr['y'], BV(1, nbits))),
        Equals(curr['z'], prox['z'])
    )

    # Esta transição está boa
    trans_left_error = And(
        Equals(curr['loc'], LOC_LEFT),
        Equals(prox['loc'], LOC_ERROR),
        cond_overflow_left,
        Equals(prox['x'], BVLShl(curr['x'], BV(1, nbits))),
        Equals(prox['y'], BVLShr(curr['y'], BV(1, nbits))),
        Equals(prox['z'], curr['z'])
    )

    # Esta transição está boa
    trans_right_error = And(
        Equals(curr['loc'], LOC_RIGHT),
        Equals(prox['loc'], LOC_ERROR),
        cond_overflow_right,
        Equals(prox['x'], curr['x']),
        Equals(prox['y'], BVSub(curr['y'], BV(1, nbits))),
        Equals(prox['z'], BVAdd(curr['z'], curr['x']))
    )

    # Constantes
    const_ab = And(
        Equals(curr['a'], prox['a']),
        Equals(curr['b'], prox['b'])
    )

    return And(
        const_ab,
        Or(
            trans_init_skip,
            trans_skip_stop,
            trans_skip_left,
            trans_skip_right,
            trans_right_skip,
            trans_left_skip,
            trans_left_error,
            trans_right_error
        )
    )

def invariante(state):
    # Propriedade: x*y + z == a*b
    lado_esquerdo = BVAdd(
        BVMul(state['x'], state['y']),
        state['z']
    )
    lado_direito = BVMul(
        state['a'],
        state['b']
    )
    return Equals(lado_esquerdo, lado_direito)

def bmc_para_invariante(declare, init, trans, inv, K):
    estados = {
        0:'INIT',
        1:'SKIP',
        2:'LEFT',
        3:'RIGHT',
        4:'STOP',
        5:'ERROR'
    }
    for k in range(1, K + 1):
        with Solver(name="z3") as s:
            
            states = [declare(i) for i in range(k)]

            s.add_assertion(init(states[0]))

            for i in range(k - 1):
                s.add_assertion(trans(states[i], states[i+1]))

            s.add_assertion(Or([Not(inv(states[i])) for i in range(k)]))

            # 5. Verificar
            if s.solve():
                print(f"--- CONTRA-EXEMPLO ENCONTRADO no passo {k} ---")
                model = s.get_model()
                for i in range(k):
                    print(f"Estado {i}:")
                    loc_val = model.get_py_value(states[i]['loc'])
                    print(f"  loc = {estados.get(loc_val, loc_val)}")
                    print(f"  a = {model.get_py_value(states[i]['a'])}")
                    print(f"  b = {model.get_py_value(states[i]['b'])}")
                    print(f"  x = {model.get_py_value(states[i]['x'])}")
                    print(f"  y = {model.get_py_value(states[i]['y'])}")
                    print(f"  z = {model.get_py_value(states[i]['z'])}")
                return
            else:
                print(f"O invariante não é violado em {k} passos.")
                
    print(f"\nNenhum contra-exemplo encontrado até {K} passos. O invariante foi sempre verdadeiro!")


#bmc_para_invariante(declare, init, trans, invariante, 20) 

def init_com_restricao(state, n_val, m_val):
    cond_init_original = init(state) 

    cond_N_M = And(
        BVULE(n_val, state['a']),
        BVULE(state['a'], m_val),
        BVULE(n_val, state['b']),
        BVULE(state['b'], m_val)
    )
    
    return And(cond_init_original, cond_N_M)

def is_error(state):
    return Equals(state['loc'], LOC_ERROR)

def bmc_para_seguranca(declare, init_restrito, trans, prop_erro, K_passos, n_val, m_val):
    estados = {
        0:'INIT', 1:'SKIP', 2:'LEFT',
        3:'RIGHT', 4:'STOP', 5:'ERROR'
    }
    
    for k in range(1, K_passos + 1):
        with Solver(name="z3") as s:
            
            states = [declare(i) for i in range(k)]

            s.add_assertion(init_restrito(states[0], n_val, m_val))

            for i in range(k - 1):
                s.add_assertion(trans(states[i], states[i+1]))

            s.add_assertion(prop_erro(states[k-1]))

            if s.solve():
                print(f"--- VIOLAÇÃO DE SEGURANÇA ENCONTRADA no passo {k} ---")
                model = s.get_model()
                for i in range(k):
                    print(f"Estado {i}:")
                    loc_val = model.get_py_value(states[i]['loc'])
                    print(f"  loc = {estados.get(loc_val, loc_val)}")
                    print(f"  a = {model.get_py_value(states[i]['a'])}")
                    print(f"  b = {model.get_py_value(states[i]['b'])}")
                    print(f"  x = {model.get_py_value(states[i]['x'])}")
                    print(f"  y = {model.get_py_value(states[i]['y'])}")
                    print(f"  z = {model.get_py_value(states[i]['z'])}")
                return
            else:
                print(f"Passo {k}: Nenhum estado de erro acessível.")
                
    print(f"\nO programa é SEGURO até {K_passos} passos para a={n_val}, b={m_val}.")

# Tarefa 3
nbits = 16
N_PASSOS = 10
VALOR_N = BV(10, nbits)
VALOR_M = BV(30, nbits)

bmc_para_seguranca(declare, init_com_restricao, trans,is_error,N_PASSOS,VALOR_N,VALOR_M)