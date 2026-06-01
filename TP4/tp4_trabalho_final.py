"""
================================================================================
TRABALHO PRÁTICO 4 - Verificação de Segurança em Sistemas Híbridos
================================================================================
Controlo de Tráfego Marítimo num Canal com 15 Setores

ESTRUTURA:
1. Três Autómatos Híbridos: Navio 1, Navio 2, Semáforo
2. Modelo físico com parâmetros variáveis por setor (γ, V)
3. Verificação BMC para Segurança Suficiente e Forte
4. Visualização animada dos traços

Lógica Computacional 2025
================================================================================
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from z3 import *
import numpy as np
from typing import Dict, List, Optional
from enum import Enum
import time

# ============================================================================
# CONFIGURAÇÃO
# ============================================================================

# Constantes Físicas
SIGMA = 1.0    # Atrito
DT = 0.1       # Passo de tempo
Z_MAX = 1.0    # Tamanho do setor (1 km)

# Tipos de Zona
class TipoZona(Enum):
    ACELERACAO = "acel"
    DESACELERACAO = "desacel"
    CRUZEIRO = "cruz"
    BAIXA = "baixa"

# Parâmetros por setor: (gamma, V_max, tipo)
PARAMS = {
    13: (2.0, 2.0, TipoZona.ACELERACAO),
    11: (2.0, 2.0, TipoZona.ACELERACAO),
    9:  (1.0, 1.5, TipoZona.CRUZEIRO),
    5:  (1.0, 1.5, TipoZona.CRUZEIRO),
    7:  (1.0, 1.5, TipoZona.CRUZEIRO),
    1:  (0.5, 1.0, TipoZona.DESACELERACAO),
    3:  (0.5, 1.0, TipoZona.DESACELERACAO),
    0:  (0.3, 0.5, TipoZona.BAIXA),
    2:  (2.0, 2.0, TipoZona.ACELERACAO),
    4:  (2.0, 2.0, TipoZona.ACELERACAO),
    6:  (1.0, 1.5, TipoZona.CRUZEIRO),
    10: (1.0, 1.5, TipoZona.CRUZEIRO),
    8:  (1.0, 1.5, TipoZona.CRUZEIRO),
    14: (0.5, 1.0, TipoZona.DESACELERACAO),
    12: (0.5, 1.0, TipoZona.DESACELERACAO),
}

# Coordenadas visuais
COORDS = {
    0: (0, 0),
    1: (-1, 0.5), 3: (-1, -0.5),
    9: (-2, 1), 5: (-2, 0), 7: (-2, -1),
    13: (-3, 0.5), 11: (-3, -0.5),
    2: (1, 0.5), 4: (1, -0.5),
    6: (2, 1), 10: (2, 0), 8: (2, -1),
    14: (3, 0.5), 12: (3, -0.5)
}

CORES = {
    TipoZona.ACELERACAO: '#90EE90',
    TipoZona.DESACELERACAO: '#FFB6C1',
    TipoZona.CRUZEIRO: '#ADD8E6',
    TipoZona.BAIXA: '#FFE4B5'
}

# ============================================================================
# MODELO Z3
# ============================================================================

class Sistema:
    """Sistema Híbrido base."""
    
    def state(self, i: int) -> Dict:
        """Declara variáveis de estado."""
        return {
            's1': Int(f's1_{i}'), 'z1': Real(f'z1_{i}'), 'v1': Real(f'v1_{i}'),
            'path1': Int(f'p1_{i}'),
            's2': Int(f's2_{i}'), 'z2': Real(f'z2_{i}'), 'v2': Real(f'v2_{i}'),
            'path2': Int(f'p2_{i}'),
            't': Real(f't_{i}'), 'sig1': Int(f'sg1_{i}'), 'sig2': Int(f'sg2_{i}')
        }
    
    def init(self, s: Dict) -> BoolRef:
        """Estado inicial."""
        return And(
            Or(s['s1'] == 13, s['s1'] == 11), s['z1'] == 0, s['v1'] == 0,
            Or(s['path1'] == 0, s['path1'] == 1),
            Or(s['s2'] == 14, s['s2'] == 12), s['z2'] == 0, s['v2'] == 0,
            Or(s['path2'] == 0, s['path2'] == 1),
            s['t'] == 0, s['sig1'] == 0, s['sig2'] == 0
        )
    
    def gamma(self, sid, dir_ba=False):
        """Obtém aceleração γ."""
        r = RealVal(1.0)
        for i, (g, _, t) in PARAMS.items():
            val = g
            if dir_ba:
                if t == TipoZona.ACELERACAO: val = 0.5
                elif t == TipoZona.DESACELERACAO: val = 2.0
            r = If(sid == i, RealVal(val), r)
        return r
    
    def vmax(self, sid):
        """Obtém V máximo."""
        r = RealVal(1.5)
        for i, (_, v, _) in PARAMS.items():
            r = If(sid == i, RealVal(v), r)
        return r
    
    def dyn(self, v, sid, dir_ba=False):
        """Dinâmica: v' = v + dt*(γ - σv)"""
        g = self.gamma(sid, dir_ba)
        vm = self.vmax(sid)
        f = If(v <= vm, g, RealVal(0))
        vn = v + DT * (f - SIGMA * v)
        return If(vn < 0, RealVal(0), vn)
    
    def next1(self, s, p):
        """Próximo setor N1 (A→B)."""
        n = IntVal(-1)
        n = If(s == 13, If(p == 0, 9, 5), n)
        n = If(s == 11, If(p == 1, 7, 5), n)
        n = If(s == 9, 1, n)
        n = If(s == 7, 3, n)
        n = If(s == 5, If(p == 0, 1, 3), n)
        n = If(Or(s == 1, s == 3), 0, n)
        n = If(s == 0, If(p == 0, 2, 4), n)
        n = If(s == 2, If(p == 0, 6, 10), n)
        n = If(s == 4, If(p == 1, 8, 10), n)
        n = If(s == 6, 14, n)
        n = If(s == 8, 12, n)
        n = If(s == 10, If(p == 0, 14, 12), n)
        return n
    
    def next2(self, s, p):
        """Próximo setor N2 (B→A)."""
        n = IntVal(-1)
        n = If(s == 14, If(p == 0, 6, 10), n)
        n = If(s == 12, If(p == 1, 8, 10), n)
        n = If(s == 6, 2, n)
        n = If(s == 8, 4, n)
        n = If(s == 10, If(p == 0, 2, 4), n)
        n = If(Or(s == 2, s == 4), 0, n)
        n = If(s == 0, If(p == 0, 1, 3), n)
        n = If(s == 1, If(p == 0, 9, 5), n)
        n = If(s == 3, If(p == 1, 7, 5), n)
        n = If(s == 9, 13, n)
        n = If(s == 7, 11, n)
        n = If(s == 5, If(p == 0, 13, 11), n)
        return n


class SistemaInseguro(Sistema):
    """Sem semáforos."""
    
    def trans(self, s, sn):
        v1n = self.dyn(s['v1'], s['s1'])
        z1p = s['z1'] + DT * s['v1']
        nx1 = self.next1(s['s1'], s['path1'])
        cr1 = And(z1p >= Z_MAX, nx1 != -1)
        
        t1 = If(cr1,
            And(sn['s1'] == nx1, sn['z1'] == 0, sn['v1'] == s['v1'], sn['path1'] == s['path1']),
            And(sn['s1'] == s['s1'], sn['z1'] == If(z1p > Z_MAX, Z_MAX, z1p),
                sn['v1'] == v1n, sn['path1'] == s['path1']))
        
        v2n = self.dyn(s['v2'], s['s2'], True)
        z2p = s['z2'] + DT * s['v2']
        nx2 = self.next2(s['s2'], s['path2'])
        cr2 = And(z2p >= Z_MAX, nx2 != -1)
        
        t2 = If(cr2,
            And(sn['s2'] == nx2, sn['z2'] == 0, sn['v2'] == s['v2'], sn['path2'] == s['path2']),
            And(sn['s2'] == s['s2'], sn['z2'] == If(z2p > Z_MAX, Z_MAX, z2p),
                sn['v2'] == v2n, sn['path2'] == s['path2']))
        
        ts = And(sn['t'] == s['t'] + DT, sn['sig1'] == 0, sn['sig2'] == 0)
        return And(t1, t2, ts)


class SistemaSeguro(Sistema):
    """Com semáforos."""
    
    def trans(self, s, sn):
        nx1 = self.next1(s['s1'], s['path1'])
        nx2 = self.next2(s['s2'], s['path2'])
        
        sig1 = If(nx1 == s['s2'], 2, 0)
        sig2 = If(nx2 == s['s1'], 2, 0)
        conflict = And(nx1 == nx2, nx1 != -1)
        
        v1n = self.dyn(s['v1'], s['s1'])
        z1p = s['z1'] + DT * s['v1']
        at1 = z1p >= Z_MAX
        can1 = And(at1, nx1 != -1, sig1 != 2)
        
        t1 = If(can1,
            And(sn['s1'] == nx1, sn['z1'] == 0, sn['v1'] == s['v1'], sn['path1'] == s['path1']),
            If(at1,
                And(sn['s1'] == s['s1'], sn['z1'] == Z_MAX, sn['v1'] == 0, sn['path1'] == s['path1']),
                And(sn['s1'] == s['s1'], sn['z1'] == z1p, sn['v1'] == v1n, sn['path1'] == s['path1'])))
        
        v2n = self.dyn(s['v2'], s['s2'], True)
        z2p = s['z2'] + DT * s['v2']
        at2 = z2p >= Z_MAX
        can2 = And(at2, nx2 != -1, sig2 != 2, Not(conflict))
        
        t2 = If(can2,
            And(sn['s2'] == nx2, sn['z2'] == 0, sn['v2'] == s['v2'], sn['path2'] == s['path2']),
            If(at2,
                And(sn['s2'] == s['s2'], sn['z2'] == Z_MAX, sn['v2'] == 0, sn['path2'] == s['path2']),
                And(sn['s2'] == s['s2'], sn['z2'] == z2p, sn['v2'] == v2n, sn['path2'] == s['path2'])))
        
        ts = And(sn['t'] == s['t'] + DT, sn['sig1'] == sig1, sn['sig2'] == If(conflict, 2, sig2))
        return And(t1, t2, ts)


# ============================================================================
# VERIFICAÇÃO BMC
# ============================================================================

def colisao(s):
    return s['s1'] == s['s2']

def deadlock(s, sis):
    nx1 = sis.next1(s['s1'], s['path1'])
    nx2 = sis.next2(s['s2'], s['path2'])
    b1 = And(s['z1'] == Z_MAX, s['v1'] == 0, Or(nx1 == -1, nx1 == s['s2']))
    b2 = And(s['z2'] == Z_MAX, s['v2'] == 0, Or(nx2 == -1, nx2 == s['s1']))
    return And(b1, b2)

def extract(model, states, k):
    def gr(v):
        val = model[v]
        if val is None: return 0.0
        try: return float(val.numerator_as_long()) / float(val.denominator_as_long())
        except: return float(val.as_long()) if hasattr(val, 'as_long') else 0.0
    def gi(v):
        val = model[v]
        return val.as_long() if val else 0
    
    return [{'t': gr(states[k]['t']),
             's1': gi(states[k]['s1']), 'z1': gr(states[k]['z1']), 'v1': gr(states[k]['v1']),
             's2': gi(states[k]['s2']), 'z2': gr(states[k]['z2']), 'v2': gr(states[k]['v2']),
             'sig1': gi(states[k]['sig1']), 'sig2': gi(states[k]['sig2'])} for k in range(k+1)]

def bmc(sis, kmax, check_dead=False, verbose=True):
    """Bounded Model Checking."""
    solver = Solver()
    states = [sis.state(0)]
    solver.add(sis.init(states[0]))
    
    # Forçar conflito
    solver.add(states[0]['path1'] == 0, states[0]['path2'] == 0)
    solver.add(states[0]['s1'] == 13, states[0]['s2'] == 14)
    
    if verbose: print(f"  BMC até k={kmax}...")
    t0 = time.time()
    
    for k in range(1, kmax + 1):
        states.append(sis.state(k))
        solver.add(sis.trans(states[k-1], states[k]))
        
        solver.push()
        prop = colisao(states[k])
        if check_dead:
            prop = Or(prop, deadlock(states[k], sis))
        solver.add(prop)
        
        if solver.check() == sat:
            m = solver.model()
            dt = time.time() - t0
            
            is_col = is_true(m.eval(colisao(states[k])))
            is_dead = check_dead and is_true(m.eval(deadlock(states[k], sis)))
            
            if verbose:
                print(f"\n  ⚠️ FALHA em k={k} (t={k*DT:.1f}s)")
                if is_col:
                    print(f"     COLISÃO em S{m[states[k]['s1']].as_long()}")
                if is_dead:
                    print(f"     DEADLOCK")
                print(f"     Tempo: {dt:.2f}s")
            
            return extract(m, states, k)
        
        solver.pop()
        if verbose and k % 10 == 0: print(f"  k={k} OK...")
    
    if verbose:
        print(f"\n  ✓ Seguro até k={kmax} ({time.time()-t0:.2f}s)")
    return None


# ============================================================================
# VISUALIZAÇÃO
# ============================================================================

def animar(trace, titulo="Simulação"):
    if not trace: return None
    
    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={'width_ratios': [3, 1]})
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect('equal')
    ax.set_title(titulo, fontsize=13, fontweight='bold')
    ax.axis('off')
    
    # Setores
    pdict = {}
    for sid, (x, y) in COORDS.items():
        cor = CORES[PARAMS[sid][2]]
        rect = patches.FancyBboxPatch((x-0.4, y-0.4), 0.8, 0.8, boxstyle="round,pad=0.05",
                                       linewidth=2, edgecolor='black', facecolor=cor, alpha=0.7)
        ax.add_patch(rect)
        ax.text(x, y, f"S{sid}", ha='center', va='center', fontsize=9, fontweight='bold')
        pdict[sid] = rect
    
    ax.text(-3.5, 1.5, "PORTO A", fontsize=10, fontweight='bold', color='blue')
    ax.text(3.0, 1.5, "PORTO B", fontsize=10, fontweight='bold', color='red')
    
    ship1, = ax.plot([], [], 'bo', markersize=14, zorder=10)
    ship2, = ax.plot([], [], 'rs', markersize=14, zorder=10)
    
    sem1 = patches.Circle((-4, 1.2), 0.1, color='green')
    sem2 = patches.Circle((4, 1.2), 0.1, color='green')
    ax.add_patch(sem1)
    ax.add_patch(sem2)
    
    ax_info.axis('off')
    info = ax_info.text(0.05, 0.9, "", transform=ax_info.transAxes, fontsize=9,
                        verticalalignment='top', fontfamily='monospace')
    status = ax.text(0, -2.0, "", ha='center', fontsize=10,
                     bbox=dict(facecolor='white', edgecolor='black'))
    
    def pnext(s, n1):
        t = {13:9, 11:7, 9:1, 7:3, 5:1, 1:0, 3:0, 0:2, 2:6, 4:8, 6:14, 8:12, 10:14}
        if not n1:
            t = {14:6, 12:8, 6:2, 8:4, 10:2, 2:0, 4:0, 0:1, 1:9, 3:7, 5:13, 9:13, 7:11}
        return t.get(s)
    
    def gpos(s, z, nx):
        if s not in COORDS: return 0, 0
        x, y = COORDS[s]
        if nx and nx in COORDS:
            xn, yn = COORDS[nx]
            z = min(z, 0.85)
            return x + z*(xn-x), y + z*(yn-y)
        return x, y
    
    scol = {0: 'green', 1: 'yellow', 2: 'red'}
    
    def update(frame):
        d = trace[frame]
        
        nx1 = pnext(d['s1'], True)
        nx2 = pnext(d['s2'], False)
        x1, y1 = gpos(d['s1'], d['z1'], nx1)
        x2, y2 = gpos(d['s2'], d['z2'], nx2)
        
        ship1.set_data([x1], [y1])
        ship2.set_data([x2], [y2])
        
        sem1.set_color(scol.get(d.get('sig1', 0), 'gray'))
        sem2.set_color(scol.get(d.get('sig2', 0), 'gray'))
        
        col = d['s1'] == d['s2']
        for sid, rect in pdict.items():
            if col and sid == d['s1']:
                rect.set_facecolor('red')
                rect.set_alpha(1.0)
            else:
                rect.set_facecolor(CORES[PARAMS[sid][2]])
                rect.set_alpha(0.7)
        
        if col:
            status.set_text(f"⚠️ COLISÃO S{d['s1']}! ⚠️")
            status.set_color('red')
        else:
            status.set_text(f"t = {d['t']:.1f}s")
            status.set_color('black')
        
        sn = {0: '🟢', 1: '🟡', 2: '🔴'}
        info.set_text(f"""
━━━━━━━━━━━━━━━━━━━━━━━
   ESTADO DO SISTEMA
━━━━━━━━━━━━━━━━━━━━━━━
 t = {d['t']:.2f} s

 NAVIO 1 (A→B)
   Setor: S{d['s1']}
   z: {d['z1']:.3f} km
   v: {d['v1']:.3f} km/s
   Sem: {sn.get(d.get('sig1',0))}

 NAVIO 2 (B→A)
   Setor: S{d['s2']}
   z: {d['z2']:.3f} km
   v: {d['v2']:.3f} km/s
   Sem: {sn.get(d.get('sig2',0))}
━━━━━━━━━━━━━━━━━━━━━━━
""")
        return ship1, ship2, status, info, sem1, sem2, *pdict.values()
    
    return animation.FuncAnimation(fig, update, frames=len(trace), interval=150, blit=False, repeat=True)


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("""
╔══════════════════════════════════════════════════════════════════╗
║     TP4 - VERIFICAÇÃO DE SISTEMAS HÍBRIDOS                       ║
║     Controlo de Tráfego Marítimo                                 ║
║                                                                  ║
║     • 3 Autómatos: Navio 1, Navio 2, Semáforo                    ║
║     • Parâmetros por setor (γ, V)                                ║
║     • BMC: Colisão + Deadlock                                    ║
╚══════════════════════════════════════════════════════════════════╝
    """)
    
    # Sistema Inseguro
    print("\n" + "="*60)
    print("PARTE 1: Sistema SEM Semáforos (INSEGURO)")
    print("="*60)
    
    sis_ins = SistemaInseguro()
    trace_ins = bmc(sis_ins, 50, False, True)
    
    # Sistema Seguro
    print("\n" + "="*60)
    print("PARTE 2: Sistema COM Semáforos (SEGURO)")
    print("="*60)
    
    sis_seg = SistemaSeguro()
    trace_seg = bmc(sis_seg, 50, True, True)
    
    # Resumo
    print("\n" + "="*60)
    print("RESUMO")
    print("="*60)
    print(f" Inseguro: {'COLISÃO ✗' if trace_ins else 'OK ✓'}")
    print(f" Seguro:   {'FALHA ✗' if trace_seg else 'VERIFICADO ✓'}")
    
    # Visualização
    if trace_ins:
        print("\n>>> Animação sistema INSEGURO...")
        ani = animar(trace_ins, "Sistema INSEGURO - Colisão")
        plt.show()
    
    if trace_seg:
        print("\n>>> Animação sistema SEGURO...")
        ani = animar(trace_seg, "Sistema SEGURO - Análise")
        plt.show()


if __name__ == "__main__":
    main()
