#!/usr/bin/env python3
"""
Task 2: Nonlinear MPC vs ASMCA
================================
This script replaces the high-level ASMCA controller with a Nonlinear MPC (NMPC)
and compares their performance under the same fault scenarios.

Run: python run_task2.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import time

# ═══════════════════════════════════════════════════════════════════════
# 1. PHYSICAL PARAMETERS
# ═══════════════════════════════════════════════════════════════════════
m   = 2.0;  g = 9.81
Ixx = 0.0820;  Iyy = 0.0845;  Izz = 0.1377
Ld  = 0.23                 
b_t = 2.8e-6               
d_t = 7.5e-8               
Kd  = np.array([0.01, 0.012, 0.012, 0.012])

N_ROT = 8;  N_VIR = 4
U_MIN = 0.0;  U_MAX = 4.5  

Ba = np.array([
    [ 1,     1,     1,     1,     1,     1,     1,     1    ],  # Fz
    [ 0,     0,     Ld,    Ld,    0,     0,    -Ld,   -Ld   ],  # Mphi
    [ Ld,    Ld,    0,     0,    -Ld,   -Ld,   0,     0     ],  # Mthe
    [ d_t/b_t,-d_t/b_t,-d_t/b_t,d_t/b_t,
      d_t/b_t,-d_t/b_t,-d_t/b_t,d_t/b_t ],                    # Mpsi
])
Ba_pinv = np.linalg.pinv(Ba)

# ═══════════════════════════════════════════════════════════════════════
# 2. CONTROLLER GAINS (ASMCA)
# ═══════════════════════════════════════════════════════════════════════
K1  = np.array([25., 100., 100., 25.])      
K2  = np.array([10.,  20.,  20., 10.])      
KS  = np.array([ 5.,  10.,  10.,  5.])      
PHI = 0.2                                   
ADAPT_GAIN = np.array([0.5, 2.0, 2.0, 0.5]) 

# ═══════════════════════════════════════════════════════════════════════
# 3. SIM SETTINGS
# ═══════════════════════════════════════════════════════════════════════
T_SIM  = 40.0;  DT = 0.002;  T_FAULT = 20.0
Z_REF  = 1.0
P_AMP  = np.deg2rad(4.0)
P_FREQ = 0.25
DIST   = 0.08       

# NMPC settings
DT_NMPC = 0.02      # 50 Hz NMPC rate
N_HORIZON = 20      # 0.4 seconds horizon
STEPS_PER_NMPC = int(DT_NMPC / DT)

# ═══════════════════════════════════════════════════════════════════════
# 4. NMPC SETUP (CasADi)
# ═══════════════════════════════════════════════════════════════════════
def setup_nmpc():
    z    = ca.MX.sym('z');   zd  = ca.MX.sym('zd')
    ph   = ca.MX.sym('ph');  phd = ca.MX.sym('phd')
    th   = ca.MX.sym('th');  thd = ca.MX.sym('thd')
    ps   = ca.MX.sym('ps');  psd = ca.MX.sym('psd')
    x = ca.vertcat(z, zd, ph, phd, th, thd, ps, psd)
    
    u = ca.MX.sym('u', 8)
    L_opt = ca.MX.sym('L_opt', 8)
    u_actual = u * L_opt
    
    nu = ca.mtimes(ca.DM(Ba), u_actual)
    Uz = nu[0]; Up = nu[1]; Ut = nu[2]; Uy = nu[3]
    
    cp = ca.cos(ph); sp = ca.sin(ph)
    ct = ca.cos(th); st = ca.sin(th)
    
    zdd  = -g + (cp*ct/m)*Uz - Kd[0]*zd/m
    phdd = thd*psd*(Iyy-Izz)/Ixx + Up/Ixx - Kd[1]*Ld*phd/Ixx
    thdd = phd*psd*(Izz-Ixx)/Iyy + Ut/Iyy - Kd[2]*Ld*thd/Iyy
    psdd = phd*thd*(Ixx-Iyy)/Izz + Uy/Izz - Kd[3]*psd/Izz
    
    xdot = ca.vertcat(zd, zdd, phd, phdd, thd, thdd, psd, psdd)
    f = ca.Function('f', [x, u, L_opt], [xdot])
    
    X0 = ca.MX.sym('X0', 8); U0 = ca.MX.sym('U0', 8); L0 = ca.MX.sym('L0', 8)
    k1 = f(X0, U0, L0)
    k2 = f(X0 + DT_NMPC/2 * k1, U0, L0)
    k3 = f(X0 + DT_NMPC/2 * k2, U0, L0)
    k4 = f(X0 + DT_NMPC * k3, U0, L0)
    X_next = X0 + DT_NMPC/6 * (k1 + 2*k2 + 2*k3 + k4)
    F = ca.Function('F', [X0, U0, L0], [X_next])
    
    opti = ca.Opti()
    X = opti.variable(8, N_HORIZON+1)
    U = opti.variable(8, N_HORIZON)
    P = opti.parameter(8)
    Ref = opti.parameter(8, N_HORIZON+1)
    L_param = opti.parameter(8)
    
    opti.subject_to(X[:,0] == P)
    
    Q = ca.diag([200, 10, 500, 20, 500, 20, 500, 20])
    R = ca.diag([0.1]*8)
    
    cost = 0
    for k in range(N_HORIZON):
        opti.subject_to(X[:,k+1] == F(X[:,k], U[:,k], L_param))
        err = X[:,k] - Ref[:,k]
        cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([U[:,k].T, R, U[:,k]])
        
        for i in range(8):
            opti.subject_to(opti.bounded(0, U[i,k], U_MAX))
        
    err_N = X[:,N_HORIZON] - Ref[:,N_HORIZON]
    cost += ca.mtimes([err_N.T, Q*10, err_N])
    
    opti.minimize(cost)
    
    p_opts = {"expand": True, "print_time": False}
    s_opts = {"max_iter": 100, "print_level": 0, "sb": "yes"}
    opti.solver("ipopt", p_opts, s_opts)
    
    return opti, X, U, P, Ref, L_param

nmpc_opti, nmpc_X, nmpc_U, nmpc_P, nmpc_Ref, nmpc_L = setup_nmpc()

# ═══════════════════════════════════════════════════════════════════════
# 5. DYNAMICS & REFERENCE
# ═══════════════════════════════════════════════════════════════════════
def disturb(t):
    return DIST * np.array([0.3*np.sin(0.5*t),
                            0.2*np.sin(1.1*t+1), 0.2*np.cos(0.8*t+0.5),
                            0.1*np.sin(0.6*t+2)])

def dynamics(s, nu, t):
    z,zd,ph,phd,th,thd,ps,psd = s
    Uz,Up,Ut,Uy = nu
    cp,sp = np.cos(ph),np.sin(ph); ct,st = np.cos(th),np.sin(th)
    d = disturb(t)
    zdd  = -g + (cp*ct/m)*Uz - Kd[0]*zd/m + d[0]
    phdd = thd*psd*(Iyy-Izz)/Ixx + Up/Ixx - Kd[1]*Ld*phd/Ixx + d[1]
    thdd = phd*psd*(Izz-Ixx)/Iyy + Ut/Iyy - Kd[2]*Ld*thd/Iyy + d[2]
    psdd = phd*thd*(Ixx-Iyy)/Izz + Uy/Izz - Kd[3]*psd/Izz     + d[3]
    return np.array([zd,zdd, phd,phdd, thd,thdd, psd,psdd])

def rk4(s,nu,dt,t):
    k1=dynamics(s,nu,t); k2=dynamics(s+.5*dt*k1,nu,t+.5*dt)
    k3=dynamics(s+.5*dt*k2,nu,t+.5*dt); k4=dynamics(s+dt*k3,nu,t+dt)
    return s+(dt/6)*(k1+2*k2+2*k3+k4)

def ref(t):
    from scipy.signal import square
    freq = np.pi / 20.0
    td = P_AMP * square(freq * t)
    tdd = 0.0
    tddd = 0.0
    return (np.array([Z_REF,0,td,0]), np.array([0,0,tdd,0]), np.array([0,0,tddd,0]))

def ref_state(t):
    """Returns full 8-state reference."""
    from scipy.signal import square
    freq = np.pi / 20.0
    td = P_AMP * square(freq * t)
    tdd = 0.0
    return np.array([Z_REF, 0, 0, 0, td, tdd, 0, 0])

# ═══════════════════════════════════════════════════════════════════════
# 6. ASMCA HELPERS
# ═══════════════════════════════════════════════════════════════════════
sat = lambda x: np.clip(x/PHI, -1., 1.)

def errs(s,pd,vd):
    return (np.array([s[0]-pd[0], s[2]-pd[1], s[4]-pd[2], s[6]-pd[3]]),
            np.array([s[1]-vd[0], s[3]-vd[1], s[5]-vd[2], s[7]-vd[3]]))

def f_i(s):
    _,_,_,phd,_,thd,_,psd = s
    return np.array([-g, thd*psd*(Iyy-Izz)/Ixx, phd*psd*(Izz-Ixx)/Iyy, phd*thd*(Ixx-Iyy)/Izz])

def h_i(s):
    return np.array([np.cos(s[2])*np.cos(s[4])/m, 1./Ixx, 1./Iyy, 1./Izz])

def asmc(s, pd, vd, ad, sig, Gh):
    f = f_i(s); e,ed = errs(s,pd,vd)
    ff = ad - K2*ed - K1*e - f
    nu = Gh*ff - Gh*KS*sat(sig)
    sd = sig - PHI*sat(sig)
    Gd = ADAPT_GAIN * (-ff + KS*sat(sig)) * sd
    Gd -= 0.001 * (Gh - h_i(s)**(-1))
    return nu, Gd

# ═══════════════════════════════════════════════════════════════════════
# 7. CONTROL ALLOCATION
# ═══════════════════════════════════════════════════════════════════════
def fault_L(t, sc):
    L = np.ones(N_ROT)
    if t >= T_FAULT:
        if sc == 1: L[0] = 0.0
        elif sc == 2: L[0] = 0.0; L[4] = 0.6
    return L

def ca_with_fdd(nu_d, L):
    L_safe = np.maximum(L, 1e-8)
    B_eff  = Ba @ np.diag(L_safe)
    W = np.diag(L_safe**2)
    M = B_eff @ W @ B_eff.T + 1e-8*np.eye(N_VIR)
    try: Mi = np.linalg.inv(M)
    except: Mi = np.linalg.pinv(M)
    u = W @ B_eff.T @ Mi @ nu_d
    return np.clip(u, U_MIN, U_MAX)

# ═══════════════════════════════════════════════════════════════════════
# 8. SIMULATION
# ═══════════════════════════════════════════════════════════════════════
def simulate(ctrl, sc, T=T_SIM, dt=DT):
    N = int(T/dt); ta = np.linspace(0,T,N+1)
    x  = np.zeros((N+1,8)); x[0] = [Z_REF,0,0,0,0,0,0,0]
    nud = np.zeros((N,N_VIR)); nua = np.zeros((N,N_VIR))
    ul  = np.zeros((N,N_ROT))
    
    # ASMCA specifics
    h0 = h_i(x[0])
    Gh = 1./np.where(np.abs(h0)>1e-10, h0, 1e-10)
    ie = np.zeros(N_VIR)
    
    # NMPC specifics
    last_u_nmpc_step = np.ones(8) * (m*g/8)
    last_u_nmpc  = np.ones((8, N_HORIZON)) * (m*g/8) # warm start
    last_x_nmpc  = np.zeros((8, N_HORIZON+1))

    for k in range(N):
        t = ta[k]; s = x[k]
        L = fault_L(t, sc)

        if ctrl == 'ASMCA':
            pd,vd,ad = ref(t)
            e,ed = errs(s,pd,vd)
            for i in range(N_VIR):
                if np.abs(e[i]) < 0.5: ie[i] += e[i]*dt
                ie[i] = np.clip(ie[i], -2.0, 2.0)
            sig = ed + K2*e + K1*ie
            nu, Gd = asmc(s, pd, vd, ad, sig, Gh)
            Gh = np.maximum(Gh + Gd*dt, 1e-4)
            Gh = np.minimum(Gh, 10.0 / np.array([1/m, 1/Ixx, 1/Iyy, 1/Izz]))
            u = ca_with_fdd(nu, L)
            u_act = L * u
            nu_act = Ba @ u_act
            nud[k]=nu; nua[k]=nu_act; ul[k]=u
            
        elif ctrl == 'NMPC':
            if k % STEPS_PER_NMPC == 0:
                # generate ref trajectory over horizon
                ref_traj = np.zeros((8, N_HORIZON+1))
                for h in range(N_HORIZON+1):
                    ref_traj[:,h] = ref_state(t + h*DT_NMPC)
                    
                nmpc_opti.set_value(nmpc_P, s)
                nmpc_opti.set_value(nmpc_Ref, ref_traj)
                nmpc_opti.set_value(nmpc_L, L)
                
                # Warm start
                nmpc_opti.set_initial(nmpc_X, last_x_nmpc)
                nmpc_opti.set_initial(nmpc_U, last_u_nmpc)
                
                try:
                    sol = nmpc_opti.solve()
                    last_u_nmpc_step = np.array(sol.value(nmpc_U)[:,0]).flatten()
                    last_u_nmpc = sol.value(nmpc_U)
                    last_x_nmpc = sol.value(nmpc_X)
                except Exception as e:
                    last_u_nmpc_step = np.array(nmpc_opti.debug.value(nmpc_U)[:,0]).flatten()
            
            u = last_u_nmpc_step
            u_act = L * u
            nu_act = Ba @ u_act
            nud[k] = nu_act; nua[k] = nu_act; ul[k] = u

        x[k+1] = rk4(s, nu_act, dt, t)
        
        # safety clip
        x[k+1, 0] = np.clip(x[k+1, 0], -5, 20)
        x[k+1, 2] = np.clip(x[k+1, 2], -np.pi, np.pi)
        x[k+1, 4] = np.clip(x[k+1, 4], -np.pi, np.pi)
        x[k+1, 6] = np.clip(x[k+1, 6], -np.pi, np.pi)

    return dict(t=ta, x=x, nu_d=nud, nu_act=nua, u=ul)

# ═══════════════════════════════════════════════════════════════════════
# 9. PLOTTING
# ═══════════════════════════════════════════════════════════════════════
def plot_comparisons(r_asmc, r_nmpc, sc, d):
    plt.rcParams.update({'figure.figsize':(12,6), 'font.size':12, 'font.family':'serif',
                         'axes.grid':True, 'grid.alpha':0.3})
    t = r_asmc['t'][:-1]
    
    # 1. Pitch Tracking
    fig, ax = plt.subplots()
    ref_pitch = np.rad2deg([ref_state(tt)[4] for tt in t])
    ax.plot(t, ref_pitch, 'k--', lw=2, label='Desired Pitch')
    ax.plot(t, np.rad2deg(r_asmc['x'][:-1,4]), 'b-', lw=1.5, label='ASMCA')
    ax.plot(t, np.rad2deg(r_nmpc['x'][:-1,4]), 'r-.', lw=1.5, label='NMPC')
    ax.axvline(T_FAULT, color='gray', ls=':', alpha=.7)
    ax.set(xlabel='Time (s)', ylabel='Pitch Angle (deg)', title=f'Pitch Tracking Comparison - Scenario {sc}')
    ax.legend()
    fig.tight_layout(); fig.savefig(d/f'compare_pitch_sc{sc}.png', dpi=200); plt.close()
    
    # 2. Altitude Tracking
    fig, ax = plt.subplots()
    ax.plot(t, np.ones_like(t)*Z_REF, 'k--', lw=2, label='Desired Altitude')
    ax.plot(t, r_asmc['x'][:-1,0], 'b-', lw=1.5, label='ASMCA')
    ax.plot(t, r_nmpc['x'][:-1,0], 'r-.', lw=1.5, label='NMPC')
    ax.axvline(T_FAULT, color='gray', ls=':', alpha=.7)
    ax.set(xlabel='Time (s)', ylabel='Altitude (m)', title=f'Altitude Tracking Comparison - Scenario {sc}')
    ax.legend()
    fig.tight_layout(); fig.savefig(d/f'compare_alt_sc{sc}.png', dpi=200); plt.close()

# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    d = Path(__file__).parent / 'results'; d.mkdir(exist_ok=True)
    print("="*60)
    print("  Task 2: ASMCA vs NMPC")
    print("="*60)
    
    for sc in [1, 2]:
        print(f"\n> Scenario {sc}")
        print("  Running ASMCA ...", end=" ", flush=True)
        r_asmc = simulate('ASMCA', sc)
        print("done.")
        
        print("  Running NMPC ...", end=" ", flush=True)
        t0 = time.time()
        r_nmpc = simulate('NMPC', sc)
        print(f"done. ({time.time()-t0:.1f}s)")
        
        plot_comparisons(r_asmc, r_nmpc, sc, d)

    print("\n[OK] Task 2 completed. Figures saved to results/")
    for f in sorted(d.glob('compare*.png')): print(f"    {f.name}")
    print("="*60)
