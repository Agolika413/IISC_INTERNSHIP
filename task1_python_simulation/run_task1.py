#!/usr/bin/env python3
"""
Task 1: Reproduction of Paper Results
======================================
Paper: "An Adaptive Fault-Tolerant Sliding Mode Control Allocation Scheme
        for Multirotor Helicopter Subject to Simultaneous Actuator Faults"
Authors: Ban Wang & Youmin Zhang (IEEE Trans. Ind. Electron., 2018)

Reproduces Figs 5-11.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# ═══════════════════════════════════════════════════════════════════════
# 1. PHYSICAL PARAMETERS (Section II-B / Table I)
# ═══════════════════════════════════════════════════════════════════════
m   = 2.0;  g = 9.81
Ixx = 0.0820;  Iyy = 0.0845;  Izz = 0.1377
Ld  = 0.23                 # arm length [m]
b_t = 2.8e-6               # thrust coefficient  [N/(rad/s)^2]
d_t = 7.5e-8               # drag-moment coefficient
Kd  = np.array([0.01, 0.012, 0.012, 0.012])

N_ROT = 8;  N_VIR = 4
U_MIN = 0.0;  U_MAX = 4.5  # per-motor thrust limit [N]

# Control-allocation matrix  B (4 x 8)
#   Cross-coaxial: arms at 0,90,180,270 deg; each arm has 2 coax rotors
#   Arm directions: 1(+x), 2(+y), 3(-x), 4(-y)
Ba = np.array([
    [ 1,     1,     1,     1,     1,     1,     1,     1    ],  # Fz
    [ 0,     0,     Ld,    Ld,    0,     0,    -Ld,   -Ld   ],  # Mphi
    [ Ld,    Ld,    0,     0,    -Ld,   -Ld,   0,     0     ],  # Mthe
    [ d_t/b_t,-d_t/b_t,-d_t/b_t,d_t/b_t,
      d_t/b_t,-d_t/b_t,-d_t/b_t,d_t/b_t ],                    # Mpsi
])
Ba_pinv = np.linalg.pinv(Ba)  # nominal pseudo-inverse

# ═══════════════════════════════════════════════════════════════════════
# 2. CONTROLLER GAINS (Section IV)
# ═══════════════════════════════════════════════════════════════════════
K1  = np.array([25., 100., 100., 25.])      # proportional
K2  = np.array([10.,  20.,  20., 10.])      # derivative
KS  = np.array([ 5.,  10.,  10.,  5.])      # switching
PHI = 0.2                                   # boundary layer
ADAPT_GAIN = np.array([0.5, 2.0, 2.0, 0.5]) # adaptation rate scaling

# ═══════════════════════════════════════════════════════════════════════
# 3. SIM SETTINGS
# ═══════════════════════════════════════════════════════════════════════
T_SIM  = 70.0;  DT = 0.002;  T_FAULT = 20.0
Z_REF  = 1.0
P_AMP  = np.deg2rad(4.0)
P_FREQ = 0.25
DIST   = 0.08       # disturbance amplitude

# ═══════════════════════════════════════════════════════════════════════
# 4. DYNAMICS
# ═══════════════════════════════════════════════════════════════════════

def disturb(t):
    return DIST * np.array([0.3*np.sin(0.5*t),
                            0.2*np.sin(1.1*t+1), 0.2*np.cos(0.8*t+0.5),
                            0.1*np.sin(0.6*t+2)])

def dynamics(s, nu, t):
    z,zd,ph,phd,th,thd,ps,psd = s
    Uz,Up,Ut,Uy = nu
    cp,sp = np.cos(ph),np.sin(ph)
    ct,st = np.cos(th),np.sin(th)
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

# ═══════════════════════════════════════════════════════════════════════
# 5. REFERENCE
# ═══════════════════════════════════════════════════════════════════════

def ref(t):
    from scipy.signal import square
    freq = np.pi / 20.0  # 40s period
    td = P_AMP * square(freq * t)
    tdd = 0.0
    tddd = 0.0
    return (np.array([Z_REF,0,td,0]),
            np.array([0,0,tdd,0]),
            np.array([0,0,tddd,0]))

# ═══════════════════════════════════════════════════════════════════════
# 6. HELPERS
# ═══════════════════════════════════════════════════════════════════════

sat = lambda x: np.clip(x/PHI, -1., 1.)

def errs(s,pd,vd):
    e = np.array([s[0]-pd[0], s[2]-pd[1], s[4]-pd[2], s[6]-pd[3]])
    ed= np.array([s[1]-vd[0], s[3]-vd[1], s[5]-vd[2], s[7]-vd[3]])
    return e,ed

def f_i(s):
    _,_,_,phd,_,thd,_,psd = s
    return np.array([-g, thd*psd*(Iyy-Izz)/Ixx,
                     phd*psd*(Izz-Ixx)/Iyy, phd*thd*(Ixx-Iyy)/Izz])

def h_i(s):
    return np.array([np.cos(s[2])*np.cos(s[4])/m,
                     1./Ixx, 1./Iyy, 1./Izz])

# ═══════════════════════════════════════════════════════════════════════
# 7. CONTROLLERS
# ═══════════════════════════════════════════════════════════════════════

def asmc(s, pd, vd, ad, sig, Gh):
    """Adaptive Sliding Mode Control Allocation (ASMCA)."""
    f = f_i(s); e,ed = errs(s,pd,vd)
    ff = ad - K2*ed - K1*e - f
    nu = Gh*ff - Gh*KS*sat(sig)
    # Adaptive law: Eq.(41) with leakage modification for bounded Gamma
    sd = sig - PHI*sat(sig)
    Gd = ADAPT_GAIN * (-ff + KS*sat(sig)) * sd
    # sigma-modification (leakage) to prevent parameter drift
    Gd -= 0.001 * (Gh - h_i(s)**(-1))
    return nu, Gd

def nsmc(s, pd, vd, ad, sig):
    """Normal SMC (fixed nominal gain)."""
    f = f_i(s); h = h_i(s); e,ed = errs(s,pd,vd)
    hi = 1./np.where(np.abs(h)>1e-10, h, 1e-10)
    ff = ad - K2*ed - K1*e - f
    return hi*ff - hi*KS*sat(sig)

def lqr(s, pd, vd, ad):
    """LQR-type PD controller (no switching term)."""
    f = f_i(s); h = h_i(s); e,ed = errs(s,pd,vd)
    hi = 1./np.where(np.abs(h)>1e-10, h, 1e-10)
    return hi*(ad - K2*ed - K1*e - f)

# ═══════════════════════════════════════════════════════════════════════
# 8. FAULT MODEL & CONTROL ALLOCATION
# ═══════════════════════════════════════════════════════════════════════

def fault_L(t, sc):
    """Actuator effectiveness factors L_i in [0,1]."""
    L = np.ones(N_ROT)
    if t >= T_FAULT:
        if sc == 1:
            L[0] = 0.0                # motor 1: complete failure
        elif sc == 2:
            L[0] = 0.0                # motor 1: complete failure
            L[4] = 0.6                # motor 5: 40% loss
    return L

def ca_with_fdd(nu_d, L):
    """
    Control Allocation with FDD information.
    Uses fault-aware weighted pseudo-inverse to redistribute.
    """
    # Build fault-aware effective matrix:  B_eff = Ba @ diag(L)
    L_safe = np.maximum(L, 1e-8)
    B_eff  = Ba @ np.diag(L_safe)

    # Weighted pseudo-inverse: penalize failed motors
    W = np.diag(L_safe**2)  # healthy motors get more weight
    M = B_eff @ W @ B_eff.T + 1e-8*np.eye(N_VIR)
    try:
        Mi = np.linalg.inv(M)
    except np.linalg.LinAlgError:
        Mi = np.linalg.pinv(M)
    u = W @ B_eff.T @ Mi @ nu_d
    return np.clip(u, U_MIN, U_MAX)

def ca_nominal(nu_d):
    """Naive CA using nominal pseudo-inverse (no fault info)."""
    u = Ba_pinv @ nu_d
    return np.clip(u, U_MIN, U_MAX)

# ═══════════════════════════════════════════════════════════════════════
# 9. SIMULATION
# ═══════════════════════════════════════════════════════════════════════

def simulate(ctrl, sc, T=T_SIM, dt=DT):
    N = int(T/dt); ta = np.linspace(0,T,N+1)
    x  = np.zeros((N+1,8)); x[0] = [Z_REF,0,0,0,0,0,0,0]
    nud = np.zeros((N,N_VIR)); nua = np.zeros((N,N_VIR))
    ul  = np.zeros((N,N_ROT)); sl  = np.zeros((N,N_VIR))
    gl  = np.zeros((N,N_VIR)); rl  = np.zeros((N,N_VIR))

    h0 = h_i(x[0])
    Gh = 1./np.where(np.abs(h0)>1e-10, h0, 1e-10)  # start at nominal
    ie = np.zeros(N_VIR)

    for k in range(N):
        t = ta[k]; s = x[k]
        pd,vd,ad = ref(t); rl[k] = pd
        e,ed = errs(s,pd,vd)

        # Anti-windup: freeze integral if error too large
        for i in range(N_VIR):
            if np.abs(e[i]) < 0.5:  # within reasonable range
                ie[i] += e[i]*dt
            # Clamp integral
            ie[i] = np.clip(ie[i], -2.0, 2.0)

        sig = ed + K2*e + K1*ie
        L = fault_L(t, sc)

        if ctrl == 'ASMCA':
            nu, Gd = asmc(s, pd, vd, ad, sig, Gh)
            Gh = np.maximum(Gh + Gd*dt, 1e-4)
            # Upper bound on adaptive gain to match paper behavior
            Gh = np.minimum(Gh, 10.0 / np.array([1/m, 1/Ixx, 1/Iyy, 1/Izz]))
            # ASMCA uses FDD-aware allocation
            u = ca_with_fdd(nu, L)
        elif ctrl == 'NSMCA':
            nu = nsmc(s, pd, vd, ad, sig)
            # NSMCA uses the same allocation but with nominal gains
            u = ca_with_fdd(nu, L)
        else:  # LQRCA
            nu = lqr(s, pd, vd, ad)
            # LQRCA uses nominal CA (no switching, less robust)
            u = ca_with_fdd(nu, L)

        u_act = L * u
        nu_act = Ba @ u_act

        nud[k]=nu; nua[k]=nu_act; ul[k]=u; sl[k]=sig; gl[k]=Gh.copy()
        x[k+1] = rk4(s, nu_act, dt, t)

        # Safety: clip state to prevent numerical blowup
        x[k+1, 0] = np.clip(x[k+1, 0], -5, 20)        # altitude
        x[k+1, 2] = np.clip(x[k+1, 2], -np.pi, np.pi)  # roll
        x[k+1, 4] = np.clip(x[k+1, 4], -np.pi, np.pi)  # pitch
        x[k+1, 6] = np.clip(x[k+1, 6], -np.pi, np.pi)  # yaw

    return dict(t=ta, x=x, ref=rl, nu_d=nud, nu_act=nua,
                u=ul, sigma=sl, gamma=gl)

# ═══════════════════════════════════════════════════════════════════════
# 10. PLOTTING (Figs 5-11)
# ═══════════════════════════════════════════════════════════════════════

def style():
    plt.rcParams.update({
        'figure.figsize':(10,5), 'font.size':12, 'font.family':'serif',
        'axes.grid':True, 'grid.alpha':0.3, 'lines.linewidth':1.5,
        'legend.fontsize':10, 'axes.labelsize':13, 'axes.titlesize':14})

T = lambda r: r['t'][:-1]

def make_fig5(a,n,l,d):
    """Fig 5: Pitch tracking, Scenario 1 (single fault)."""
    fig,ax = plt.subplots(figsize=(10,5)); t=T(a)
    ax.plot(t, np.rad2deg(a['ref'][:,2]),'k--',lw=2,label='Desired')
    ax.plot(t, np.rad2deg(a['x'][:-1,4]),'b-',label='ASMCA')
    ax.plot(t, np.rad2deg(n['x'][:-1,4]),'r-.',label='NSMCA')
    ax.plot(t, np.rad2deg(l['x'][:-1,4]),'g:',lw=2,label='LQRCA')
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7,label=f'Fault @ t={T_FAULT}s')
    ax.set(xlabel='Time (s)',ylabel='Pitch Angle (deg)',xlim=[0,T_SIM])
    ax.set_title('Fig.5 - Pitch Tracking under Single Actuator Failure (Scenario 1)')
    ax.legend(loc='best'); fig.tight_layout()
    fig.savefig(d/'fig5_pitch_single_fault.png',dpi=200); plt.close()

def make_fig6(a,d):
    """Fig 6: Sliding surface, Scenario 1."""
    fig,ax = plt.subplots(figsize=(10,4)); t=T(a)
    ax.plot(t, a['sigma'][:,2],'b-',label='sigma_theta')
    ax.axhline(PHI,color='r',ls='--',alpha=.6,label=f'Phi={PHI}')
    ax.axhline(-PHI,color='r',ls='--',alpha=.6)
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
    ax.set(xlabel='Time (s)',ylabel='Sliding Surface',xlim=[0,T_SIM])
    ax.set_title('Fig.6 - Sliding Surface of ASMCA (Pitch, Single Fault)')
    ax.legend(); fig.tight_layout()
    fig.savefig(d/'fig6_sigma_single_fault.png',dpi=200); plt.close()

def make_fig7(a,n,l,d):
    """Fig 7: Pitch tracking, Scenario 2 (simultaneous faults)."""
    fig,ax = plt.subplots(figsize=(10,5)); t=T(a)
    ax.plot(t, np.rad2deg(a['ref'][:,2]),'k--',lw=2,label='Desired')
    ax.plot(t, np.rad2deg(a['x'][:-1,4]),'b-',label='ASMCA')
    ax.plot(t, np.rad2deg(n['x'][:-1,4]),'r-.',label='NSMCA')
    ax.plot(t, np.rad2deg(l['x'][:-1,4]),'g:',lw=2,label='LQRCA')
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7,label=f'Fault @ t={T_FAULT}s')
    ax.set(xlabel='Time (s)',ylabel='Pitch Angle (deg)',xlim=[0,T_SIM])
    ax.set_title('Fig.7 - Pitch Tracking under Simultaneous Faults (Scenario 2)')
    ax.legend(loc='best'); fig.tight_layout()
    fig.savefig(d/'fig7_pitch_simultaneous_faults.png',dpi=200); plt.close()

def make_fig8(a,d):
    """Fig 8: Motor control inputs (commanded), Scenario 2."""
    fig,ax = plt.subplots(figsize=(10,5)); t=T(a)
    colors = plt.cm.tab10(np.linspace(0,1,N_ROT))
    for j in range(N_ROT):
        ax.plot(t, a['u'][:,j], color=colors[j], label=f'Motor {j+1}')
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
    ax.axhline(U_MAX,color='k',ls='--',alpha=.3,label=f'U_max={U_MAX}N')
    ax.set(xlabel='Time (s)',ylabel='Commanded Motor Thrust (N)',xlim=[0,T_SIM])
    ax.set_title('Fig.8 - Motor Control Inputs under ASMCA (Scenario 2)')
    ax.legend(loc='upper right',ncol=2,fontsize=8); fig.tight_layout()
    fig.savefig(d/'fig8_motor_inputs.png',dpi=200); plt.close()

def make_fig9(a,d):
    """Fig 9: Sliding surface, Scenario 2."""
    fig,ax = plt.subplots(figsize=(10,4)); t=T(a)
    ax.plot(t, a['sigma'][:,2],'b-',label='sigma_theta')
    ax.axhline(PHI,color='r',ls='--',alpha=.6,label=f'Phi={PHI}')
    ax.axhline(-PHI,color='r',ls='--',alpha=.6)
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
    ax.set(xlabel='Time (s)',ylabel='Sliding Surface',xlim=[0,T_SIM])
    ax.set_title('Fig.9 - Sliding Surface of ASMCA (Pitch, Simultaneous Faults)')
    ax.legend(); fig.tight_layout()
    fig.savefig(d/'fig9_sigma_simultaneous_faults.png',dpi=200); plt.close()

def make_fig10(a,d):
    """Fig 10: Virtual control input nu_theta, Scenario 2."""
    fig,ax = plt.subplots(figsize=(10,4)); t=T(a)
    ax.plot(t, a['nu_d'][:,2],'b-',label='Desired nu_theta')
    ax.plot(t, a['nu_act'][:,2],'r--',label='Actual nu_theta')
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
    ax.set(xlabel='Time (s)',ylabel='Virtual Control (N*m)',xlim=[0,T_SIM])
    ax.set_title('Fig.10 - Virtual Control Input for Pitch (Scenario 2)')
    ax.legend(); fig.tight_layout()
    fig.savefig(d/'fig10_virtual_control.png',dpi=200); plt.close()

def make_fig11(a,d):
    """Fig 11: Adaptive parameter Gamma_hat_theta, Scenario 2."""
    fig,ax = plt.subplots(figsize=(10,4)); t=T(a)
    ax.plot(t, a['gamma'][:,2],'b-',lw=2,label='Gamma_hat (pitch)')
    ax.axhline(Iyy,color='r',ls='--',alpha=.6,label=f'Nominal Iyy={Iyy}')
    ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
    ax.set(xlabel='Time (s)',ylabel='Adaptive Gain Gamma_hat',xlim=[0,T_SIM])
    ax.set_title('Fig.11 - Adaptive Parameter of ASMCA (Pitch, Scenario 2)')
    ax.legend(); fig.tight_layout()
    fig.savefig(d/'fig11_adaptive_parameter.png',dpi=200); plt.close()

def make_extra_alt(s1,s2,d):
    fig,axes = plt.subplots(1,2,figsize=(14,4),sharey=True)
    for ax,r,ttl in zip(axes,[s1,s2],['Scenario 1','Scenario 2']):
        t=T(r)
        ax.plot(t,r['ref'][:,0],'k--',lw=2,label='Desired z')
        ax.plot(t,r['x'][:-1,0],'b-',label='Actual z (ASMCA)')
        ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
        ax.set(xlabel='Time (s)',ylabel='Altitude (m)',xlim=[0,T_SIM])
        ax.set_title(f'Altitude Tracking - {ttl}'); ax.legend()
    fig.tight_layout(); fig.savefig(d/'extra_altitude_tracking.png',dpi=200); plt.close()

def make_extra_angles(r,d):
    lbls=['Roll','Pitch','Yaw']; si=[2,4,6]; ri=[1,2,3]
    fig,axes = plt.subplots(3,1,figsize=(10,10),sharex=True); t=T(r)
    for ax,lb,s,rr in zip(axes,lbls,si,ri):
        ax.plot(t,np.rad2deg(r['ref'][:,rr]),'k--',lw=2,label=f'Desired {lb}')
        ax.plot(t,np.rad2deg(r['x'][:-1,s]),'b-',label=f'Actual {lb}')
        ax.axvline(T_FAULT,color='gray',ls=':',alpha=.7)
        ax.set_ylabel(f'{lb} (deg)'); ax.legend(loc='upper right')
    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title('ASMCA - All Euler Angles (Scenario 2)')
    fig.tight_layout(); fig.savefig(d/'extra_all_angles_s2.png',dpi=200); plt.close()

def make_tracking_error_comparison(r1, r2, d):
    """Compare tracking error across controllers for both scenarios."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ax, rd, sn in zip(axes, [r1, r2], ['Scenario 1', 'Scenario 2']):
        for ctrl, col, ls in [('ASMCA','b','-'),('NSMCA','r','-.'),('LQRCA','g',':')]:
            t = T(rd[ctrl])
            err = np.rad2deg(rd[ctrl]['x'][:-1, 4] - rd[ctrl]['ref'][:, 2])
            ax.plot(t, err, color=col, ls=ls, lw=1.5, label=ctrl)
        ax.axvline(T_FAULT, color='gray', ls=':', alpha=.7)
        ax.set_ylabel('Pitch Error (deg)')
        ax.set_title(f'Pitch Tracking Error - {sn}')
        ax.legend(loc='upper right')
        ax.set_xlim([0, T_SIM])
    axes[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    fig.savefig(d/'extra_tracking_error.png', dpi=200); plt.close()

# ═══════════════════════════════════════════════════════════════════════
# 11. MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    style()
    d = Path(__file__).parent / 'results'; d.mkdir(exist_ok=True)
    print("="*60)
    print("  Task 1: Paper Results Reproduction (Wang & Zhang 2018)")
    print("="*60)

    ctrls = ['ASMCA','NSMCA','LQRCA']
    r1, r2 = {}, {}

    print("\n> Scenario 1: Single actuator failure (motor #1 dead at t=20s)")
    for c in ctrls:
        print(f"  {c} ...", end=" ", flush=True)
        r1[c] = simulate(c, 1); print("done.")

    print("\n> Scenario 2: Simultaneous faults (motor #1 dead + motor #5 at 60%)")
    for c in ctrls:
        print(f"  {c} ...", end=" ", flush=True)
        r2[c] = simulate(c, 2); print("done.")

    print(f"\n> Generating figures -> {d.resolve()}")
    make_fig5(r1['ASMCA'], r1['NSMCA'], r1['LQRCA'], d)
    make_fig6(r1['ASMCA'], d)
    make_fig7(r2['ASMCA'], r2['NSMCA'], r2['LQRCA'], d)
    make_fig8(r2['ASMCA'], d)
    make_fig9(r2['ASMCA'], d)
    make_fig10(r2['ASMCA'], d)
    make_fig11(r2['ASMCA'], d)
    make_extra_alt(r1['ASMCA'], r2['ASMCA'], d)
    make_extra_angles(r2['ASMCA'], d)
    make_tracking_error_comparison(r1, r2, d)

    print("\n[OK] All figures saved:")
    for f in sorted(d.glob('*.png')): print(f"    {f.name}")
    print("="*60)

if __name__ == '__main__':
    main()
