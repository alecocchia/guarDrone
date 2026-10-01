#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_MPC.py — Test standalone a ciclo chiuso dell'MPC in modalità CONTROLLER (Thrust e Torques)
senza Gazebo, PX4 o nodi ROS attivi.

Funzionamento:
- Inizializza il modello Acados e il solver con le stesse impostazioni di MPC_planner_node.py.
- Esegue una simulazione Receding Horizon a ciclo chiuso:
    MPC(xk) -> u0 = [Fz, tau_x, tau_y, tau_z] -> Integratore RK4 (dinamica quadricottero) -> x_{k+1}
- Registra stati, input e tempi di risoluzione.
- Genera grafici dettagliati e salva l'output su file 'mpc_test_results.png'.
"""

import os
import sys
import time
import numpy as np

# Aggiunta dei percorsi di ricerca per guardrone_pkg e utils_pkg
current_dir = os.path.dirname(os.path.abspath(__file__))
ws_src_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
search_paths = [
    current_dir,
    os.path.join(ws_src_dir, 'guardrone_pkg'),
    os.path.join(ws_src_dir, 'utils_pkg'),
    '/root/my_ros2_ws/src/guardrone_pkg',
    '/root/my_ros2_ws/src/utils_pkg',
]
for p in search_paths:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import casadi as ca
from scipy.spatial.transform import Rotation
try:
    import mpl_toolkits
    _local_mpl = '/usr/local/lib/python3.10/dist-packages/mpl_toolkits'
    if os.path.exists(_local_mpl) and _local_mpl not in mpl_toolkits.__path__:
        mpl_toolkits.__path__.insert(0, _local_mpl)
    from mpl_toolkits.mplot3d import Axes3D
except Exception:
    pass
import matplotlib
# Se DISPLAY non è definito, imposta backend non interattivo 'Agg' per salvare i grafici
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Import dei moduli di progetto
from guardrone_pkg.drone_MPC_settings import (
    setup_model, setup_initial_conditions, configure_mpc, set_initial_state, build_yref_online
)
from utils_pkg.common import quat_to_RPY, g0, wrap_pi, traj_plot3D_animated_with_orientation
from utils_pkg.utils_np import min_angle, cylindrical_to_cartesian
from utils_pkg.planner import generate_trapezoidal_trajectory


def run_standalone_test(sim_time=30.0, plot_save_path=None, save_gif=False):
    if plot_save_path is None:
        plot_save_path = os.path.join(current_dir, 'mpc_test_results.png')
    print("=" * 60)
    print(" Avvio test standalone MPC (Modalità Controller: Fz, Tau)")
    print("=" * 60)

    # 1. Parametri fisici del drone (identici a MPC_planner_node.py)
    mass = 2.064
    ixx = 0.0216
    iyy = 0.0216
    izz = 0.040
    arm_l_x = 0.174
    arm_l_y = 0.174
    moment_const = 0.016
    f_max = 34.0

    u_f_max = f_max
    u_tau_x = arm_l_y * u_f_max / 2.0
    u_tau_y = arm_l_x * u_f_max / 2.0
    u_tau_z = moment_const * u_f_max
    u_hover = np.array([mass * g0, 0.0, 0.0, 0.0])

    # 2. Parametri tempo e orizzonte
    Hz = 100.0
    ts = 1.0 / Hz          # 10 ms
    N_horiz = 15         # Orizzonte di predizione (150 ms)
    Tp = N_horiz * ts
    n_steps = int(sim_time / ts)

    print(f"[INFO] ts: {ts*1e3:.1f} ms | N: {N_horiz} | Orizzonte: {Tp*1e3:.1f} ms | Durata sim: {sim_time} s ({n_steps} passi)")

    # 3. Inizializzazione modello
    model, model_rpy = setup_model(mass, ixx, iyy, izz)

    # 3a. Posizioni oggetto (target) e riferimento cilindrico
    # Definiti prima delle condizioni iniziali del drone, così lo start è coerente col riferimento
    p_A = np.array([0.0, 0.0, 1.0])    # posizione iniziale oggetto [m]
    p_B = np.array([2.0, 1.0, 1.0])    # posizione finale oggetto [m]

    # Riferimento cilindrico: [r_ref (m), beta_ref (rad), z_rel_ref (m)]
    cyl_ref = np.array([2.0, np.pi, 0.0])
    yaw_ref = 0.0  # [rad] yaw aggiuntivo rispetto alla direzione oggetto

    # Condizioni iniziali del drone: posizione = riferimento cilindrico rispetto a p_A
    # Yaw iniziale = direzione del drone verso l'oggetto (beta + pi) + yaw_ref
    p0_drone = cylindrical_to_cartesian(cyl_ref, p_origin=p_A)+np.array([0.3,0.3,-0.2])  # posizione desiderata
    yaw0 = wrap_pi(cyl_ref[1] + np.pi + yaw_ref)               # punta verso l'oggetto
    start_x, start_y, start_z = p0_drone[0], p0_drone[1], p0_drone[2]
    start_roll, start_pitch, start_yaw = 0.0, 0.0, float(yaw0)
    print(f"[INFO] Start drone: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f}) m | yaw0={np.degrees(start_yaw):.1f} deg")
    x0, x0_rpy = setup_initial_conditions(start_x, start_y, start_z, start_roll, start_pitch, start_yaw)

    # 3b. Generazione traiettoria trapezoidale per il target da punto A a punto B
    v_target_max = 0.1    # [m/s] velocità massima target
    a_target_max = 0.2   # [m/s^2] accelerazione massima target
    delay_start_target = 1  # [s] attesa iniziale prima della partenza del target

    _, p_trap, _ = generate_trapezoidal_trajectory(
        np.hstack([p_A, [0, 0, 0]]),
        np.hstack([p_B, [0, 0, 0]]),
        dt=ts, v_max=v_target_max, a_max=a_target_max
    )

    p_obj_hist = np.tile(p_A, (n_steps, 1))
    idx_start_mov = int(delay_start_target / ts)
    n_trap = len(p_trap)
    if idx_start_mov + n_trap <= n_steps:
        p_obj_hist[idx_start_mov:idx_start_mov + n_trap, :] = p_trap
        p_obj_hist[idx_start_mov + n_trap:, :] = p_B
    else:
        p_obj_hist[idx_start_mov:, :] = p_trap[:n_steps - idx_start_mov]

    v_obj_hist = np.gradient(p_obj_hist, ts, axis=0)
    rpy_obj_hist = np.zeros((n_steps, 3))
    p_obj = p_A.copy()

    # 4. Pesi della funzione di costo — formulazione cartesiana
    # Range cartesiani: errore massimo atteso su x, y, z [m] e yaw [rad]
    X_CART = 0.2
    Y_CART = 0.2
    Z_CART = 0.2
    Y_CYL  = np.pi / 3   
    
    V = np.array([0.3, 0.3, 0.3])
    ANG_DOT = np.array([0.15, 0.15, 0.3])
    ACC = np.array([0.4, 0.4, 0.4])
    ACC_ANG = np.array([0.4, 0.4, 0.8])
    
    PesoVis = 10.0
    PesoX   = PesoVis
    PesoY   = PesoVis
    PesoZ   = PesoVis
    PesoYaw = PesoVis
    PesoVel    = PesoVis / 10
    PesoAngVel = PesoVis / 50
    PesoAcc    = PesoVis / 20
    PesoAngAcc = PesoVis / 100
    PesoForce  = PesoVis / 100
    PesoTorque = PesoForce 
    # Q cartesiana: [ex, ey, ez, yaw_err]
    Q_cart = np.diag([
        PesoX   / X_CART**2,
        PesoY   / Y_CART**2,
        PesoZ   / Z_CART**2,
        PesoYaw / Y_CYL**2
    ])
    Q_vel     = np.diag([PesoVel] * 3) / np.array(V)**2
    Q_ang_dot = np.diag([PesoAngVel] * 3) / np.array(ANG_DOT)**2
    Q_acc     = np.diag([PesoAcc] * 3) / np.array(ACC)**2
    Q_acc_ang = np.diag([PesoAngAcc] * 3) / np.array(ACC_ANG)**2
    R_f   = np.diag([PesoForce / u_f_max**2])
    R_tau = np.diag([
        PesoTorque / u_tau_x**2,
        PesoTorque / u_tau_y**2,
        PesoTorque / u_tau_z**2
    ])
    R   = ca.diagcat(R_f, R_tau)
    Q   = ca.diagcat(Q_cart, Q_vel, Q_ang_dot, Q_acc, Q_acc_ang)
    Q_e = ca.diagcat(10 * Q_cart, 10 * Q_vel, 10 * Q_ang_dot, 1 * Q_acc, 1 * Q_acc_ang)

        
    u_min = np.array([0.0, -u_tau_x, -u_tau_y, -u_tau_z])
    u_max = np.array([u_f_max, u_tau_x, u_tau_y, u_tau_z])

    W   = ca.diagcat(Q, R).full()
    W_e = Q_e.full()

    # Target cartesiano iniziale (conversione cyl→cart avviene qui, non nel solver)
    p_target_init = p_A + cylindrical_to_cartesian(cyl_ref)

    print("[INFO] Configurazione del solver Acados...")
    ocp_solver, N_horiz, nx, nu, y_idx, ny, ny_e = configure_mpc(
        model=model, x0=x0,
        p_target=p_target_init, Tf=Tp, ts=ts,
        W=W, W_e=W_e, u_min=u_min, u_max=u_max,
        yaw_des=yaw0
    )
    print("[INFO] Solver Acados configurato con successo.")

    # Diagnostica condizionamento matrice pesi W
    cond = np.linalg.cond(W)
    w_diag = np.diag(W)
    print(f"[INFO] Condition number di W: {cond:.2e}")
    print(f"[INFO] W min={w_diag.min():.2e} (idx {w_diag.argmin()}), max={w_diag.max():.2e} (idx {w_diag.argmax()})")

    # 5. Costruzione integratore RK4 della dinamica del drone
    # f_expl_expr modella: p_dot = v, v_dot = 1/m*(R*Fz + F_ext) - g, q_dot, w_dot
    f_dyn = ca.Function('f_dyn', [model.x, model.u, model.p], [model.f_expl_expr])

    def rk4_step(x, u, p, dt):
        k1 = f_dyn(x, u, p).full().flatten()
        k2 = f_dyn(x + 0.5 * dt * k1, u, p).full().flatten()
        k3 = f_dyn(x + 0.5 * dt * k2, u, p).full().flatten()
        k4 = f_dyn(x + dt * k3, u, p).full().flatten()
        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        # Normalizzazione quaternione numerico
        q_norm = np.linalg.norm(x_next[6:10])
        if q_norm > 1e-6:
            x_next[6:10] /= q_norm
        return x_next

    # 6. Preparazione loop di simulazione a ciclo chiuso
    xk = x0.copy()
    # Parametri (10): [p_target(3), yaw_des(1), F_ext(3), Tau_ext(3)]
    params = np.zeros(10)
    params[3]    = yaw0
    params[4:7]  = np.zeros(3)   # F_ext
    params[7:10] = np.zeros(3)   # Tau_ext

    yref_val = build_yref_online(y_idx, np.zeros(3), u_ref=u_hover)
    yref_e = yref_val[:ny_e]

    for i in range(N_horiz):
        ocp_solver.set(i, "yref", yref_val)
    ocp_solver.set(N_horiz, "yref", yref_e)

    # Buffer per memorizzare i risultati
    t_hist = np.zeros(n_steps)
    x_hist = np.zeros((n_steps, nx))
    u_hist = np.zeros((n_steps, nu))
    rpy_hist = np.zeros((n_steps, 3))
    solve_times = np.zeros(n_steps)
    cyl_errors = np.zeros((n_steps, 4))  # r_err, beta_err, z_err, yaw_err
    acc_lin_hist = np.zeros((n_steps, 3))  # [ax, ay, az] accelerazioni lineari
    acc_ang_hist = np.zeros((n_steps, 3))  # [alpha_x, alpha_y, alpha_z] accelerazioni angolari

    print("[INFO] Inizio ciclo chiuso di simulazione...")
    t_start_all = time.perf_counter()
    
    for i in range(N_horiz):
        ocp_solver.set(i, "u", u_hover)
        ocp_solver.set(i, "x", x0)
    ocp_solver.set(N_horiz, "x", x0)

    for k in range(n_steps):
        t_now = k * ts
        t_hist[k] = t_now
        x_hist[k, :] = xk
        p_obj_now = p_obj_hist[k]

        # Calcolo angoli Roll, Pitch, Yaw correnti
        q_curr = xk[6:10]
        # quat_to_RPY si aspetta [qw, qx, qy, qz]
        rpy_curr = quat_to_RPY(q_curr)
        rpy_hist[k, :] = [float(rpy_curr[0]), float(rpy_curr[1]), float(rpy_curr[2])]

        # Calcolo errori cilindrici correnti per il log
        p_rel = xk[0:3] - p_obj_now
        r_cyl_now = np.sqrt(p_rel[0]**2 + p_rel[1]**2)
        beta_raw = np.arctan2(p_rel[1], p_rel[0])
        r_err = cyl_ref[0] - r_cyl_now
        beta_err = wrap_pi(beta_raw - cyl_ref[1])
        z_err = cyl_ref[2] - p_rel[2]
        yaw_des = wrap_pi(beta_raw + np.pi + yaw_ref)
        yaw_err = wrap_pi(rpy_hist[k, 2] - yaw_des)
        cyl_errors[k, :] = [r_err, beta_err, z_err, yaw_err]

        # Imposta lo stato iniziale per questo passo nel solver
        set_initial_state(ocp_solver, xk)

        # Aggiorna p_target e yaw_des sull'orizzonte (precalcolati fuori dal solver)
        for i in range(N_horiz + 1):
            k_future = min(k + i, n_steps - 1)
            p_target_k = p_obj_hist[k_future] + cylindrical_to_cartesian(cyl_ref)
            # Direzione verso l'oggetto (norma = r_cyl > 0, mai singolarità)
            p_diff_k = p_obj_hist[k_future] - p_target_k
            yaw_des_k = wrap_pi(np.arctan2(p_diff_k[1], p_diff_k[0]) + yaw_ref)
            params[0:3] = p_target_k
            params[3]    = yaw_des_k
            ocp_solver.set(i, "p", params)

        # Risoluzione MPC
        t0_solve = time.perf_counter()
        status = ocp_solver.solve()
        dt_solve = time.perf_counter() - t0_solve
        solve_times[k] = dt_solve * 1e3  # ms

        if status != 0:
            print(f"[WARN] Passo {k} (t={t_now:.2f}s): Acados solver status {status}")

        # Estrazione comando u0 da applicare
        u0 = ocp_solver.get(0, "u")
        u_hist[k, :] = u0

        # Calcolo accelerazioni correnti tramite la dinamica del drone (xdot = f_dyn)
        xdot = f_dyn(xk, u0, params).full().flatten()
        acc_lin_hist[k, :] = xdot[3:6]
        acc_ang_hist[k, :] = xdot[10:13]

        # Avanzamento della fisica tramite integratore RK4 (Ciclo Chiuso)
        xk = rk4_step(xk, u0, params, ts)

    total_time = time.perf_counter() - t_start_all
    print(f"[INFO] Simulazione completata in {total_time:.2f} s ({sim_time/total_time:.1f}x real-time)")
    print(f"[INFO] Tempo medio risoluzione Acados: {np.mean(solve_times):.2f} ms (max: {np.max(solve_times):.2f} ms)")

    # 7. Generazione grafici
    print(f"[INFO] Generazione grafici su '{plot_save_path}'...")
    fig, axs = plt.subplots(4, 2, figsize=(14, 10))
    fig.suptitle("Test MPC Standalone — Risultati Controller a Ciclo Chiuso (No Gazebo)", fontsize=14)

    # Posizione cartesiana desiderata del drone: p_obj + offset cilindrico (r*cos(beta), r*sin(beta), z_rel)
    x_des_cart = p_obj_hist[:, 0] + cyl_ref[0] * np.cos(cyl_ref[1])
    y_des_cart = p_obj_hist[:, 1] + cyl_ref[0] * np.sin(cyl_ref[1])
    z_des_cart = p_obj_hist[:, 2] + cyl_ref[2]

    # Plot 1: Posizione cartesiana X, Y, Z (drone vs desiderato vs oggetto)
    axs[0, 0].plot(t_hist, x_hist[:, 0], label='x drone', color='tab:blue')
    axs[0, 0].plot(t_hist, x_hist[:, 1], label='y drone', color='tab:orange')
    axs[0, 0].plot(t_hist, x_hist[:, 2], label='z drone', color='tab:green')
    axs[0, 0].plot(t_hist, x_des_cart, color='tab:blue', linestyle='--', alpha=0.8, label='x des (drone)')
    axs[0, 0].plot(t_hist, y_des_cart, color='tab:orange', linestyle='--', alpha=0.8, label='y des (drone)')
    axs[0, 0].plot(t_hist, z_des_cart, color='tab:green', linestyle='--', alpha=0.8, label='z des (drone)')
    axs[0, 0].plot(t_hist, p_obj_hist[:, 0], color='tab:blue', linestyle=':', alpha=0.35, label='x oggetto')
    axs[0, 0].plot(t_hist, p_obj_hist[:, 1], color='tab:orange', linestyle=':', alpha=0.35, label='y oggetto')
    axs[0, 0].set_ylabel('Posizione [m]')
    axs[0, 0].legend(loc='upper right', fontsize=7)
    axs[0, 0].grid(True)
    axs[0, 0].set_title('Posizioni nel mondo (Drone vs Desiderato vs Oggetto)')

    # Plot 2: Traiettoria nel piano orizzontale (X vs Y)
    axs[0, 1].plot(x_hist[:, 0], x_hist[:, 1], label='Traiettoria drone', color='purple')
    axs[0, 1].plot(p_obj_hist[:, 0], p_obj_hist[:, 1], label='Traiettoria oggetto', color='red', linestyle=':')
    axs[0, 1].plot(x_des_cart, y_des_cart, color='blue', linestyle='--', alpha=0.7, label='Pos. desiderata drone')
    axs[0, 1].scatter([x_hist[0, 0]], [x_hist[0, 1]], color='green', marker='o', s=80, label='Inizio drone')
    axs[0, 1].scatter([p_A[0]], [p_A[1]], color='red', marker='X', s=90, label='Oggetto A (Start)')
    axs[0, 1].scatter([p_B[0]], [p_B[1]], color='darkred', marker='X', s=90, label='Oggetto B (End)')
    axs[0, 1].scatter([x_des_cart[-1]], [y_des_cart[-1]], color='blue', marker='*', s=120, label='Pos. desiderata finale')
    axs[0, 1].set_xlabel('X [m]')
    axs[0, 1].set_ylabel('Y [m]')
    axs[0, 1].legend(loc='best', fontsize=7.5)
    axs[0, 1].grid(True)
    axs[0, 1].axis('equal')
    axs[0, 1].set_title('Traiettoria XY')

    # Plot 3: Errori Cilindrici (raggio e quota)
    axs[1, 0].plot(t_hist, cyl_errors[:, 0], label=r'Errore raggio $r_{err}$ [m]', color='tab:cyan')
    axs[1, 0].plot(t_hist, cyl_errors[:, 2], label=r'Errore quota $z_{err}$ [m]', color='tab:olive')
    axs[1, 0].axhline(0.0, color='black', linestyle=':', alpha=0.7)
    axs[1, 0].set_ylabel('Errore [m]')
    axs[1, 0].legend(loc='upper right', fontsize=8)
    axs[1, 0].grid(True)
    axs[1, 0].set_title('Errori di posizione relativa (Distanza e Quota)')

    # Plot 4: Errori Angolari (Azimut beta e Yaw)
    axs[1, 1].plot(t_hist, cyl_errors[:, 1], label=r'Errore azimut $\beta_{err}$ [rad]', color='tab:pink')
    axs[1, 1].plot(t_hist, cyl_errors[:, 3], label=r'Errore yaw [rad]', color='tab:brown')
    axs[1, 1].axhline(0.0, color='black', linestyle=':', alpha=0.7)
    axs[1, 1].set_ylabel('Errore [rad]')
    axs[1, 1].legend(loc='upper right', fontsize=8)
    axs[1, 1].grid(True)
    axs[1, 1].set_title('Errori angolari (Azimut e Yaw)')

    # Plot 5: Comando di spinta Fz
    axs[2, 0].plot(t_hist, u_hist[:, 0], label='Fz applicato', color='tab:red')
    axs[2, 0].axhline(u_hover[0], color='black', linestyle='--', alpha=0.7, label=f'Hover ({u_hover[0]:.1f} N)')
    axs[2, 0].axhline(u_f_max, color='red', linestyle=':', alpha=0.5, label='Fz max')
    axs[2, 0].axhline(0.0, color='red', linestyle=':', alpha=0.5)
    axs[2, 0].set_ylabel('Spinta [N]')
    axs[2, 0].legend(loc='upper right', fontsize=8)
    axs[2, 0].grid(True)
    axs[2, 0].set_title('Controllo: Spinta Totale (Thrust)')

    # Plot 6: Comandi di coppia (Torques Tau_x, Tau_y, Tau_z)
    axs[2, 1].plot(t_hist, u_hist[:, 1], label=r'$\tau_x$ (Roll)', color='tab:blue')
    axs[2, 1].plot(t_hist, u_hist[:, 2], label=r'$\tau_y$ (Pitch)', color='tab:orange')
    axs[2, 1].plot(t_hist, u_hist[:, 3], label=r'$\tau_z$ (Yaw)', color='tab:green')
    axs[2, 1].set_ylabel('Coppia [Nm]')
    axs[2, 1].legend(loc='upper right', fontsize=8)
    axs[2, 1].grid(True)
    axs[2, 1].set_title('Controllo: Coppie (Torques)')

    # Plot 7: Assetto Roll e Pitch del drone
    axs[3, 0].plot(t_hist, rpy_hist[:, 0], label='Roll [rad]', color='tab:blue')
    axs[3, 0].plot(t_hist, rpy_hist[:, 1], label='Pitch [rad]', color='tab:orange')
    axs[3, 0].set_ylabel('Angolo [rad]')
    axs[3, 0].set_xlabel('Tempo [s]')
    axs[3, 0].legend(loc='upper right', fontsize=8)
    axs[3, 0].grid(True)
    axs[3, 0].set_title('Assetto del drone (Roll e Pitch)')

    # Plot 8: Tempi di risoluzione del solver
    axs[3, 1].plot(t_hist, solve_times, label='Tempo Acados [ms]', color='teal')
    axs[3, 1].axhline(ts * 1e3, color='red', linestyle='--', label=f'Budget Ts ({ts*1e3:.0f} ms)')
    axs[3, 1].set_ylabel('Tempo [ms]')
    axs[3, 1].set_xlabel('Tempo [s]')
    axs[3, 1].legend(loc='upper right', fontsize=8)
    axs[3, 1].grid(True)
    axs[3, 1].set_title('Tempo di risoluzione per ciclo')

    plt.tight_layout()
    plt.savefig(plot_save_path, dpi=150)
    print(f"[SUCCESS] Grafici salvati con successo in: {os.path.abspath(plot_save_path)}")

    # 8. Generazione seconda figura: Cinematica e Dinamica (Velocità e Accelerazioni)
    plot_kin_save_path = plot_save_path.replace('.png', '_kinematics.png')
    print(f"[INFO] Generazione grafici cinematica su '{plot_kin_save_path}'...")
    fig_kin, axs_kin = plt.subplots(2, 2, figsize=(14, 8))
    fig_kin.suptitle("Test MPC Standalone — Velocità e Accelerazioni (Lineari e Angolari)", fontsize=14)

    # Plot 1: Velocità lineari Vx, Vy, Vz
    axs_kin[0, 0].plot(t_hist, x_hist[:, 3], label='v_x', color='tab:blue')
    axs_kin[0, 0].plot(t_hist, x_hist[:, 4], label='v_y', color='tab:orange')
    axs_kin[0, 0].plot(t_hist, x_hist[:, 5], label='v_z', color='tab:green')
    axs_kin[0, 0].axhline(V[0], color='tab:blue', linestyle='--', alpha=0.5, label='lim v_xy')
    axs_kin[0, 0].axhline(-V[0], color='tab:blue', linestyle='--', alpha=0.5)
    axs_kin[0, 0].axhline(V[2], color='tab:green', linestyle='--', alpha=0.5, label='lim v_z')
    axs_kin[0, 0].axhline(-V[2], color='tab:green', linestyle='--', alpha=0.5)
    axs_kin[0, 0].set_ylabel('Velocità [m/s]')
    axs_kin[0, 0].set_xlabel('Tempo [s]')
    axs_kin[0, 0].legend(loc='upper right', fontsize=8)
    axs_kin[0, 0].grid(True)
    axs_kin[0, 0].set_title('Velocità Lineari nel Mondo')

    # Plot 2: Velocità angolari wx, wy, wz
    axs_kin[0, 1].plot(t_hist, x_hist[:, 10], label=r'$\omega_x$', color='tab:blue')
    axs_kin[0, 1].plot(t_hist, x_hist[:, 11], label=r'$\omega_y$', color='tab:orange')
    axs_kin[0, 1].plot(t_hist, x_hist[:, 12], label=r'$\omega_z$', color='tab:green')
    axs_kin[0, 1].axhline(ANG_DOT[0], color='tab:blue', linestyle='--', alpha=0.5, label=r'lim $\omega_{xy}$')
    axs_kin[0, 1].axhline(ANG_DOT[0], color='tab:blue', linestyle='--', alpha=0.5)
    axs_kin[0, 1].axhline(ANG_DOT[2], color='tab:green', linestyle='--', alpha=0.5, label=r'lim $\omega_z$')
    axs_kin[0, 1].axhline(ANG_DOT[2], color='tab:green', linestyle='--', alpha=0.5)
    axs_kin[0, 1].set_ylabel('Velocità angolare [rad/s]')
    axs_kin[0, 1].set_xlabel('Tempo [s]')
    axs_kin[0, 1].legend(loc='upper right', fontsize=8)
    axs_kin[0, 1].grid(True)
    axs_kin[0, 1].set_title('Velocità Angolari Body')

    # Plot 3: Accelerazioni lineari ax, ay, az
    axs_kin[1, 0].plot(t_hist, acc_lin_hist[:, 0], label='a_x', color='tab:blue')
    axs_kin[1, 0].plot(t_hist, acc_lin_hist[:, 1], label='a_y', color='tab:orange')
    axs_kin[1, 0].plot(t_hist, acc_lin_hist[:, 2], label='a_z', color='tab:green')
    axs_kin[1, 0].axhline(ACC[0], color='tab:blue', linestyle='--', alpha=0.5, label='lim a_xy')
    axs_kin[1, 0].axhline(-ACC[0], color='tab:blue', linestyle='--', alpha=0.5)
    axs_kin[1, 0].axhline(ACC[2], color='tab:green', linestyle='--', alpha=0.5, label='lim a_z')
    axs_kin[1, 0].axhline(-ACC[2], color='tab:green', linestyle='--', alpha=0.5)
    axs_kin[1, 0].set_ylabel(r'Accelerazione [$m/s^2$]')
    axs_kin[1, 0].set_xlabel('Tempo [s]')
    axs_kin[1, 0].legend(loc='upper right', fontsize=8)
    axs_kin[1, 0].grid(True)
    axs_kin[1, 0].set_title('Accelerazioni Lineari nel Mondo')

    # Plot 4: Accelerazioni angolari alpha_x, alpha_y, alpha_z
    axs_kin[1, 1].plot(t_hist, acc_ang_hist[:, 0], label=r'$\dot{\omega}_x$', color='tab:blue')
    axs_kin[1, 1].plot(t_hist, acc_ang_hist[:, 1], label=r'$\dot{\omega}_y$', color='tab:orange')
    axs_kin[1, 1].plot(t_hist, acc_ang_hist[:, 2], label=r'$\dot{\omega}_z$', color='tab:green')
    axs_kin[1, 1].axhline(ACC_ANG[0], color='tab:blue', linestyle='--', alpha=0.5, label=r'lim $\dot{\omega}_{xy}$')
    axs_kin[1, 1].axhline(ACC_ANG[0], color='tab:blue', linestyle='--', alpha=0.5)
    axs_kin[1, 1].axhline(ACC_ANG[2], color='tab:green', linestyle='--', alpha=0.5, label=r'lim $\dot{\omega}_z$')
    axs_kin[1, 1].axhline(ACC_ANG[2], color='tab:green', linestyle='--', alpha=0.5)
    axs_kin[1, 1].set_ylabel(r'Accelerazione angolare [$rad/s^2$]')
    axs_kin[1, 1].set_xlabel('Tempo [s]')
    axs_kin[1, 1].legend(loc='upper right', fontsize=8)
    axs_kin[1, 1].grid(True)
    axs_kin[1, 1].set_title('Accelerazioni Angolari Body')

    fig_kin.tight_layout()
    fig_kin.savefig(plot_kin_save_path, dpi=150)
    print(f"[SUCCESS] Grafici cinematica salvati con successo in: {os.path.abspath(plot_kin_save_path)}")

    # 9. Animazione 3D traiettorie e orientamenti (Drone vs Oggetto)
    if save_gif:
        plot_anim_save_path = plot_save_path.replace('.png', '_animation.gif')
        print(f"[INFO] Creazione animazione 3D su '{plot_anim_save_path}'...")
        # 1 frame ogni 0.1s (10 Hz)
        anim_step = max(1, int(0.1 / ts))
        ani = traj_plot3D_animated_with_orientation(
            t_hist, x_hist[:, 0:3], rpy_hist, p_obj_hist, rpy_obj_hist,
            interval=30, step=anim_step, axis_length=0.2,
            save_path=plot_anim_save_path, show=False
        )

    # Se c'è un server X attivo e non siamo headless, mostriamo le finestre
    if os.environ.get('DISPLAY'):
        try:
            plt.show()
        except Exception:
            pass


if __name__ == '__main__':
    run_standalone_test()
