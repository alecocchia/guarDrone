#!/usr/bin/env python3
import argparse, numpy as np
import matplotlib.pyplot as plt
from utils_pkg.utils_np import wrap_pi

def myPlot(time, data_list, labels, title, ncols=2, use_tex=True, block=False, fignum=None, task_start=-1.0):
    plt.rcParams.update({"text.usetex": use_tex, "font.family": "serif"})
    n = len(data_list)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3.5 * nrows), squeeze=False, num=fignum)
    if fignum is not None:
        try:
            fig.canvas.manager.set_window_title(f"Figure {fignum}: {title}")
        except AttributeError:
            try:
                fig.canvas.set_window_title(f"Figure {fignum}: {title}")
            except Exception:
                pass
    axes = axes.flatten()
    
    for i in range(n):
        time_plot = time[:len(data_list[i]['sim'])]
        axes[i].plot(time_plot, data_list[i]['sim'], 'b-', label='Actual', linewidth=1.5)
        if 'ref' in data_list[i] and data_list[i]['ref'] is not None:
            ref_data = data_list[i]['ref']
            if np.isscalar(ref_data):
                axes[i].axhline(y=ref_data, color='r', linestyle='--', label='Ref')
            else:
                axes[i].plot(time_plot, ref_data[:len(time_plot)], 'r--', label='Reference', linewidth=1.2)
        if task_start > 0:
            axes[i].axvline(x=task_start, color='k', linestyle='--', linewidth=1.5, label='Mission Start')
        
        axes[i].set_title(labels[i])
        axes[i].grid(True, alpha=0.3)
        axes[i].legend(loc='upper right', fontsize='small')
    
    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])
        
    fig.suptitle(title, fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    if block:
        plt.show()
    return fig

def main():
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=str, default="/tmp/sim_run.mat")
    ap.add_argument("--tex", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--out-dir", type=str, default=".", help="Cartella in cui salvare i plot e i log")
    ap.add_argument("--all", action="store_true", help="Show all figures at once (default is sequential)")
    ap.add_argument("--task-start", type=float, default=None,
                    help="[s] Tempo (relativo) inizio task: disegna linea verticale (sovrascrive il log)")
    ap.add_argument("--formats", type=str, nargs="+", default=["png"],
                    choices=["png", "pdf", "eps"],
                    help="Formati di salvataggio (es: --formats png pdf eps). Default: png")
    args = ap.parse_args()

    try:
        from scipy.io import loadmat
        _raw = loadmat(args.log, squeeze_me=True)
        # loadmat aggiunge chiavi interne '__header__', '__version__', '__globals__' — le filtriamo
        data = {k: v for k, v in _raw.items() if not k.startswith('__')}
    except Exception as e:
        print(f"Errore nel caricamento del log: {e}")
        return

    def indata(key):
        """Controlla se la chiave esiste e il dato non è degenere (array vuoto)."""
        return key in data and np.asarray(data[key]).size > 0

    t = data['t']
    mass = data['mass'] if indata('mass') else 2.0
    g = 9.80665
    block = not args.all and not args.save # Se vogliamo sequenziale, block=True a ogni plot
    task_start = float(np.asarray(data['task_start_time']).flat[0]) if indata('task_start_time') else -1.0
    if args.task_start is not None:   # argomento CLI sovrascrive il valore del log
        task_start = args.task_start
    print(f"[DEBUG] task_start_time presente: {indata('task_start_time')}, value used: {task_start:.3f} s")

    # --- FIGURE 1: Position (ENU) ---
    fig_pos_data = [
        {'sim': data['pos'][:, 0], 'ref': data['pref_pos'][:, 0]},
        {'sim': data['pos'][:, 1], 'ref': data['pref_pos'][:, 1]},
        {'sim': data['pos'][:, 2], 'ref': data['pref_pos'][:, 2]}
    ]
    myPlot(t, fig_pos_data, ["Position X [m]", "Position Y [m]", "Position Z [m]"], 
           "Drone Position vs MPC optimal trajectory", ncols=3, use_tex=args.tex, block=block, fignum=1, task_start=task_start)

    # --- FIGURE 2: Orientation (RPY) ---
    fig_rpy_data = [
        {'sim': data['rpy'][:, 0], 'ref': data['pref_rpy'][:, 0]},
        {'sim': data['rpy'][:, 1], 'ref': data['pref_rpy'][:, 1]}
    ]
    myPlot(t, fig_rpy_data, ["Roll [rad]", "Pitch [rad]"], 
           "Drone Orientation (Roll/Pitch)", ncols=2, use_tex=args.tex, block=block, fignum=2, task_start=task_start)

    # --- FIGURE 3: Velocities ---
    fig_vel_data = [
        {'sim': data['v'][:, 0], 'ref': data['vref'][:, 0]},
        {'sim': data['v'][:, 1], 'ref': data['vref'][:, 1]},
        {'sim': data['v'][:, 2], 'ref': data['vref'][:, 2]},
        {'sim': data['omega'][:, 0], 'ref': data['omegaref'][:, 0]},
        {'sim': data['omega'][:, 1], 'ref': data['omegaref'][:, 1]},
        {'sim': data['omega'][:, 2], 'ref': data['omegaref'][:, 2]}
    ]
    myPlot(t, fig_vel_data, ["Vel X [m/s]", "Vel Y [m/s]", "Vel Z [m/s]", 
                  "Omega X [rad/s]", "Omega Y [rad/s]", "Omega Z [rad/s]"], 
           "Drone Velocities vs MPC Reference", ncols=3, use_tex=args.tex, block=block, fignum=3, task_start=task_start)

    # --- FIGURE 4: Riferimento Cartesiano Camera vs Posizione Effettiva Camera ---
    # Usiamo direttamente p_cam e p_cam_target salvati dal logger.py
    # Questi tengono conto correttamente dell'offset della telecamera 
    fig_cart_ref_data = [
        {'sim': data['p_cam'][:, 0], 'ref': data['p_cam_target'][:, 0]},
        {'sim': data['p_cam'][:, 1], 'ref': data['p_cam_target'][:, 1]},
        {'sim': data['p_cam'][:, 2], 'ref': data['p_cam_target'][:, 2]}
    ]
    myPlot(t, fig_cart_ref_data, ["Cam X [m]", "Cam Y [m]", "Cam Z [m]"], 
           "Camera Cartesian Position vs Derived Target", ncols=3, use_tex=args.tex, block=block, fignum=4, task_start=task_start)

    # --- FIGURE 5: Coordinate Cilindriche (r_cyl, beta, z) vs Riferimento ---
    # Allineamento dell'azimut per evitare che ref e sim si sdoppino di 2*pi nel plot
    beta_sim_unwrapped = np.unwrap(data['beta_cyl'])
    beta_diff_wrapped = (data['online_cyl_ref'][:, 1] - data['beta_cyl'] + np.pi) % (2 * np.pi) - np.pi
    beta_ref_aligned = beta_sim_unwrapped + beta_diff_wrapped

    fig4_data = [
        {'sim': data['r_cyl'],    'ref': data['online_cyl_ref'][:, 0]},
        {'sim': beta_sim_unwrapped, 'ref': beta_ref_aligned},
        {'sim': data['z_cyl'],'ref': data['online_cyl_ref'][:, 2]},
    ]
    myPlot(t, fig4_data,
           ["Distance r_cyl [m]", "Azimuth beta [rad]", "Elevation z [m]"],
           "Cylindrical PoV Tracking (World Frame)",
           ncols=3, use_tex=args.tex, block=block, fignum=5, task_start=task_start)

    # --- FIGURE 6: Yaw Tracking (puntamento verso oggetto) ---
    # yaw_err_cyl è già loggato direttamente dall'MPC (wrap_pi applicato correttamente)
    # yaw_desired = beta_cyl + pi (coerente con la definizione MPC)
    # Wrappiamo entrambi in [-pi, pi] per la visualizzazione usando utils_np
    yaw_actual  = wrap_pi(data['rpy'][:, 2])
    yaw_desired = wrap_pi(data['online_cyl_ref'][:, 1] + np.pi)
    fig5_data = [
        {'sim': yaw_actual,               'ref': yaw_desired},
        {'sim': data['yaw_err_cyl'],      'ref': 0.0},
    ]
    myPlot(t, fig5_data,
           ["Yaw Actual vs Desired [rad]", "Yaw Error [rad]"],
           "Yaw Tracking: Drone Pointing Toward Target",
           ncols=2, use_tex=args.tex, block=block, fignum=6, task_start=task_start)

    # --- FIGURE 6: Errori di Tracking Primari (cilindrici + posizione + orientamento) ---
    err_pos = np.linalg.norm(data['pos'][:, :2] - data['pref_pos'][:, :2], axis=1)
    err_r   = np.abs(data['r_cyl']    - data['online_cyl_ref'][:, 0])
    err_beta  = np.abs(np.arctan2(
        np.sin(data['beta_cyl']  - data['online_cyl_ref'][:, 1]),
        np.cos(data['beta_cyl']  - data['online_cyl_ref'][:, 1])))
    err_z = np.abs(data['z_cyl'] - data['online_cyl_ref'][:, 2])
    err_yaw = np.abs(data['yaw_err_cyl'])
    err_rp = np.linalg.norm(data['q'][:, 1:3], axis=1)  # qx, qy

    fig6_data = [
        {'sim': err_pos,   'ref': 0},
        {'sim': err_r,     'ref': 0},
        {'sim': err_beta,  'ref': 0},
        {'sim': err_z, 'ref': 0},
        {'sim': err_yaw,   'ref': 0},
        {'sim': err_rp,    'ref': 0},
    ]
    myPlot(t, fig6_data,
           ["Norm Pos Error XY [m]", "Distance Error |r_cyl_err| [m]",
            "Azimuth Error |beta_err| [rad]", "Elevation Error |z_err| [m]",
            "Yaw Error |yaw_err| [rad]", "Norm Roll/Pitch Error"],
           "Primary Tracking Errors (Cylindrical)",
           ncols=3, use_tex=args.tex, block=block, fignum=7, task_start=task_start)

    # --- FIGURE 7: Dynamic States Errors & Derivatives ---
    err_vel = np.linalg.norm(data['v'] - data['vref'], axis=1)
    err_omega = np.linalg.norm(data['omega'] - data['omegaref'], axis=1)
    
    fig7_data = [
        {'sim': err_vel, 'ref': 0}, {'sim': err_omega, 'ref': 0}, 
        {'sim': np.linalg.norm(data['acc'], axis=1), 'ref': 0},
        {'sim': np.linalg.norm(data['ang_acc'], axis=1), 'ref': 0}, 
        {'sim': np.linalg.norm(data['jerk'], axis=1), 'ref': 0}, 
        {'sim': np.linalg.norm(data['snap'], axis=1), 'ref': 0}
    ]
    myPlot(t, fig7_data, ["Norm Vel Error [m/s]", "Norm Omega Error [rad/s]", "Norm Acc [m/s^2]",
               "Norm AngAcc [rad/s^2]", "Norm Jerk [m/s^3]", "Norm Snap [m/s^4]"], 
           "Dynamic States Errors and Feedforward Derivatives", ncols=3, use_tex=args.tex, block=block, fignum=8, task_start=task_start)

    # --- FIGURE 8: Wrench ---
    fig8_data = [
        {'sim': data['optimal_wrench'][:, 0], 'ref': data['wrench_target'][:, 0]},
        {'sim': data['optimal_wrench'][:, 1], 'ref': data['wrench_target'][:, 1]},
        {'sim': data['optimal_wrench'][:, 2], 'ref': data['wrench_target'][:, 2]},
        {'sim': data['optimal_wrench'][:, 3], 'ref': data['wrench_target'][:, 3]}
    ]
    myPlot(t, fig8_data, ["Force Z (Thrust) [N]", "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"], 
           f"Control Wrench (Hover Force = {mass*g:.2f}N)", ncols=2, use_tex=args.tex, block=block, fignum=9, task_start=task_start)

    # --- FIGURE 9: Haptic Forces ---
    if indata('haptic_force'):
        fig9_data = [
            {'sim': data['haptic_force'][:, 0], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 1], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig9_data, ["Force X (Zoom) [N]", "Force Y (Pan) [N]", "Force Z (Altitude) [N]"], 
               "Haptic Feedback Forces Transmitted to haptic device", ncols=3, use_tex=args.tex, block=block, fignum=10, task_start=task_start)

    # --- FIGURE 10: Individual Linear and Angular Accelerations ---
    fig10_data = [
        {'sim': data['acc'][:, 0], 'ref': 0.0},
        {'sim': data['acc'][:, 1], 'ref': 0.0},
        {'sim': data['acc'][:, 2], 'ref': 0.0},
        {'sim': data['ang_acc'][:, 0], 'ref': 0.0},
        {'sim': data['ang_acc'][:, 1], 'ref': 0.0},
        {'sim': data['ang_acc'][:, 2], 'ref': 0.0}
    ]
    myPlot(t, fig10_data, 
           ["Linear Acc X [m/s^2]", "Linear Acc Y [m/s^2]", "Linear Acc Z [m/s^2]", 
            "Angular Acc X [rad/s^2]", "Angular Acc Y [rad/s^2]", "Angular Acc Z [rad/s^2]"], 
           "Drone Linear and Angular Accelerations", ncols=3, use_tex=args.tex, block=block, fignum=11, task_start=task_start)

    # --- FIGURE 11: Peg External Forces ---
    if indata('peg_ext_force'):
        fig11_data = [
            {'sim': data['peg_ext_force'][:, 0], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 1], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig11_data, 
               ["Force X (Sensor) [N]", "Force Y (Sensor) [N]", "Force Z (Sensor) [N]"], 
               "Peg External Contact Forces (FT Sensor)", ncols=3, use_tex=args.tex, block=block, fignum=12, task_start=task_start)

    # --- FIGURE 12: Admittance delta_p (spostamento di ammettenza in ENU) ---
    if indata('delta_p'):
        dp = data['delta_p']
        dp_norm = np.linalg.norm(dp, axis=1)
        fig12_data = [
            {'sim': dp[:, 0], 'ref': 0.0},
            {'sim': dp[:, 1], 'ref': 0.0},
            {'sim': dp[:, 2], 'ref': 0.0},
            {'sim': dp_norm,  'ref': 0.0}
        ]
        myPlot(t, fig12_data,
               [r"$\Delta p_x$ [m] (ENU)", r"$\Delta p_y$ [m] (ENU)", r"$\Delta p_z$ [m] (ENU)",
                r"$\|\Delta p\|$ [m]"],
               "Admittance Displacement $\\Delta p$ (ENU frame)",
               ncols=2, use_tex=args.tex, block=block, fignum=13, task_start=task_start)

    # --- FIGURE 12b: delta_p in terna SENSORE ---
    if indata('delta_p_sensor'):
        dps = data['delta_p_sensor']
        dps_norm = np.linalg.norm(dps, axis=1)
        fig12b_data = [
            {'sim': dps[:, 0], 'ref': 0.0},
            {'sim': dps[:, 1], 'ref': 0.0},
            {'sim': dps[:, 2], 'ref': 0.0},
            {'sim': dps_norm,  'ref': 0.0}
        ]
        myPlot(t, fig12b_data,
               [r"$\Delta p_{sx}$ [m] (Sensor X)", r"$\Delta p_{sy}$ [m] (Sensor Y)",
                r"$\Delta p_{sz}$ [m] (Sensor Z)", r"$\|\Delta p_s\|$ [m]"],
               "Admittance Displacement in Sensor Frame",
               ncols=2, use_tex=args.tex, block=block, fignum=131, task_start=task_start)

    # --- FIGURE 13: Confronto ||delta_p|| vs ||F_ext|| ---
    if indata('delta_p') and indata('peg_ext_force'):
        dp_norm  = np.linalg.norm(data['delta_p'], axis=1)
        fext_norm = np.linalg.norm(data['peg_ext_force'], axis=1)
        fig13, ax13 = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                                   num=13)
        try:
            fig13.canvas.manager.set_window_title("Figure 13: Admittance vs Contact Force")
        except Exception:
            pass
        ax13[0].plot(t, fext_norm, 'r-', linewidth=1.5, label=r'$\|F_{ext}\|$ [N]')
        ax13[0].set_ylabel(r'$\|F_{ext}\|$ [N]')
        ax13[0].legend(loc='upper right')
        ax13[0].grid(True, alpha=0.3)
        ax13[0].set_title("Contact Force Norm")
        ax13[1].plot(t, dp_norm, 'b-', linewidth=1.5, label=r'$\|\Delta p\|$ [m]')
        ax13[1].set_xlabel('Time [s]')
        ax13[1].set_ylabel(r'$\|\Delta p\|$ [m]')
        ax13[1].legend(loc='upper right')
        ax13[1].grid(True, alpha=0.3)
        ax13[1].set_title("Admittance Displacement Norm")
        fig13.suptitle("Admittance Effect: Contact Force vs Position Deviation", fontsize=14)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if block:
            plt.show()

    # --- FIGURE 14: Interaction Drone Position ENU (Actual vs Reference) + Yaw ---
    has_peg_actual = indata('peg_actual_pos')
    has_peg_ref    = indata('peg_ref_pos')
    has_peg_yaw    = indata('peg_actual_yaw')
    if has_peg_actual or has_peg_ref:
        peg_act = data['peg_actual_pos'] if has_peg_actual else np.zeros((len(t), 3))
        peg_ref = data['peg_ref_pos']    if has_peg_ref    else None

        fig14_data = [
            {'sim': peg_act[:, 0], 'ref': peg_ref[:, 0] if peg_ref is not None else None},
            {'sim': peg_act[:, 1], 'ref': peg_ref[:, 1] if peg_ref is not None else None},
            {'sim': peg_act[:, 2], 'ref': peg_ref[:, 2] if peg_ref is not None else None},
        ]
        if has_peg_yaw:
            peg_ref_yaw = data['peg_ref_yaw'] if indata('peg_ref_yaw') else None
            fig14_data.append({'sim': data['peg_actual_yaw'], 'ref': peg_ref_yaw})
        myPlot(t, fig14_data,
               ["Peg X [m]", "Peg Y [m]", "Peg Z [m]"] + (["Peg Yaw [rad]"] if has_peg_yaw else []),
               "Interaction Drone Position ENU (Actual vs Planner Reference)",
               ncols=2, use_tex=args.tex, block=block, fignum=14, task_start=task_start)

    # --- FIGURE 15: Interaction Drone Velocities (ENU) + Yaw Rate ---
    has_peg_vel      = indata('peg_actual_vel')
    has_peg_yaw_rate = indata('peg_actual_yaw_rate')
    if has_peg_vel or has_peg_yaw_rate:
        fig15_data, labels15 = [], []
        peg_ref_vel      = data['peg_ref_vel']      if indata('peg_ref_vel')      else None
        peg_ref_yaw_rate = data['peg_ref_yaw_rate'] if indata('peg_ref_yaw_rate') else None
        if has_peg_vel:
            peg_vel = data['peg_actual_vel']
            fig15_data += [
                {'sim': peg_vel[:, 0], 'ref': peg_ref_vel[:, 0] if peg_ref_vel is not None else None},
                {'sim': peg_vel[:, 1], 'ref': peg_ref_vel[:, 1] if peg_ref_vel is not None else None},
                {'sim': peg_vel[:, 2], 'ref': peg_ref_vel[:, 2] if peg_ref_vel is not None else None},
            ]
            labels15 += ["Vel X [m/s]", "Vel Y [m/s]", "Vel Z [m/s]"]
        if has_peg_yaw_rate:
            fig15_data.append({'sim': data['peg_actual_yaw_rate'], 'ref': peg_ref_yaw_rate})
            labels15.append("Yaw Rate [rad/s]")
        myPlot(t, fig15_data, labels15,
               "Interaction Drone Velocities (ENU) and Yaw Rate",
               ncols=2, use_tex=args.tex, block=block, fignum=15, task_start=task_start)

    # --- FIGURE 16: Estimated Wrench (Momentum Based Estimator) ---
    if indata('estimated_wrench'):
        fig16_data = [
            {'sim': data['estimated_wrench'][:, 0], 'ref': 0.0},
            {'sim': data['estimated_wrench'][:, 1], 'ref': 0.0},
            {'sim': data['estimated_wrench'][:, 2], 'ref': 0.0},
            {'sim': data['estimated_wrench'][:, 3], 'ref': 0.0},
            {'sim': data['estimated_wrench'][:, 4], 'ref': 0.0},
            {'sim': data['estimated_wrench'][:, 5], 'ref': 0.0}
        ]
        myPlot(t, fig16_data, 
               ["Force X [N]", "Force Y [N]", "Force Z [N]", 
                "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"], 
               "Estimated Wrench (Momentum-Based Estimator)", ncols=3, use_tex=args.tex, block=block, fignum=16, task_start=task_start)

    # --- FIGURE 17: Violazione geometrica vincolo soft r_min ---
    if indata('r_cyl'):
        r_min = 1.0      # [m] — deve corrispondere a drone_MPC_settings.py
        Z_pen = 1e3      # L2 penalty — aggiornare se modificato in configure_mpc
        z_pen = 1e2      # L1 penalty — aggiornare se modificato in configure_mpc
        s_geom  = np.maximum(0.0, r_min - data['r_cyl'])   # violazione geometrica reale
        J_slack = 0.5 * Z_pen * s_geom**2 + z_pen * s_geom

        fig17, axes17 = plt.subplots(1, 2, figsize=(12, 4), num=17)
        try:
            fig17.canvas.manager.set_window_title("Figure 17: Violazione Vincolo Soft r_min")
        except Exception:
            pass

        # --- Subplot sx: violazione s(t) ---
        ax_s = axes17[0]
        ax_s.plot(t, s_geom, 'r-', linewidth=1.5, label=r'$s = \max(0,\,r_{min} - r_{cyl})$')
        if task_start > 0:
            ax_s.axvline(x=task_start, color='k', linestyle='--', linewidth=1.5, label='Mission Start')
        ax_s.set_title(r'Violazione geometrica $s(t)$ [$r_{min}$=' + f'{r_min} m]')
        ax_s.set_xlabel('Time [s]')
        ax_s.set_ylabel('s [m]')
        ax_s.legend(fontsize='small')
        ax_s.grid(True, alpha=0.3)

        # --- Subplot dx: costo slack J(t) ---
        ax_j = axes17[1]
        ax_j.plot(t, J_slack, 'm-', linewidth=1.5, label=r'$J_{slack} = \frac{1}{2} Z s^2 + z_p s$')
        if task_start > 0:
            ax_j.axvline(x=task_start, color='k', linestyle='--', linewidth=1.5, label='Mission Start')
        ax_j.set_title(f'Costo slack $J_{{slack}}(t)$  [Z={Z_pen:.0e}, $z_p$={z_pen:.0e}]')
        ax_j.set_xlabel('Time [s]')
        ax_j.set_ylabel('J [adim.]')
        ax_j.legend(fontsize='small')
        ax_j.grid(True, alpha=0.3)

        fig17.suptitle('Soft Constraint — Distanza Minima da Oggetto', fontsize=14)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if block:
            plt.show()

    # --- FIGURE 18: Integral Action ---
    if indata('integral_action'):
        fig18_data = [
            {'sim': data['integral_action'][:, 0], 'ref': 0.0},
            {'sim': data['integral_action'][:, 1], 'ref': 0.0},
            {'sim': data['integral_action'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig18_data, 
               ["Integral e_x [m*s]", "Integral e_y [m*s]", "Integral e_z [m*s]"], 
               "Cartesian Integral Action (Anti-windup clipped)", ncols=3, use_tex=args.tex, block=block, fignum=18, task_start=task_start)

    if args.save:
        if args.out_dir != "." and not os.path.exists(args.out_dir):
            os.makedirs(args.out_dir)
        # Opzioni di salvataggio per formato vettoriale (pdf/eps): dpi alto, niente trasparenza
        fmt_opts = {
            "png": {"dpi": 150},
            "pdf": {"dpi": 300, "bbox_inches": "tight"},
            "eps": {"dpi": 300, "bbox_inches": "tight", "format": "eps"},
        }
        fig_nums = plt.get_fignums()
        for i in fig_nums:
            for fmt in args.formats:
                out_path = os.path.join(args.out_dir, f"plot_fig_{i}.{fmt}")
                plt.figure(i).savefig(out_path, **fmt_opts[fmt])

        # --- PDF multi-pagina: tutti i grafici in un unico file scrollabile ---
        from matplotlib.backends.backend_pdf import PdfPages
        multipage_path = os.path.join(args.out_dir, "all_figures.pdf")
        with PdfPages(multipage_path) as pdf:
            for i in fig_nums:
                pdf.savefig(plt.figure(i), bbox_inches="tight")
        print(f"Grafici salvati in: {os.path.abspath(args.out_dir)} | Formati: {args.formats}")
        print(f"PDF scrollabile multi-pagina generato in: {multipage_path}")
    elif args.all:
        plt.show()

if __name__ == "__main__":
    main()
