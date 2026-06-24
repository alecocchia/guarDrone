#!/usr/bin/env python3
import argparse, numpy as np
import matplotlib.pyplot as plt

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
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=str, default="/tmp/sim_run.npz")
    ap.add_argument("--tex", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--all", action="store_true", help="Show all figures at once (default is sequential)")
    ap.add_argument("--task-start", type=float, default=None,
                    help="[s] Tempo (relativo) inizio task: disegna linea verticale (sovrascrive il log)")
    args = ap.parse_args()

    try:
        data = np.load(args.log, allow_pickle=True)
    except Exception as e:
        print(f"Errore nel caricamento del log: {e}")
        return

    t = data['t']
    mass = data['mass'] if 'mass' in data.files else 2.0
    g = 9.80665
    block = not args.all and not args.save # Se vogliamo sequenziale, block=True a ogni plot
    task_start = float(data['task_start_time'][0]) if 'task_start_time' in data.files else -1.0
    if args.task_start is not None:   # argomento CLI sovrascrive il valore del log
        task_start = args.task_start
    print(f"[DEBUG] task_start_time in files: {'task_start_time' in data.files}, value used: {task_start:.3f} s")

    # --- FIGURE 1: Position (ENU) ---
    fig_pos_data = [
        {'sim': data['pos'][:, 0], 'ref': data['pref_pos'][:, 0]},
        {'sim': data['pos'][:, 1], 'ref': data['pref_pos'][:, 1]},
        {'sim': data['pos'][:, 2], 'ref': data['pref_pos'][:, 2]}
    ]
    myPlot(t, fig_pos_data, ["Position X [m]", "Position Y [m]", "Position Z [m]"], 
           "Drone Position vs MPC Reference", ncols=3, use_tex=args.tex, block=block, fignum=1, task_start=task_start)

    # --- FIGURE 2: Orientation (RPY) ---
    fig_rpy_data = [
        {'sim': data['rpy'][:, 0], 'ref': data['pref_rpy'][:, 0]},
        {'sim': data['rpy'][:, 1], 'ref': data['pref_rpy'][:, 1]}
    ]
    myPlot(t, fig_rpy_data, ["Roll [rad]", "Pitch [rad]"], 
           "Drone Orientation (Roll/Pitch) vs MPC Reference", ncols=2, use_tex=args.tex, block=block, fignum=2, task_start=task_start)

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

    # --- FIGURE 4: Coordinate Sferiche (r, beta, gamma) vs Riferimento ---
    fig4_data = [
        {'sim': data['r_sph'],    'ref': data['online_sph_ref'][:, 0]},
        {'sim': data['beta_sph'], 'ref': data['online_sph_ref'][:, 1]},
        {'sim': data['gamma_sph'],'ref': data['online_sph_ref'][:, 2]},
    ]
    myPlot(t, fig4_data,
           ["Distance r [m]", "Azimuth beta [rad]", "Elevation gamma [rad]"],
           "Spherical PoV Tracking (World Frame)",
           ncols=3, use_tex=args.tex, block=block, fignum=4, task_start=task_start)

    # --- FIGURE 5: Yaw Tracking (puntamento verso oggetto) ---
    yaw_actual  = data['rpy'][:, 2]
    yaw_desired = np.arctan2(-data['r_sph'] * np.sin(data['beta_sph']),
                             -data['r_sph'] * np.cos(data['beta_sph']))
    # Nota: yaw_desired calcolato dal vettore drone->obj (= -p_rel direction)
    yaw_desired = np.arctan2(
        -(data['pos'][:, 1] - data['peg_pos'][:, 1]),
        -(data['pos'][:, 0] - data['peg_pos'][:, 0])
    )
    fig5_data = [
        {'sim': yaw_actual,          'ref': yaw_desired},
        {'sim': data['yaw_err_sph'], 'ref': 0.0},
    ]
    myPlot(t, fig5_data,
           ["Yaw Actual vs Desired [rad]", "Yaw Error [rad]"],
           "Yaw Tracking: Drone Pointing Toward Target",
           ncols=2, use_tex=args.tex, block=block, fignum=5, task_start=task_start)

    # --- FIGURE 6: Errori di Tracking Primari (sferici + posizione + orientamento) ---
    err_pos = np.linalg.norm(data['pos'][:, :2] - data['pref_pos'][:, :2], axis=1)
    err_r   = np.abs(data['r_sph']    - data['online_sph_ref'][:, 0])
    err_beta  = np.abs(np.arctan2(
        np.sin(data['beta_sph']  - data['online_sph_ref'][:, 1]),
        np.cos(data['beta_sph']  - data['online_sph_ref'][:, 1])))
    err_gamma = np.abs(data['gamma_sph'] - data['online_sph_ref'][:, 2])
    err_yaw = np.abs(data['yaw_err_sph'])
    err_rp = np.linalg.norm(data['q'][:, 1:3], axis=1)  # qx, qy

    fig6_data = [
        {'sim': err_pos,   'ref': 0},
        {'sim': err_r,     'ref': 0},
        {'sim': err_beta,  'ref': 0},
        {'sim': err_gamma, 'ref': 0},
        {'sim': err_yaw,   'ref': 0},
        {'sim': err_rp,    'ref': 0},
    ]
    myPlot(t, fig6_data,
           ["Norm Pos Error XY [m]", "Distance Error |r_err| [m]",
            "Azimuth Error |beta_err| [rad]", "Elevation Error |gamma_err| [rad]",
            "Yaw Error |yaw_err| [rad]", "Norm Roll/Pitch Error"],
           "Primary Tracking Errors (Spherical)",
           ncols=3, use_tex=args.tex, block=block, fignum=6, task_start=task_start)

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
           "Dynamic States Errors and Derivatives", ncols=3, use_tex=args.tex, block=block, fignum=7, task_start=task_start)

    # --- FIGURE 8: Wrench ---
    fig8_data = [
        {'sim': data['wrench_cmd'][:, 0], 'ref': data['wrench_target'][:, 0]},
        {'sim': data['wrench_cmd'][:, 1], 'ref': data['wrench_target'][:, 1]},
        {'sim': data['wrench_cmd'][:, 2], 'ref': data['wrench_target'][:, 2]},
        {'sim': data['wrench_cmd'][:, 3], 'ref': data['wrench_target'][:, 3]}
    ]
    myPlot(t, fig8_data, ["Force Z (Thrust) [N]", "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"], 
           f"Control Wrench (Hover Force = {mass*g:.2f}N)", ncols=2, use_tex=args.tex, block=block, fignum=8, task_start=task_start)

    # --- FIGURE 9: Haptic Forces ---
    if 'haptic_force' in data.files:
        fig9_data = [
            {'sim': data['haptic_force'][:, 0], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 1], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig9_data, ["Force X (Zoom) [N]", "Force Y (Pan) [N]", "Force Z (Altitude) [N]"], 
               "Haptic Feedback Forces Transmitted to haptic device", ncols=3, use_tex=args.tex, block=block, fignum=9, task_start=task_start)

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
           "Drone Linear and Angular Accelerations", ncols=3, use_tex=args.tex, block=block, fignum=10, task_start=task_start)

    # --- FIGURE 11: Peg External Forces ---
    if 'peg_ext_force' in data.files:
        fig11_data = [
            {'sim': data['peg_ext_force'][:, 0], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 1], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig11_data, 
               ["Force X (Sensor) [N]", "Force Y (Sensor) [N]", "Force Z (Sensor) [N]"], 
               "Peg External Contact Forces (FT Sensor)", ncols=3, use_tex=args.tex, block=block, fignum=11, task_start=task_start)

    # --- FIGURE 12: Admittance delta_p (spostamento di ammettenza) ---
    if 'delta_p' in data.files:
        dp = data['delta_p']
        dp_norm = np.linalg.norm(dp, axis=1)
        fig12_data = [
            {'sim': dp[:, 0], 'ref': 0.0},
            {'sim': dp[:, 1], 'ref': 0.0},
            {'sim': dp[:, 2], 'ref': 0.0},
            {'sim': dp_norm,  'ref': 0.0}
        ]
        myPlot(t, fig12_data,
               [r"$\Delta p_x$ [m]", r"$\Delta p_y$ [m]", r"$\Delta p_z$ [m]",
                r"$\|\Delta p\|$ [m]"],
               "Admittance Displacement $\\Delta p$ (Sensor Frame)",
               ncols=2, use_tex=args.tex, block=block, fignum=12, task_start=task_start)

    # --- FIGURE 13: Confronto ||delta_p|| vs ||F_ext|| ---
    if 'delta_p' in data.files and 'peg_ext_force' in data.files:
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
    has_peg_actual = 'peg_actual_pos' in data.files
    has_peg_ref    = 'peg_ref_pos'    in data.files
    has_peg_yaw    = 'peg_actual_yaw' in data.files
    if has_peg_actual or has_peg_ref:
        peg_act = data['peg_actual_pos'] if has_peg_actual else np.zeros((len(t), 3))
        peg_ref = data['peg_ref_pos']    if has_peg_ref    else None

        fig14_data = [
            {'sim': peg_act[:, 0], 'ref': peg_ref[:, 0] if peg_ref is not None else None},
            {'sim': peg_act[:, 1], 'ref': peg_ref[:, 1] if peg_ref is not None else None},
            {'sim': peg_act[:, 2], 'ref': peg_ref[:, 2] if peg_ref is not None else None},
        ]
        if has_peg_yaw:
            peg_ref_yaw = data['peg_ref_yaw'] if 'peg_ref_yaw' in data.files else None
            fig14_data.append({'sim': data['peg_actual_yaw'], 'ref': peg_ref_yaw})
        myPlot(t, fig14_data,
               ["Peg X [m]", "Peg Y [m]", "Peg Z [m]"] + (["Peg Yaw [rad]"] if has_peg_yaw else []),
               "Interaction Drone Position ENU (Actual vs Planner Reference)",
               ncols=2, use_tex=args.tex, block=block, fignum=14, task_start=task_start)

    # --- FIGURE 15: Interaction Drone Velocities (ENU) + Yaw Rate ---
    has_peg_vel      = 'peg_actual_vel'      in data.files
    has_peg_yaw_rate = 'peg_actual_yaw_rate' in data.files
    if has_peg_vel or has_peg_yaw_rate:
        fig15_data, labels15 = [], []
        peg_ref_vel      = data['peg_ref_vel']      if 'peg_ref_vel'      in data.files else None
        peg_ref_yaw_rate = data['peg_ref_yaw_rate'] if 'peg_ref_yaw_rate' in data.files else None
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

    if args.save:
        for i in plt.get_fignums():
            plt.figure(i).savefig(f"plot_fig_{i}.png")
        print("Grafici salvati.")
    elif args.all:
        plt.show()

if __name__ == "__main__":
    main()
