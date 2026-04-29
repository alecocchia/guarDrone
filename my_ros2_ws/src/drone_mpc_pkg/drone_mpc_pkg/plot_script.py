#!/usr/bin/env python3
import argparse, numpy as np
import matplotlib.pyplot as plt

def myPlot(time, data_list, labels, title, ncols=2, use_tex=True, block=False):
    plt.rcParams.update({"text.usetex": use_tex, "font.family": "serif"})
    n = len(data_list)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3.5 * nrows), squeeze=False)
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
    ap.add_argument("--log", type=str, default="/tmp/pid_run.npz")
    ap.add_argument("--tex", action="store_true")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--all", action="store_true", help="Show all figures at once (default is sequential)")
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

    # --- FIGURE 1: Position (ENU) ---
    fig_pos_data = [
        {'sim': data['pos'][:, 0], 'ref': data['pref_pos'][:, 0]},
        {'sim': data['pos'][:, 1], 'ref': data['pref_pos'][:, 1]},
        {'sim': data['pos'][:, 2], 'ref': data['pref_pos'][:, 2]}
    ]
    myPlot(t, fig_pos_data, ["Position X [m]", "Position Y [m]", "Position Z [m]"], 
           "Drone Position vs MPC Reference", ncols=3, use_tex=args.tex, block=block)

    # --- FIGURE 2: Orientation (RPY) ---
    fig_rpy_data = [
        {'sim': data['rpy'][:, 0], 'ref': data['pref_rpy'][:, 0]},
        {'sim': data['rpy'][:, 1], 'ref': data['pref_rpy'][:, 1]},
        {'sim': data['rpy'][:, 2], 'ref': data['pref_rpy'][:, 2]}
    ]
    myPlot(t, fig_rpy_data, ["Roll [rad]", "Pitch [rad]", "Yaw [rad]"], 
           "Drone Orientation vs MPC Reference", ncols=3, use_tex=args.tex, block=block)

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
           "Drone Velocities vs MPC Reference", ncols=3, use_tex=args.tex, block=block)

    # --- FIGURE 4: PoV Orbiting & Orientation ---
    fig4_data = [
        {'sim': data['pan_real'],    'ref': data['online_ref'][:, 1]},
        {'sim': data['rpy'][:, 2],   'ref': data['pref_rpy'][:, 2]},
        {'sim': data['radius_real'], 'ref': data['online_ref'][:, 0]}
    ]
    myPlot(t, fig4_data, ["Pan Mutuo (Orbit) [rad]", "Absolute Yaw [rad]", "Mutual Distance (Xc) [m]"], 
           "PoV Orbiting and Orientation Tracking", ncols=3, use_tex=args.tex, block=block)

    # --- FIGURE 5: Visual Servoing (Camera Frame) ---
    fig5_data = [
        {'sim': data['Xc'], 'ref': data['online_visual_ref'][:, 0]},
        {'sim': data['Yc'], 'ref': data['online_visual_ref'][:, 1]},
        {'sim': data['Zc'], 'ref': data['online_visual_ref'][:, 2]}
    ]
    myPlot(t, fig5_data, ["Xc (Depth/Zoom) [m]", "Yc (Horizontal Offset) [m]", "Zc (Vertical Offset) [m]"], 
           "Visual Servoing: Target Position in Camera Frame", ncols=3, use_tex=args.tex, block=block)

    # --- FIGURE 6: Primary Tracking Errors (Position, Visual, Orientation) ---
    err_pos = np.linalg.norm(data['pos'][:, :2] - data['pref_pos'][:, :2], axis=1)
    err_vis = np.linalg.norm(np.column_stack((data['Xc'], data['Yc'], data['Zc'])) - data['online_visual_ref'], axis=1)
    err_rp = np.linalg.norm(data['q'][:, 1:3], axis=1) # qx, qy
    
    fig6_data = [
        {'sim': err_pos, 'ref': 0}, {'sim': err_vis, 'ref': 0}, {'sim': err_rp, 'ref': 0}
    ]
    myPlot(t, fig6_data, ["Norm Pos Error [m]", "Norm Visual Error [m]", "Norm Roll/Pitch Error"], 
           "Primary Tracking Errors", ncols=3, use_tex=args.tex, block=block)

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
           "Dynamic States Errors and Derivatives", ncols=3, use_tex=args.tex, block=block)

    # --- FIGURE 8: Wrench ---
    fig8_data = [
        {'sim': data['wrench_cmd'][:, 0], 'ref': data['wrench_target'][:, 0]},
        {'sim': data['wrench_cmd'][:, 1], 'ref': data['wrench_target'][:, 1]},
        {'sim': data['wrench_cmd'][:, 2], 'ref': data['wrench_target'][:, 2]},
        {'sim': data['wrench_cmd'][:, 3], 'ref': data['wrench_target'][:, 3]}
    ]
    myPlot(t, fig8_data, ["Force Z (Thrust) [N]", "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"], 
           f"Control Wrench (Hover Force = {mass*g:.2f}N)", ncols=2, use_tex=args.tex, block=block)

    if args.save:
        for i in plt.get_fignums():
            plt.figure(i).savefig(f"plot_fig_{i}.png")
        print("Grafici salvati.")
    elif args.all:
        plt.show()

if __name__ == "__main__":
    main()
