#!/usr/bin/env python3
import os, sys
import argparse, numpy as np

# Risolve potenziale conflitto namespace per mpl_toolkits (mplot3d / Axes3D) prima di importare pyplot
try:
    import mpl_toolkits
    import matplotlib
    local_mpl = os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), 'mpl_toolkits')
    if os.path.exists(local_mpl) and local_mpl not in mpl_toolkits.__path__:
        mpl_toolkits.__path__.insert(0, local_mpl)
    from mpl_toolkits.mplot3d import Axes3D
except Exception:
    pass

import matplotlib.pyplot as plt

# Aggiunta percorsi di ricerca per utils_pkg (esecuzione sia standalone che ROS2)
current_dir = os.path.dirname(os.path.abspath(__file__))
ws_src_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
for p in [current_dir, os.path.join(ws_src_dir, 'utils_pkg'), '/root/my_ros2_ws/src/utils_pkg']:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

from utils_pkg.utils_np import wrap_pi, cylindrical_to_cartesian

def myPlot(time, data_list, labels, title, ncols=2, use_tex=True, block=False, fignum=None, task_start=-1.0, task_end=-1.0, phases=None, haptic_intervals=None):
    """Generazione grafici ad alta leggibilità con palette moderna Tableau, griglia discreta e marker eleganti."""
    plt.rcParams.update({"text.usetex": use_tex, "font.family": "serif", "axes.ymargin": 0.15})
    if phases is None and hasattr(myPlot, 'default_phases'):
        phases = myPlot.default_phases
    if haptic_intervals is None and hasattr(myPlot, 'default_haptic_intervals'):
        haptic_intervals = myPlot.default_haptic_intervals

    n = len(data_list)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12.5, 3.0 * nrows + 0.6), squeeze=False, num=fignum)
    fig.patch.set_facecolor('#ffffff')
    if fignum is not None:
        try:
            fig.canvas.manager.set_window_title(f"Figure {fignum}: {title}")
        except AttributeError:
            try:
                fig.canvas.set_window_title(f"Figure {fignum}: {title}")
            except Exception:
                pass
    axes = axes.flatten()

    # Stile cromatico moderno: Tableau blue per Actual, Crimson red tratteggiato per Reference
    c_actual = '#1f77b4'
    c_ref = '#d62728'
    c_start = '#2ca02c'   # Emerald green
    c_end = '#7f7f7f'     # Slate gray
    
    # Mappatura stili grafici per ciascuna fase della missione FSM
    phase_style_map = {
        'WAIT_TAKEOFF':        {'color': '#94a3b8', 'linestyle': ':',  'label': 'Wait Takeoff'},
        'ARM_OFFBOARD':        {'color': '#64748b', 'linestyle': ':',  'label': 'Arm & Offboard'},
        'TAKEOFF_MONITOR':     {'color': '#0284c7', 'linestyle': '--', 'label': 'Takeoff'},
        'HOVERING':            {'color': '#2563eb', 'linestyle': '--', 'label': 'Hovering'},
        'APPROACH':            {'color': '#f59e0b', 'linestyle': '--', 'label': 'Approach'},
        'INTERACTION':         {'color': '#16a34a', 'linestyle': '-',  'label': 'Interaction'},
        'HOME':                {'color': '#2563eb', 'linestyle': '--', 'label': 'Home'},
        'MISSION_START':       {'color': '#f59e0b', 'linestyle': '--', 'label': 'Mission Start'},
        'INSPECTION_START':    {'color': '#16a34a', 'linestyle': '-',  'label': 'Inspection Start'},
        'INSPECTION_END':      {'color': '#9333ea', 'linestyle': '--', 'label': 'Inspection End'},
        'MISSION_PREPARATION': {'color': '#f59e0b', 'linestyle': '--', 'label': 'Mission Prep'},
        'MISSION':             {'color': '#16a34a', 'linestyle': '-',  'label': 'Mission'},
        'DETACHMENT':          {'color': '#9333ea', 'linestyle': '--', 'label': 'Detachment'},
        'RETURN_HOME':         {'color': '#2563eb', 'linestyle': '--', 'label': 'Return Home'},
        'LANDING':             {'color': '#dc2626', 'linestyle': '--', 'label': 'Landing'},
        'EMERGENCY':           {'color': '#b91c1c', 'linestyle': '-.', 'label': 'Emergency'},
        'Mission Start':       {'color': '#2ca02c', 'linestyle': '--', 'label': 'Mission Start'},
        'Mission End':         {'color': '#7f7f7f', 'linestyle': ':',  'label': 'Mission End'},
    }
    fallback_colors = ['#0284c7', '#f59e0b', '#16a34a', '#9333ea', '#2563eb', '#dc2626', '#64748b']

    # Identifica gli intervalli temporali in cui l'MPC è attivo per l'ombreggiatura gialla
    # (escluse le fasi preparatorie/decollo e atterraggio/emergenza)
    NON_MPC_PHASES = {'WAIT_EKF', 'WAIT_START', 'WAIT_TAKEOFF', 'ARM_OFFBOARD', 'TAKEOFF_MONITOR',
                      'LANDING', 'DISARM_WAIT', 'MISSION_COMPLETE', 'EMERGENCY'}
    mpc_intervals = []
    if phases:
        in_mpc = False
        t_start_mpc = None
        for idx_p, (pt, pname) in enumerate(phases):
            is_mpc = (pname not in NON_MPC_PHASES)
            if is_mpc and not in_mpc:
                in_mpc = True
                t_start_mpc = pt
            elif not is_mpc and in_mpc:
                in_mpc = False
                mpc_intervals.append((t_start_mpc, pt))
        if in_mpc:
            mpc_intervals.append((t_start_mpc, float(time[-1])))
    elif task_start > 0:
        t_end_val = task_end if (task_end > task_start and task_end < float(time[-1])) else float(time[-1])
        mpc_intervals.append((task_start, t_end_val))

    for i in range(n):
        ax = axes[i]
        time_plot = time[:len(data_list[i]['sim'])]

        # Ombreggiatura gialla per evidenziare quando l'MPC è attivo
        for idx_m, (t_m_s, t_m_e) in enumerate(mpc_intervals):
            ax.axvspan(t_m_s, t_m_e, color='#facc15', alpha=0.10,
                       label='MPC Active' if idx_m == 0 else None, zorder=0)

        # Ombreggiatura intervalli di utilizzo dell'haptic device (rosso per GuarDrone, viola per Interaction)
        if haptic_intervals:
            for idx_h, item in enumerate(haptic_intervals):
                if len(item) >= 4:
                    t_h_start, t_h_end, h_color, h_label = item[:4]
                else:
                    t_h_start, t_h_end = item[:2]
                    h_color = '#ef4444'
                    h_label = 'Haptic Active'
                ax.axvspan(t_h_start, t_h_end, color=h_color, alpha=0.15,
                           label=h_label if idx_h == 0 else None, zorder=0)

        # Plot segnale effettivo
        ax.plot(time_plot, data_list[i]['sim'], color=c_actual, label='Actual', linewidth=1.7, zorder=3)

        # Plot riferimento
        if 'ref' in data_list[i] and data_list[i]['ref'] is not None:
            ref_data = data_list[i]['ref']
            if np.isscalar(ref_data):
                ax.axhline(y=ref_data, color=c_ref, linestyle='--', label='Ref', linewidth=1.3, alpha=0.85, zorder=2)
            else:
                ax.plot(time_plot, ref_data[:len(time_plot)], color=c_ref, linestyle='--', label='Reference', linewidth=1.3, alpha=0.85, zorder=2)

        # Indicatori di transizione delle fasi di missione
        if phases:
            for idx_p, (pt, pname) in enumerate(phases):
                st = phase_style_map.get(pname, {
                    'color': fallback_colors[idx_p % len(fallback_colors)],
                    'linestyle': '--',
                    'label': pname
                })
                if pt >= 0:
                    ax.axvline(x=pt, color=st['color'], linestyle=st['linestyle'],
                               linewidth=1.3 if pname in ('MISSION', 'LANDING') else 1.1,
                               alpha=0.85 if pname in ('MISSION', 'LANDING') else 0.70,
                               label=st['label'])
        else:
            if task_start > 0:
                ax.axvline(x=task_start, color=c_start, linestyle='--', linewidth=1.3, alpha=0.75, label='Mission Start')
            if task_end > 0 and task_end < float(time[-1]):
                ax.axvline(x=task_end, color='#dc2626', linestyle='--', linewidth=1.3, alpha=0.75, label='Landing')
        
        ax.set_title(labels[i], fontsize=10.5, fontweight='semibold', color='#0f172a', pad=4)
        ax.set_xlabel('Time [s]', fontsize=9, color='#334155')
        ax.tick_params(axis='both', which='both', labelsize=8.5, colors='#334155')
        ax.grid(True, linestyle='--', alpha=0.45, color='#94a3b8')

        # Bordi e assi sottili ed eleganti
        for spine in ax.spines.values():
            spine.set_color('#cbd5e1')
            spine.set_linewidth(0.8)

    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])

    # Legenda unica condivisa posizionata all'esterno in alto (sotto il titolo)
    handles_dict = {}
    for ax in axes[:n]:
        h, l = ax.get_legend_handles_labels()
        for handle, label in zip(h, l):
            if label not in handles_dict and label and not label.startswith('_'):
                handles_dict[label] = handle

    # Margine superiore e posizionamento legenda adattivi
    if len(handles_dict) > 5:
        top_rect = 0.79 if nrows == 1 else 0.86
        leg_y = 0.88 if nrows == 1 else 0.925
    else:
        top_rect = 0.83 if nrows == 1 else 0.89
        leg_y = 0.89 if nrows == 1 else 0.93
    title_y = 0.98 if nrows == 1 else 0.99

    if handles_dict:
        fig.legend(
            list(handles_dict.values()),
            list(handles_dict.keys()),
            loc='upper center',
            bbox_to_anchor=(0.5, leg_y),
            ncol=min(len(handles_dict), 5),
            frameon=True,
            facecolor='white',
            framealpha=0.92,
            edgecolor='#cbd5e1',
            fontsize=8.5
        )

    fig.suptitle(title, fontsize=13.0, fontweight='bold', color='#0f172a', y=title_y)
    plt.tight_layout(rect=[0, 0.02, 1, top_rect])
    
    if block:
        plt.show()
    return fig

def filter_spatial_trajectory(pts, d_min=0.03):
    """Filtra micro-jitter da hovering mantenendo solo avanzamenti reali >= d_min."""
    if len(pts) <= 2 or d_min <= 0: return pts
    res = [pts[0]]
    for p in pts[1:-1]:
        if np.linalg.norm(p - res[-1]) >= d_min:
            res.append(p)
    res.append(pts[-1])
    return np.array(res)

def smooth_trajectory(pts, w=7):
    """Ammorbidisce la traiettoria con media mobile preservando gli estremi."""
    if len(pts) < w or w <= 1: return pts
    pad = np.pad(pts, ((w // 2, w // 2), (0, 0)), mode='edge')
    return np.column_stack([np.convolve(pad[:, i], np.ones(w) / w, mode='valid') for i in range(3)])

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
    ap.add_argument("--d-min-3d", type=float, default=0.08,
                    help="[m] Soglia minima filtro spaziale 3D per hovering (default: 0.08, 0 per disattivare)")
    ap.add_argument("--smooth-window-3d", type=int, default=21,
                    help="Finestra media mobile traiettoria 3D (default: 21, <=1 per disattivare)")
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
    task_end = float(np.asarray(data['task_end_time']).flat[0]) if indata('task_end_time') else -1.0
    print(f"[DEBUG] task_end_time presente: {indata('task_end_time')}, value used: {task_end:.3f} s")

    # --- Caricamento fasi FSM registrate dal supervisore ---
    phases = []
    if indata('phase_times') and indata('phase_names'):
        p_times = np.atleast_1d(data['phase_times'])
        p_names = np.atleast_1d(data['phase_names'])
        for pt, pn in zip(p_times, p_names):
            name_str = str(pn).strip()
            # Mostriamo solo le fasi operative significative nei plot, escludendo preparatori e disarmo
            if name_str not in ('WAIT_EKF', 'WAIT_START', 'WAIT_TAKEOFF', 'ARM_OFFBOARD', 'DISARM_WAIT', 'MISSION_COMPLETE'):
                phases.append((float(pt), name_str))
        print(f"[DEBUG] Fasi FSM caricate dal log ({len(phases)}): {phases}")
    myPlot.default_phases = phases

    # Se task_start non era esplicito ma abbiamo le fasi, allinea task_start alla preparazione/missione
    if task_start <= 0 and phases:
        for pt, pn in phases:
            if pn in ('HOVERING', 'APPROACH', 'INTERACTION', 'DETACHMENT', 'RETURN_HOME', 'HOME', 'MISSION_START', 'INSPECTION_START', 'MISSION_PREPARATION', 'MISSION'):
                task_start = pt
                break

    # --- Rilevamento intervalli di attività dell'haptic device (GuarDrone vs Interaction) ---
    haptic_gd_intervals = []
    h_gd_key = 'haptic_guardrone_active' if indata('haptic_guardrone_active') else 'haptic_active'
    if indata(h_gd_key):
        h_act = np.asarray(data[h_gd_key]).flat
        t_arr = np.asarray(t)
        in_haptic = False
        t_h_start = 0.0
        for idx_h, val in enumerate(h_act):
            if val > 0.5 and not in_haptic:
                in_haptic = True
                t_h_start = float(t_arr[idx_h])
            elif val <= 0.5 and in_haptic:
                in_haptic = False
                haptic_gd_intervals.append((t_h_start, float(t_arr[idx_h]), '#ef4444', 'Haptic GuarDrone'))
        if in_haptic:
            haptic_gd_intervals.append((t_h_start, float(t_arr[-1]), '#ef4444', 'Haptic GuarDrone'))
        if haptic_gd_intervals:
            print(f"[DEBUG] Intervalli Haptic GuarDrone ({len(haptic_gd_intervals)}): {haptic_gd_intervals}")

    haptic_peg_intervals = []
    if indata('haptic_peg_active'):
        h_peg = np.asarray(data['haptic_peg_active']).flat
        t_arr = np.asarray(t)
        in_haptic_peg = False
        t_hp_start = 0.0
        for idx_h, val in enumerate(h_peg):
            if val > 0.5 and not in_haptic_peg:
                in_haptic_peg = True
                t_hp_start = float(t_arr[idx_h])
            elif val <= 0.5 and in_haptic_peg:
                in_haptic_peg = False
                haptic_peg_intervals.append((t_hp_start, float(t_arr[idx_h]), '#a855f7', 'Haptic Interaction'))
        if in_haptic_peg:
            haptic_peg_intervals.append((t_hp_start, float(t_arr[-1]), '#a855f7', 'Haptic Interaction'))
        if haptic_peg_intervals:
            print(f"[DEBUG] Intervalli Haptic Interaction ({len(haptic_peg_intervals)}): {haptic_peg_intervals}")

    # Default per figure GuarDrone (fig 1-10, 17-19)
    myPlot.default_haptic_intervals = haptic_gd_intervals
    myPlot.haptic_gd_intervals = haptic_gd_intervals
    myPlot.haptic_peg_intervals = haptic_peg_intervals

    # --- Mascheramento riferimenti pre-task con NaN ---
    # Prima dello switch/task_start i riferimenti dell'MPC non sono attivi.
    # Impostando a NaN i campioni pre-task, Matplotlib non disegna nulla prima dello switch,
    # eliminando qualsiasi discontinuità grafica a gradino da zero o falsi errori durante il decollo.
    def nan_pre_task(arr, idx_s):
        """Imposta a NaN i campioni prima dell'indice di avvio task."""
        arr = np.asarray(arr, dtype=float).copy()
        if idx_s > 0 and idx_s <= len(arr):
            arr[:idx_s] = np.nan
        return arr

    # Determina l'indice di avvio dell'MPC
    idx_s = 0
    if task_start > 0:
        idx_s = int(np.searchsorted(t, task_start))

    # Avanza idx_s al primo campione effettivamente valido (diverso da zero)
    # per compensare il delay tra il segnale di start e il primo solve dell'MPC
    ref_check_key = 'pref_pos' if indata('pref_pos') else ('online_cyl_ref' if indata('online_cyl_ref') else None)
    if ref_check_key is not None:
        sub_arr = np.asarray(data[ref_check_key])
        nonzero = np.where(np.any(sub_arr[idx_s:] != 0, axis=-1))[0]
        if len(nonzero) > 0:
            idx_s = idx_s + nonzero[0]
        elif idx_s == 0:
            nonzero_all = np.where(np.any(sub_arr != 0, axis=-1))[0]
            if len(nonzero_all) > 0:
                idx_s = nonzero_all[0]

    if idx_s > 0:
        # 1. Variabili puramente MPC / task PoV (non esistono durante il decollo): sempre NaN pre-task
        keys_mpc_only = [
            'optimal_wrench', 'wrench_target', 'online_cyl_ref', 'online_ref',
            'actual_pov', 'r_cyl', 'beta_cyl', 'z_cyl', 'yaw_err_cyl'
        ]
        for key in keys_mpc_only:
            if indata(key):
                data[key] = nan_pre_task(data[key], idx_s)

        # 2. Riferimenti cinematici del drone (pref_pos, pref_rpy, vref, omegaref):
        # Se contengono la traiettoria di decollo di offboard_trajectory_planner, li manteniamo continui!
        # Se invece erano rimasti a zero (es. log vecchi), mascheriamo a NaN per evitare il gradino.
        for key in ['pref_pos', 'pref_rpy', 'pref_q', 'vref', 'omegaref']:
            if indata(key):
                sub = np.asarray(data[key])
                if np.all(sub[:idx_s] == 0):
                    data[key] = nan_pre_task(data[key], idx_s)

        # Ricalcola p_cam_target dopo il mascheramento con NaN
        if indata('online_cyl_ref') and indata('peg_pos'):
            data['p_cam_target'] = cylindrical_to_cartesian(
                data['online_cyl_ref'], p_origin=np.asarray(data['peg_pos']))

    # --- FIGURE 1: 3D Trajectories (GuarDrone & Interaction Drone) ---
    plt.rcParams.update({"text.usetex": args.tex, "font.family": "serif"})
    fig1 = plt.figure(figsize=(10, 8), num=1)
    ax1 = fig1.add_subplot(111, projection='3d')
    fig1.patch.set_facecolor('#ffffff')
    ax1.set_facecolor('#ffffff')

    try:
        fig1.canvas.manager.set_window_title("Figure 1: 3D Drones Trajectories (World Frame ENU)")
    except AttributeError:
        try:
            fig1.canvas.set_window_title("Figure 1: 3D Drones Trajectories (World Frame ENU)")
        except Exception:
            pass

    # Traiettoria GuarDrone
    pos = np.asarray(data['pos'])
    pos_plot = smooth_trajectory(filter_spatial_trajectory(pos, args.d_min_3d), args.smooth_window_3d)
    ax1.plot(pos_plot[:, 0], pos_plot[:, 1], pos_plot[:, 2], color='#1f77b4', linewidth=2.0, label='GuarDrone Trajectory', zorder=3)

    # Traiettoria Interaction Drone (Peg)
    all_pts = [pos]
    has_peg_actual = indata('peg_actual_pos')
    has_peg_pos    = indata('peg_pos')
    if has_peg_actual or has_peg_pos:
        peg_act = np.asarray(data['peg_actual_pos']) if has_peg_actual else np.asarray(data['peg_pos'])
        if peg_act.ndim == 1:
            peg_act = peg_act.reshape(1, -1)
        if len(peg_act) > 0:
            all_pts.append(peg_act)
            if len(peg_act) > 1 and np.linalg.norm(peg_act[-1] - peg_act[0]) > 0.05:
                peg_plot = smooth_trajectory(filter_spatial_trajectory(peg_act, args.d_min_3d), args.smooth_window_3d)
                ax1.plot(peg_plot[:, 0], peg_plot[:, 1], peg_plot[:, 2], color='#9333ea', linewidth=2.0, label='Interaction Drone Trajectory', zorder=3)
                ax1.scatter(peg_act[0, 0], peg_act[0, 1], peg_act[0, 2], color='#16a34a', marker='o', s=60, edgecolors='black', linewidth=0.8, zorder=5)
                ax1.scatter(peg_act[-1, 0], peg_act[-1, 1], peg_act[-1, 2], color='#dc2626', marker='s', s=60, edgecolors='black', linewidth=0.8, zorder=5)
            else:
                # Hovering statico
                ax1.scatter(peg_act[0, 0], peg_act[0, 1], peg_act[0, 2], color='#9333ea', marker='^', s=80, edgecolors='black', linewidth=0.8, label='Interaction Drone (Hovering)', zorder=5)

    # Punti di Start ed End (stesso simbolo e colore per entrambi i droni)
    ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color='#16a34a', marker='o', s=60, edgecolors='black', linewidth=0.8, label='Start', zorder=5)
    ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], color='#dc2626', marker='s', s=60, edgecolors='black', linewidth=0.8, label='End', zorder=5)

    ax1.set_xlabel('X [m]', fontsize=10, labelpad=8)
    ax1.set_ylabel('Y [m]', fontsize=10, labelpad=8)
    ax1.set_zlabel('Z [m]', fontsize=10, labelpad=8)
    ax1.set_title('3D Drones Trajectories in World Frame (ENU)', fontsize=13, fontweight='bold', pad=15, color='#0f172a')
    ax1.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.92, edgecolor='#cbd5e1', fontsize=8.5)
    ax1.grid(True, linestyle='--', alpha=0.45, color='#94a3b8')

    # Scala assi isotropa (proporzioni 1:1:1 reali nello spazio 3D)
    all_coords = np.vstack(all_pts)
    max_range = np.array([
        all_coords[:, 0].max() - all_coords[:, 0].min(),
        all_coords[:, 1].max() - all_coords[:, 1].min(),
        all_coords[:, 2].max() - all_coords[:, 2].min()
    ]).max() / 2.0
    margin = max(max_range, 0.5)
    mid_x = (all_coords[:, 0].max() + all_coords[:, 0].min()) * 0.5
    mid_y = (all_coords[:, 1].max() + all_coords[:, 1].min()) * 0.5
    mid_z = (all_coords[:, 2].max() + all_coords[:, 2].min()) * 0.5

    ax1.set_xlim(mid_x - margin, mid_x + margin)
    ax1.set_ylim(mid_y - margin, mid_y + margin)
    ax1.set_zlim(mid_z - margin, mid_z + margin)

    plt.tight_layout()
    if block:
        plt.show()

    # --- FIGURE 2: Orientation (RPY) ---
    fig_rpy_data = [
        {'sim': data['rpy'][:, 0], 'ref': data['pref_rpy'][:, 0]},
        {'sim': data['rpy'][:, 1], 'ref': data['pref_rpy'][:, 1]}
    ]
    myPlot(t, fig_rpy_data, ["Roll [rad]", "Pitch [rad]"], 
           "Drone Orientation (Roll/Pitch)", ncols=2, use_tex=args.tex, block=block, fignum=2, task_start=task_start, task_end=task_end)

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
           "Drone Velocities vs MPC Reference", ncols=3, use_tex=args.tex, block=block, fignum=3, task_start=task_start, task_end=task_end)

    # --- FIGURE 4: Riferimento Cartesiano Camera vs Posizione Effettiva Camera ---
    # Usiamo direttamente p_cam e p_cam_target salvati dal logger.py
    # Questi tengono conto correttamente dell'offset della telecamera 
    fig_cart_ref_data = [
        {'sim': data['p_cam'][:, 0], 'ref': data['p_cam_target'][:, 0]},
        {'sim': data['p_cam'][:, 1], 'ref': data['p_cam_target'][:, 1]},
        {'sim': data['p_cam'][:, 2], 'ref': data['p_cam_target'][:, 2]}
    ]
    myPlot(t, fig_cart_ref_data, ["Cam X [m]", "Cam Y [m]", "Cam Z [m]"], 
           "Camera Cartesian Position vs Derived Target", ncols=3, use_tex=args.tex, block=block, fignum=4, task_start=task_start, task_end=task_end)

    # --- FIGURE 5: Coordinate Cilindriche (r_cyl, beta, z) vs Riferimento ---
    # Allineamento dell'azimut per evitare che ref e sim si sdoppino di 2*pi nel plot
    beta_sim_unwrapped = np.full_like(data['beta_cyl'], np.nan)
    if idx_s < len(data['beta_cyl']):
        beta_sim_unwrapped[idx_s:] = np.unwrap(data['beta_cyl'][idx_s:])
    else:
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
           ncols=3, use_tex=args.tex, block=block, fignum=5, task_start=task_start, task_end=task_end)

    # --- FIGURE 6: Yaw Tracking (puntamento verso oggetto) ---
    # yaw_err_cyl è già loggato direttamente dall'MPC (wrap_pi applicato correttamente)
    # yaw_desired = beta_cyl + pi (puntamento ottico reale coerente con la definizione MPC)
    yaw_actual  = wrap_pi(data['rpy'][:, 2])
    # yaw_offset_ref: decentramento FoV impostato dal supervisore (default 0 per log vecchi a 3 colonne)
    _cyl_ref = np.asarray(data['online_cyl_ref'])
    yaw_offset_ref = _cyl_ref[:, 3] if _cyl_ref.ndim == 2 and _cyl_ref.shape[1] > 3 else 0.0
    # yaw_desired include il decentramento FoV (yaw_offset_ref) coerentemente con l'MPC
    yaw_desired = wrap_pi(data['beta_cyl'] + np.pi + yaw_offset_ref)
    fig5_data = [
        {'sim': yaw_actual,               'ref': yaw_desired},
        {'sim': data['yaw_err_cyl'],      'ref': yaw_offset_ref},
    ]
    myPlot(t, fig5_data,
           ["Yaw Actual vs Desired [rad]", "Yaw Error [rad]"],
           "Yaw Tracking: Drone Pointing Target",
           ncols=2, use_tex=args.tex, block=block, fignum=6, task_start=task_start, task_end=task_end)

    # --- FIGURE 6 (fignum=7): Errori di Tracking Primari (Cilindrici & Yaw) ---
    err_r   = np.abs(data['r_cyl']    - data['online_cyl_ref'][:, 0])
    err_beta  = np.abs(np.arctan2(
        np.sin(data['beta_cyl']  - data['online_cyl_ref'][:, 1]),
        np.cos(data['beta_cyl']  - data['online_cyl_ref'][:, 1])))
    err_z = np.abs(data['z_cyl'] - data['online_cyl_ref'][:, 2])
    err_yaw = np.abs(data['yaw_err_cyl'] - yaw_offset_ref)

    fig6_data = [
        {'sim': err_r,     'ref': 0.0},
        {'sim': err_beta,  'ref': 0.0},
        {'sim': err_z,     'ref': 0.0},
        {'sim': err_yaw,   'ref': 0.0},
    ]
    myPlot(t, fig6_data,
           ["Distance Error |r_cyl_err| [m]",
            "Azimuth Error |beta_err| [rad]", 
            "Elevation Error |z_err| [m]",
            "Yaw Error |yaw_err| [rad]"],
           "Primary Tracking Errors (Cylindrical & Yaw)",
           ncols=2, use_tex=args.tex, block=block, fignum=7, task_start=task_start, task_end=task_end)

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
           "Dynamic States Errors and Feedforward Derivatives", ncols=3, use_tex=args.tex, block=block, fignum=8, task_start=task_start, task_end=task_end)

    # --- FIGURE 8: Wrench ---
    fig8_data = [
        {'sim': data['optimal_wrench'][:, 0], 'ref': data['wrench_target'][:, 0]},
        {'sim': data['optimal_wrench'][:, 1], 'ref': data['wrench_target'][:, 1]},
        {'sim': data['optimal_wrench'][:, 2], 'ref': data['wrench_target'][:, 2]},
        {'sim': data['optimal_wrench'][:, 3], 'ref': data['wrench_target'][:, 3]}
    ]
    myPlot(t, fig8_data, ["Force Z (Thrust) [N]", "Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"], 
           f"Control Wrench (Hover Force = {mass*g:.2f}N)", ncols=2, use_tex=args.tex, block=block, fignum=9, task_start=task_start, task_end=task_end)

    # --- FIGURE 9 (fignum=10): Haptic Repulsive Forces / Feedback ---
    if indata('haptic_rep_force'):
        fig9_data = [
            {'sim': data['haptic_rep_force'][:, 0], 'ref': 0.0},
            {'sim': data['haptic_rep_force'][:, 1], 'ref': 0.0},
            {'sim': data['haptic_rep_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig9_data, 
               ["Repulsive Force X (Distance limit $r_{min}$) [N]", 
                "Repulsive Force Y (Horiz. FoV) [N]", 
                "Repulsive Force Z (Vert. FoV) [N]"], 
               "Haptic APF Repulsive Forces (Safety Constraints)", 
               ncols=3, use_tex=args.tex, block=block, fignum=10, 
               task_start=task_start, task_end=task_end)
    elif indata('haptic_force'):
        fig9_data = [
            {'sim': data['haptic_force'][:, 0], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 1], 'ref': 0.0},
            {'sim': data['haptic_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig9_data, ["Force X (Zoom) [N]", "Force Y (Pan) [N]", "Force Z (Altitude) [N]"], 
               "Haptic Feedback Forces Transmitted to haptic device", ncols=3, use_tex=args.tex, block=block, fignum=10, task_start=task_start, task_end=task_end)

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
           "Drone Linear and Angular Accelerations", ncols=3, use_tex=args.tex, block=block, fignum=11, task_start=task_start, task_end=task_end)

    # --- FIGURE 11: Peg External Forces ---
    if indata('peg_ext_force'):
        fig11_data = [
            {'sim': data['peg_ext_force'][:, 0], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 1], 'ref': 0.0},
            {'sim': data['peg_ext_force'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig11_data, 
               ["Force X (Sensor) [N]", "Force Y (Sensor) [N]", "Force Z (Sensor) [N]"], 
               "Peg External Contact Forces (FT Sensor)", ncols=3, use_tex=args.tex, block=block, fignum=12, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

    # --- FIGURE 11b: Peg External Torques ---
    if indata('peg_ext_torque'):
        fig11b_data = [
            {'sim': data['peg_ext_torque'][:, 0], 'ref': 0.0},
            {'sim': data['peg_ext_torque'][:, 1], 'ref': 0.0},
            {'sim': data['peg_ext_torque'][:, 2], 'ref': 0.0}
        ]
        myPlot(t, fig11b_data, 
               ["Torque X (Sensor) [Nm]", "Torque Y (Sensor) [Nm]", "Torque Z (Sensor) [Nm]"], 
               "Peg External Contact Torques (FT Sensor)", ncols=3, use_tex=args.tex, block=block, fignum=121, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

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
               ncols=2, use_tex=args.tex, block=block, fignum=13, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

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
               ncols=2, use_tex=args.tex, block=block, fignum=131, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

    # --- FIGURE 12c: Admittance delta_yaw (rotazione di ammettenza) ---
    if indata('delta_yaw'):
        dyaw_rad = data['delta_yaw']
        dyaw_deg = np.degrees(dyaw_rad)
        fig12c_data = [
            {'sim': dyaw_rad, 'ref': 0.0},
            {'sim': dyaw_deg, 'ref': 0.0}
        ]
        myPlot(t, fig12c_data,
               [r"$\Delta\psi$ [rad]", r"$\Delta\psi$ [deg]"],
               "Admittance Yaw Displacement $\\Delta\\psi$",
               ncols=2, use_tex=args.tex, block=block, fignum=132, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

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
        if haptic_peg_intervals:
            for idx_h, item in enumerate(haptic_peg_intervals):
                t_h_s, t_h_e = item[0], item[1]
                ax13[0].axvspan(t_h_s, t_h_e, color='#a855f7', alpha=0.15, label='Haptic Interaction' if idx_h == 0 else None, zorder=0)
                ax13[1].axvspan(t_h_s, t_h_e, color='#a855f7', alpha=0.15, label='Haptic Interaction' if idx_h == 0 else None, zorder=0)
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

    # --- FIGURE 13b: Confronto delta_yaw vs tau_yaw (Sensor X) ---
    if indata('delta_yaw') and indata('peg_ext_torque'):
        tau_x = data['peg_ext_torque'][:, 0]
        dyaw_deg = np.degrees(data['delta_yaw'])
        fig13b, ax13b = plt.subplots(2, 1, figsize=(12, 6), sharex=True, num=133)
        try:
            fig13b.canvas.manager.set_window_title("Figure 13b: Admittance Yaw vs Contact Torque")
        except Exception:
            pass
        if haptic_peg_intervals:
            for idx_h, item in enumerate(haptic_peg_intervals):
                t_h_s, t_h_e = item[0], item[1]
                ax13b[0].axvspan(t_h_s, t_h_e, color='#a855f7', alpha=0.15, label='Haptic Interaction' if idx_h == 0 else None, zorder=0)
                ax13b[1].axvspan(t_h_s, t_h_e, color='#a855f7', alpha=0.15, label='Haptic Interaction' if idx_h == 0 else None, zorder=0)
        ax13b[0].plot(t, tau_x, 'm-', linewidth=1.5, label=r'$\tau_x$ [Nm]')
        ax13b[0].set_ylabel(r'$\tau_x$ (Sensor X / Yaw) [Nm]')
        ax13b[0].legend(loc='upper right')
        ax13b[0].grid(True, alpha=0.3)
        ax13b[0].set_title(r"Contact Torque $\tau_x$ (Sensor X / Drone Yaw axis)")
        ax13b[1].plot(t, dyaw_deg, 'c-', linewidth=1.5, label=r'$\Delta\psi$ [deg]')
        ax13b[1].set_xlabel('Time [s]')
        ax13b[1].set_ylabel(r'$\Delta\psi$ [deg]')
        ax13b[1].legend(loc='upper right')
        ax13b[1].grid(True, alpha=0.3)
        ax13b[1].set_title(r"Admittance Yaw Deviation $\Delta\psi$")
        fig13b.suptitle(r"Rotational Admittance Effect: Contact Torque $\tau_x$ vs $\Delta\psi$", fontsize=14)
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
               ncols=2, use_tex=args.tex, block=block, fignum=14, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

    # --- FIGURE 15: Interaction Drone Velocities (ENU) + Omega Z ---
    has_peg_vel     = indata('peg_actual_vel')
    has_peg_omega_z = indata('peg_actual_omega_z') or indata('peg_actual_yaw_rate')
    if has_peg_vel or has_peg_omega_z:
        fig15_data, labels15 = [], []
        peg_ref_vel     = data['peg_ref_vel']     if indata('peg_ref_vel')     else None
        peg_ref_omega_z = data['peg_ref_omega_z'] if indata('peg_ref_omega_z') else (data['peg_ref_yaw_rate'] if indata('peg_ref_yaw_rate') else None)
        if has_peg_vel:
            peg_vel = data['peg_actual_vel']
            fig15_data += [
                {'sim': peg_vel[:, 0], 'ref': peg_ref_vel[:, 0] if peg_ref_vel is not None else None},
                {'sim': peg_vel[:, 1], 'ref': peg_ref_vel[:, 1] if peg_ref_vel is not None else None},
                {'sim': peg_vel[:, 2], 'ref': peg_ref_vel[:, 2] if peg_ref_vel is not None else None},
            ]
            labels15 += ["Vel X [m/s]", "Vel Y [m/s]", "Vel Z [m/s]"]
        if has_peg_omega_z:
            actual_omega_z = data['peg_actual_omega_z'] if indata('peg_actual_omega_z') else data['peg_actual_yaw_rate']
            fig15_data.append({'sim': actual_omega_z, 'ref': peg_ref_omega_z})
            labels15.append("Omega Z [rad/s]")
        myPlot(t, fig15_data, labels15,
               "Interaction Drone Velocities (ENU) and Angular Velocity Omega Z",
               ncols=2, use_tex=args.tex, block=block, fignum=15, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

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
                "Estimated Wrench (Momentum-Based Estimator)", ncols=3, use_tex=args.tex, block=block, fignum=16, task_start=task_start, task_end=task_end, haptic_intervals=haptic_peg_intervals)

    # --- FIGURE 17: Violazioni dei vincoli di sicurezza (Distanza Minima e FoV Camera) ---
    if indata('r_cyl'):
        r_min = 1.0      # [m] distanza minima di sicurezza da target
        s_r = np.maximum(0.0, r_min - data['r_cyl'])

        # Parametri FoV camera (default standard OakD-Lite: 68.98° H, 51.74° V)
        fov_h_deg = float(data['fov_h']) if indata('fov_h') else 68.98
        fov_v_deg = float(data['fov_v']) if indata('fov_v') else 51.74
        fov_h_half_deg = fov_h_deg / 2.0
        fov_v_half_deg = fov_v_deg / 2.0

        # 1. Violazione FoV Orizzontale: deviazione angolare yaw oltre la semi-apertura orizzontale
        yaw_err_deg = np.degrees(np.abs(data['yaw_err_cyl'])) if indata('yaw_err_cyl') else np.zeros_like(s_r)
        s_fov_h = np.maximum(0.0, yaw_err_deg - fov_h_half_deg)

        # 2. Violazione FoV Verticale: elevazione theta_v oltre la semi-apertura verticale
        if indata('z_cyl'):
            r_safe = np.where(data['r_cyl'] > 1e-4, data['r_cyl'], 1e-4)
            theta_v_deg = np.degrees(np.arctan2(np.abs(data['z_cyl']), r_safe))
            s_fov_v = np.maximum(0.0, theta_v_deg - fov_v_half_deg)
        else:
            s_fov_v = np.zeros_like(s_r)

        fig17_data = [
            {'sim': s_r,     'ref': 0.0},
            {'sim': s_fov_h, 'ref': 0.0},
            {'sim': s_fov_v, 'ref': 0.0},
        ]
        labels17 = [
            f"Distance Violation $s_r$ [m] ($r_{{min}}$={r_min:.1f} m)",
            f"Horiz. FoV Violation $s_{{fov,h}}$ [deg] (half-FoV={fov_h_half_deg:.0f}°)",
            f"Vert. FoV Violation $s_{{fov,v}}$ [deg] (half-FoV={fov_v_half_deg:.0f}°)"
        ]
        myPlot(t, fig17_data, labels17,
               "Safety Distance and Camera FoV Constraint Violations",
               ncols=3, use_tex=args.tex, block=block, fignum=17,
               task_start=task_start, task_end=task_end)


    # --- GENERAZIONE REPORT METRICHE COMPLETO (Benchmark, Jitter, Tracking) ---
    report_lines = []
    report_lines.append("=" * 70)
    report_lines.append("             EXPERIMENT METRICS & BENCHMARK REPORT")
    report_lines.append("=" * 70)
    report_lines.append(f"File log          : {os.path.abspath(args.log)}")
    report_lines.append(f"Campioni totali   : {len(t)}")
    report_lines.append(f"Durata totale     : {t[-1]:.2f} s")
    if task_start > 0:
        t_end_m = task_end if task_end > 0 else t[-1]
        report_lines.append(f"Tempo missione    : {t_end_m - task_start:.2f} s (da t={task_start:.2f}s a t={t_end_m:.2f}s)")

    # 1. SAMPLING & CLOCK JITTER
    dt = np.diff(t)
    if len(dt) > 0:
        dt_ms = dt * 1e3
        dt_mean = float(np.mean(dt_ms))
        dt_std = float(np.std(dt_ms))
        dt_min = float(np.min(dt_ms))
        dt_max = float(np.max(dt_ms))
        freq_mean = 1000.0 / dt_mean if dt_mean > 0 else 0.0

        report_lines.append("\n" + "-" * 70)
        report_lines.append(" 1. SAMPLING PERIOD & REAL-TIME CLOCK JITTER (delta_t)")
        report_lines.append("-" * 70)
        report_lines.append(f"Frequenza media nominale : {freq_mean:.2f} Hz")
        report_lines.append(f"Delta_t medio            : {dt_mean:.2f} ms")
        report_lines.append(f"Jitter Campionamento (Std): {dt_std:.2f} ms")
        report_lines.append(f"Delta_t Min / Max        : {dt_min:.2f} ms / {dt_max:.2f} ms")

    # 2. MPC SOLVER TIMING & JITTER
    if indata('solve_time'):
        st_raw = np.asarray(data['solve_time'])
        if task_start > 0:
            mask = (t >= task_start) & (st_raw > 0)
            if task_end > 0:
                mask &= (t <= task_end)
            st_valid = st_raw[mask] if np.any(mask) else st_raw[st_raw > 0]
        else:
            st_valid = st_raw[st_raw > 0]

        if len(st_valid) > 0:
            mean_st = float(np.mean(st_valid))
            std_st = float(np.std(st_valid))
            min_st = float(np.min(st_valid))
            max_st = float(np.max(st_valid))
            p95_st = float(np.percentile(st_valid, 95))
            p99_st = float(np.percentile(st_valid, 99))
            st_step_diff = float(np.mean(np.abs(np.diff(st_valid)))) if len(st_valid) > 1 else 0.0
            ts_ms = 10.0  # Periodo nominale Ts = 10 ms (100 Hz)
            overruns = int(np.sum(st_valid > ts_ms))
            overrun_pct = (overruns / len(st_valid)) * 100.0
            cpu_usage_pct = (mean_st / ts_ms) * 100.0

            report_lines.append("\n" + "-" * 70)
            report_lines.append(" 2. MPC SOLVER PERFORMANCE & TIMING JITTER")
            report_lines.append("-" * 70)
            report_lines.append(f"Campioni solve analizzati: {len(st_valid)}")
            report_lines.append(f"Periodo nominale (Ts)    : {ts_ms:.1f} ms (100 Hz)")
            report_lines.append(f"Tempo Medio Solve        : {mean_st:.2f} ms  (Carico medio CPU: {cpu_usage_pct:.1f}%)")
            report_lines.append(f"Deviazione Standard (Jit): {std_st:.2f} ms")
            report_lines.append(f"Minimo / Massimo         : {min_st:.2f} ms / {max_st:.2f} ms")
            report_lines.append(f"Step-to-Step Jitter      : {st_step_diff:.2f} ms (variazione media passo-passo)")
            report_lines.append(f"Percentile 95%           : {p95_st:.2f} ms")
            report_lines.append(f"Percentile 99%           : {p99_st:.2f} ms")
            report_lines.append(f"Overrun (> {ts_ms:.0f} ms)         : {overruns} / {len(st_valid)} ({overrun_pct:.2f}%)")

            # --- FIGURE 19: MPC Solve Time ---
            fig19_data = [
                {'sim': st_raw, 'ref': ts_ms}
            ]
            myPlot(t, fig19_data,
                   [f"Solve Time [ms] (Mean: {mean_st:.2f} ms, Max: {max_st:.2f} ms, Ts limit: {ts_ms:.0f} ms)"],
                   "MPC Solver Execution Time & Timing Jitter", ncols=1, use_tex=args.tex, block=block, fignum=19, task_start=task_start, task_end=task_end)

    # 3. TRACKING PERFORMANCE (in Missione)
    if indata('r_cyl') and indata('online_cyl_ref'):
        mask_trk = np.ones(len(t), dtype=bool)
        if task_start > 0:
            mask_trk &= (t >= task_start)
        if task_end > 0:
            mask_trk &= (t <= task_end)

        if np.any(mask_trk):
            rms_r = float(np.sqrt(np.nanmean(err_r[mask_trk]**2)))
            max_r = float(np.nanmax(err_r[mask_trk]))
            rms_beta = float(np.sqrt(np.nanmean(err_beta[mask_trk]**2)))
            max_beta = float(np.nanmax(err_beta[mask_trk]))
            rms_z = float(np.sqrt(np.nanmean(err_z[mask_trk]**2)))
            max_z = float(np.nanmax(err_z[mask_trk]))
            rms_yaw = float(np.sqrt(np.nanmean(err_yaw[mask_trk]**2)))
            max_yaw = float(np.nanmax(err_yaw[mask_trk]))

            report_lines.append("\n" + "-" * 70)
            report_lines.append(" 3. TRACKING PERFORMANCE SUMMARY (in Missione)")
            report_lines.append("-" * 70)
            report_lines.append(f"Errore Raggio (r_cyl)    : RMS = {rms_r:.4f} m | Max = {max_r:.4f} m")
            report_lines.append(f"Errore Azimut (beta)     : RMS = {rms_beta:.4f} rad ({np.degrees(rms_beta):.2f} deg) | Max = {max_beta:.4f} rad")
            report_lines.append(f"Errore Quota (z_cyl)     : RMS = {rms_z:.4f} m | Max = {max_z:.4f} m")
            report_lines.append(f"Errore Yaw               : RMS = {rms_yaw:.4f} rad ({np.degrees(rms_yaw):.2f} deg) | Max = {max_yaw:.4f} rad")

    # 4. CONSTRAINT VIOLATIONS SUMMARY (Distance & FoV)
    if indata('r_cyl'):
        mask_c = np.ones(len(t), dtype=bool)
        if task_start > 0:
            mask_c &= (t >= task_start)
        if task_end > 0:
            mask_c &= (t <= task_end)

        if np.any(mask_c):
            max_s_r = float(np.nanmax(s_r[mask_c]))
            max_s_h = float(np.nanmax(s_fov_h[mask_c]))
            max_s_v = float(np.nanmax(s_fov_v[mask_c]))

            report_lines.append("\n" + "-" * 70)
            report_lines.append(" 4. CONSTRAINT VIOLATIONS SUMMARY (in Mission)")
            report_lines.append("-" * 70)
            report_lines.append(f"Violazione Distanza Minima (r_min={r_min:.1f}m)  : Max = {max_s_r:.4f} m" + (" (NESSUNA)" if max_s_r <= 1e-4 else " (VIOLATO!)"))
            report_lines.append(f"Violazione FoV Orizzontale (lim={fov_h_half_deg:.0f}°) : Max = {max_s_h:.2f} deg" + (" (NESSUNA)" if max_s_h <= 1e-2 else " (VIOLATO!)"))
            report_lines.append(f"Violazione FoV Verticale   (lim={fov_v_half_deg:.0f}°) : Max = {max_s_v:.2f} deg" + (" (NESSUNA)" if max_s_v <= 1e-2 else " (VIOLATO!)"))
    report_lines.append("=" * 70 + "\n")

    report_text = "\n".join(report_lines)
    print(report_text)

    if args.save:
        if args.out_dir != "." and not os.path.exists(args.out_dir):
            os.makedirs(args.out_dir)

        # Salvataggio del report metriche su file TXT
        report_txt_path = os.path.join(args.out_dir, "metrics_report.txt")
        try:
            with open(report_txt_path, "w") as f:
                f.write(report_text)
            print(f"Report metriche salvato in: {report_txt_path}")
        except Exception as e:
            print(f"Errore nel salvataggio di metrics_report.txt: {e}")

        # Opzioni di salvataggio per formato vettoriale (pdf/eps): dpi alto, niente trasparenza
        fmt_opts = {
            "png": {"dpi": 150},
            "pdf": {"dpi": 300, "bbox_inches": "tight"},
            "eps": {"dpi": 300, "bbox_inches": "tight", "format": "eps"},
        }
        fig_nums = plt.get_fignums()
        for i in fig_nums:
            for fmt in args.formats:
                sub_dir = os.path.join(args.out_dir, fmt)
                os.makedirs(sub_dir, exist_ok=True)
                out_path = os.path.join(sub_dir, f"plot_fig_{i}.{fmt}")
                plt.figure(i).savefig(out_path, **fmt_opts[fmt])

        # --- PDF multi-pagina: tutti i grafici in un unico file scrollabile nella cartella madre ---
        from matplotlib.backends.backend_pdf import PdfPages
        multipage_path = os.path.join(args.out_dir, "all_figures.pdf")
        with PdfPages(multipage_path) as pdf:
            for i in fig_nums:
                pdf.savefig(plt.figure(i), bbox_inches="tight")
        print(f"Grafici salvati in: {os.path.abspath(args.out_dir)} | Formati: {args.formats}")
        for fmt in args.formats:
            print(f"  - Sottocartella '{fmt}/': {os.path.join(os.path.abspath(args.out_dir), fmt)}")
        print(f"  - PDF multi-pagina (cartella madre): {multipage_path}")
        print(f"  - Report metriche  (cartella madre): {report_txt_path}")
    elif args.all:
        plt.show()

if __name__ == "__main__":
    main()
