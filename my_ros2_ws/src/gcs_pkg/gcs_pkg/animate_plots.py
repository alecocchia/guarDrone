#!/usr/bin/env python3
"""
animate_plots.py
Generatore avanzato di animazioni video MP4 per telemetria di volo e teleoperazione haptic di guarDrone.

Caratteristiche principali:
1. Schede singole individuali 2:1 (~720x360 px) per ciascuna grandezza chiave:
   - r_cyl: Raggio cilindrico con soglie r_min e r_act
   - beta_cyl: Angolo azimutale (unwrapped per continuita')
   - z_cyl: Quota cilindrica con coni visivi FoV (+/- z_max)
   - yaw: Tracking angolo di imbardata (effettivo vs desiderato puntamento)
   - fx_apf: Forza repulsiva radiale F_x^rep (APF)
   - fy_apf: Forza repulsiva tangenziale F_y^rep (APF)
   - fz_apf: Forza repulsiva verticale F_z^rep (FoV APF)
   - fnorm_apf: Norma complessiva delle forze repulsive ||F_rep||

2. Modalita' di visualizzazione:
   - 'rolling': Finestra temporale scorrevole (15s) con auto-scaling dinamico continuo dell'asse Y.
   - 'full': Timeline fissa sull'intera durata con progressione temporale da sinistra a destra.
   - 'both': Genera entrambe le varianti per flessibilita' di montaggio video.
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from scipy.io import loadmat
from concurrent.futures import ProcessPoolExecutor

# Configurazione stile grafico elegante LaTeX / Computer Modern
plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "axes.grid": True,
    "grid.alpha": 0.40,
    "grid.linestyle": "--",
    "grid.color": "#94a3b8"
})


def wrap_pi(angles):
    """Normalizza gli angoli nell'intervallo [-pi, pi]."""
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def load_dataset(mat_path):
    """Carica il dataset dal file .mat ed estrae i segnali primari."""
    raw = loadmat(mat_path, squeeze_me=True)
    data = {k: v for k, v in raw.items() if not k.startswith('__')}

    t = np.asarray(data['t'], dtype=float)
    r_min = float(data.get('r_min', data.get('r_min_safety', 1.5)))
    r_act = 2.0 * r_min

    fov_h = float(data.get('fov_h', 69.0))
    fov_v = float(data.get('fov_v', 54.5 if fov_h < 75.0 else 42.0))

    # 1. Raggio cilindrico
    r_sim = np.asarray(data['r_cyl'], dtype=float)
    r_ref = np.asarray(data['online_cyl_ref'][:, 0], dtype=float)

    # 2. Beta cilindrico (unwrapped per continuita' visiva senza salti di 2*pi)
    beta_sim = np.unwrap(data['beta_cyl'])
    beta_diff = (data['online_cyl_ref'][:, 1] - data['beta_cyl'] + np.pi) % (2.0 * np.pi) - np.pi
    beta_ref = beta_sim + beta_diff

    # 3. Quota cilindrica e coni FoV verticali
    z_sim = np.asarray(data['z_cyl'], dtype=float)
    z_ref = np.asarray(data['online_cyl_ref'][:, 2], dtype=float)
    z_max = r_sim * np.tan(np.radians(fov_v / 2.0))

    # 4. Tracking Yaw
    cyl_ref = np.asarray(data['online_cyl_ref'])
    yaw_offset = cyl_ref[:, 3] if cyl_ref.ndim == 2 and cyl_ref.shape[1] > 3 else 0.0
    yaw_sim = wrap_pi(data['rpy'][:, 2]) if 'rpy' in data else np.zeros_like(t)
    yaw_des = wrap_pi(data['beta_cyl'] + np.pi + yaw_offset)

    # 5. Forze repulsive APF
    f_rep = np.asarray(data.get('haptic_rep_force', np.zeros((len(t), 3))), dtype=float)
    fx_rep = f_rep[:, 0]
    fy_rep = f_rep[:, 1]
    fz_rep = f_rep[:, 2]
    fnorm_rep = np.linalg.norm(f_rep, axis=1)

    # 6. Intervalli operativi: NMPC attivo (task_start a task_end) e Haptic attivo
    mpc_intervals = []
    task_start = float(data.get('task_start_time', -1.0))
    task_end = float(data.get('task_end_time', -1.0))
    if task_start > 0:
        t_end_val = task_end if (task_end > task_start and task_end < float(t[-1])) else float(t[-1])
        mpc_intervals.append((task_start, t_end_val))

    haptic_intervals = []
    h_gd_key = 'haptic_guardrone_active' if 'haptic_guardrone_active' in data else 'haptic_active'
    if h_gd_key in data:
        h_act = np.asarray(data[h_gd_key]).flat
        in_haptic = False
        t_h_start = 0.0
        for idx_h, val in enumerate(h_act):
            if val > 0.5 and not in_haptic:
                in_haptic = True
                t_h_start = float(t[idx_h])
            elif val <= 0.5 and in_haptic:
                in_haptic = False
                haptic_intervals.append((t_h_start, float(t[idx_h])))
        if in_haptic:
            haptic_intervals.append((t_h_start, float(t[-1])))

    return {
        't': t, 'r_min': r_min, 'r_act': r_act, 'fov_v': fov_v,
        'r_sim': r_sim, 'r_ref': r_ref,
        'beta_sim': beta_sim, 'beta_ref': beta_ref,
        'z_sim': z_sim, 'z_ref': z_ref, 'z_max': z_max,
        'yaw_sim': yaw_sim, 'yaw_des': yaw_des,
        'fx_rep': fx_rep, 'fy_rep': fy_rep, 'fz_rep': fz_rep,
        'fnorm_rep': fnorm_rep,
        'mpc_intervals': mpc_intervals,
        'haptic_intervals': haptic_intervals
    }


# Dizionario di configurazione per ciascuna grandezza singola
VARIABLE_CONFIGS = {
    'r_cyl': {
        'sim': 'r_sim', 'ref': 'r_ref',
        'ylabel': r"$r_{\mathrm{cyl}}\ [\mathrm{m}]$",
        'sim_color': '#1f77b4', 'ref_color': '#d62728',
        'sim_label': 'Actual', 'ref_label': 'Ref',
        'static_lines': [
            {'y': 'r_act', 'color': '#b45309', 'style': ':', 'label': r'$r_{\mathrm{act}}$'},
            {'y': 'r_min', 'color': '#0f172a', 'style': '-.', 'label': r'$r_{\min}$'}
        ],
        'dynamic_lines': []
    },
    'beta_cyl': {
        'sim': 'beta_sim', 'ref': 'beta_ref',
        'ylabel': r"$\beta\ [\mathrm{rad}]$",
        'sim_color': '#1f77b4', 'ref_color': '#d62728',
        'sim_label': 'Actual', 'ref_label': 'Ref',
        'static_lines': [],
        'dynamic_lines': []
    },
    'z_cyl': {
        'sim': 'z_sim', 'ref': 'z_ref',
        'ylabel': r"$z_{\mathrm{cyl}}\ [\mathrm{m}]$",
        'sim_color': '#1f77b4', 'ref_color': '#d62728',
        'sim_label': 'Actual', 'ref_label': 'Ref',
        'static_lines': [],
        'dynamic_lines': [
            {'data': 'z_max', 'color': '#b45309', 'style': ':', 'label': r'$\pm z_{\max}$'},
            {'data': '-z_max', 'color': '#b45309', 'style': ':', 'label': None}
        ]
    },
    'yaw': {
        'sim': 'yaw_sim', 'ref': 'yaw_des',
        'ylabel': r"$\psi\ [\mathrm{rad}]$",
        'sim_color': '#1f77b4', 'ref_color': '#d62728',
        'sim_label': 'Actual', 'ref_label': 'Desired',
        'static_lines': [],
        'dynamic_lines': []
    },
    'fx_apf': {
        'sim': 'fx_rep', 'ref': None,
        'ylabel': r"$F_x^{\mathrm{rep}}\ [\mathrm{N}]$",
        'sim_color': '#b45309', 'sim_label': r'$F_x^{\mathrm{rep}}$',
        'static_lines': [
            {'y': 0.0, 'color': '#94a3b8', 'style': '--', 'label': None},
            {'y': 10.0, 'color': '#ef4444', 'style': ':', 'label': 'Sat.'}
        ],
        'dynamic_lines': []
    },
    'fy_apf': {
        'sim': 'fy_rep', 'ref': None,
        'ylabel': r"$F_y^{\mathrm{rep}}\ [\mathrm{N}]$",
        'sim_color': '#0284c7', 'sim_label': r'$F_y^{\mathrm{rep}}$',
        'static_lines': [
            {'y': 0.0, 'color': '#94a3b8', 'style': '--', 'label': None}
        ],
        'dynamic_lines': []
    },
    'fz_apf': {
        'sim': 'fz_rep', 'ref': None,
        'ylabel': r"$F_z^{\mathrm{rep}}\ [\mathrm{N}]$",
        'sim_color': '#7c3aed', 'sim_label': r'$F_z^{\mathrm{rep}}$',
        'static_lines': [
            {'y': 0.0, 'color': '#94a3b8', 'style': '--', 'label': None},
            {'y': 10.0, 'color': '#ef4444', 'style': ':', 'label': 'Sat.'},
            {'y': -10.0, 'color': '#ef4444', 'style': ':', 'label': None}
        ],
        'dynamic_lines': []
    },
    'fnorm_apf': {
        'sim': 'fnorm_rep', 'ref': None,
        'ylabel': r"$\|\mathbf{F}_{\mathrm{rep}}\|\ [\mathrm{N}]$",
        'sim_color': '#dc2626', 'sim_label': r'$\|\mathbf{F}_{\mathrm{rep}}\|$',
        'static_lines': [
            {'y': 0.0, 'color': '#94a3b8', 'style': '--', 'label': None},
            {'y': 10.0, 'color': '#ef4444', 'style': ':', 'label': 'Sat.'}
        ],
        'dynamic_lines': []
    }
}


def render_single_card(signals, var_key, out_path, view_mode="rolling", window=15.0, fps=30):
    """
    Genera il video MP4 di una singola grandezza in formato card compatto 2:1 (~720x360 px).
    
    Parametri:
    - signals: dizionario con i dati estratti dal file .mat
    - var_key: chiave della grandezza (es. 'r_cyl', 'yaw', 'fx_apf')
    - out_path: percorso del file .mp4 di output
    - view_mode: 'rolling' (finestra 15s con auto-scaling Y continuo) o 'full' (timeline fissa globale)
    - window: durata della finestra scorrevole [s]
    - fps: fotogrammi al secondo
    """
    cfg = VARIABLE_CONFIGS[var_key]
    t = signals['t']
    y_sim = signals[cfg['sim']]
    y_ref = signals[cfg['ref']] if cfg.get('ref') else None

    # Setup figura compatto formato card (~2:1)
    fig, ax = plt.subplots(figsize=(4.8, 2.4))
    fig.patch.set_facecolor('#ffffff')

    for spine in ax.spines.values():
        spine.set_color('#cbd5e1')
        spine.set_linewidth(0.8)
    ax.tick_params(colors='#334155', labelsize=8)
    ax.set_ylabel(cfg['ylabel'], fontsize=9, color='#0f172a')
    ax.set_xlabel("Time [s]", fontsize=8.5, color='#334155')

    # Tracciamento linee statiche di soglia (es. r_min, r_act, saturazione)
    static_y_vals = []
    for sl in cfg.get('static_lines', []):
        val = signals[sl['y']] if isinstance(sl['y'], str) else sl['y']
        static_y_vals.append(val)
        ax.axhline(val, color=sl['color'], linestyle=sl['style'],
                   linewidth=1.1, label=sl.get('label'))

    # Linee dinamiche (es. +/- z_max)
    dyn_artists = []
    for dl in cfg.get('dynamic_lines', []):
        arr = -signals[dl['data'][1:]] if dl['data'].startswith('-') else signals[dl['data']]
        line_dyn, = ax.plot([], [], color=dl['color'], linestyle=dl['style'],
                            linewidth=1.1, label=dl.get('label'))
        dyn_artists.append((line_dyn, arr))

    # Curve principali: riferimento ed effettivo con cursore circolare (head)
    line_ref = None
    if y_ref is not None:
        line_ref, = ax.plot([], [], color=cfg['ref_color'], linestyle='--',
                            linewidth=1.3, label=cfg.get('ref_label', 'Ref'))
    line_sim, = ax.plot([], [], color=cfg['sim_color'], linewidth=1.8,
                        label=cfg.get('sim_label', 'Actual'))
    head_sim, = ax.plot([], [], marker='o', markersize=4.5, color=cfg['sim_color'])

    # Ombreggiatura intervalli operativi: NMPC attivo (giallo) e Haptic attivo (rosso)
    for idx_m, (t_m_s, t_m_e) in enumerate(signals.get('mpc_intervals', [])):
        ax.axvspan(t_m_s, t_m_e, color='#facc15', alpha=0.12,
                   label='MPC' if idx_m == 0 else None, zorder=0)

    for idx_h, (t_h_s, t_h_e) in enumerate(signals.get('haptic_intervals', [])):
        ax.axvspan(t_h_s, t_h_e, color='#ef4444', alpha=0.18,
                   label='Haptic' if idx_h == 0 else None, zorder=0)

    # Legenda ordinata compatta a 2 colonne se ci sono molti elementi
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ncol = 2 if len(labels) > 3 else 1
        ax.legend(loc='upper right', ncol=ncol, fontsize=7.0, framealpha=0.9, edgecolor='#cbd5e1')

    # Calcolo escursione globale e limite di zoom massimo (ampiezza minima Y = 35% del range globale)
    all_global_vals = [y_sim]
    if y_ref is not None:
        all_global_vals.append(y_ref)
    for _, arr in dyn_artists:
        all_global_vals.append(arr)
    for val in static_y_vals:
        all_global_vals.append(np.array([val]))

    global_concat = np.concatenate(all_global_vals)
    global_ymin = float(np.nanmin(global_concat))
    global_ymax = float(np.nanmax(global_concat))
    global_span = max(global_ymax - global_ymin, 0.5)

    # Limite di zoom massimo: l'ampiezza visibile non puo' mai scendere sotto il 35% del range globale
    min_span = max(0.35 * global_span, 0.4)

    # Configurazione limiti per modalita' full timeline
    if view_mode == "full":
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(global_ymin - 0.15 * global_span, global_ymax + 0.15 * global_span)

    plt.tight_layout(pad=0.7)

    # Campionamento temporale dei frame
    duration = t[-1] - t[0]
    total_frames = int(duration * fps)
    t_interp = np.linspace(t[0], t[-1], total_frames)

    def update(frame_idx):
        cur_t = t_interp[frame_idx]
        idx = max(1, np.searchsorted(t, cur_t))

        line_sim.set_data(t[:idx], y_sim[:idx])
        head_sim.set_data([t[idx - 1]], [y_sim[idx - 1]])
        if line_ref is not None:
            line_ref.set_data(t[:idx], y_ref[:idx])
        for ld, arr in dyn_artists:
            ld.set_data(t[:idx], arr[:idx])

        # Auto-scaling dinamico per modalita' rolling window con zoom massimo controllato
        if view_mode == "rolling":
            # 1. Asse X scorrevole ultimi W secondi
            if cur_t <= t[0] + window:
                ax.set_xlim(t[0], t[0] + window)
                idx_w = 0
            else:
                ax.set_xlim(cur_t - window, cur_t)
                idx_w = np.searchsorted(t, cur_t - window)

            # 2. Asse Y auto-adattivo sui valori visibili, vincolato a min_span per evitare iper-zoom
            vals = [y_sim[idx_w:idx]]
            if y_ref is not None:
                vals.append(y_ref[idx_w:idx])
            for _, arr in dyn_artists:
                vals.append(arr[idx_w:idx])

            concat = np.concatenate(vals)
            loc_ymin = float(np.nanmin(concat))
            loc_ymax = float(np.nanmax(concat))
            loc_span = max(loc_ymax - loc_ymin, 1e-4)

            # Applica il limite di zoom massimo (min_span = 35% del range globale)
            eff_span = max(loc_span, min_span)
            loc_center = (loc_ymin + loc_ymax) / 2.0

            half_span = (eff_span * 1.18) / 2.0
            ax.set_ylim(loc_center - half_span, loc_center + half_span)

    writer = anim.FFMpegWriter(fps=fps, bitrate=2200, extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p'])
    animation = anim.FuncAnimation(fig, update, frames=total_frames, blit=False)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    animation.save(out_path, writer=writer, dpi=150)
    plt.close(fig)
    print(f"[OK] {os.path.basename(out_path)} ({view_mode}, {duration:.1f}s)")


def _worker_render(args_tuple):
    """Wrapper per esecuzione parallela multi-core."""
    signals, var_key, out_path, vmode, window, fps = args_tuple
    render_single_card(signals, var_key, out_path, view_mode=vmode, window=window, fps=fps)


# =============================================================================
# CLI PRINCIPALE
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="Generatore avanzato di animazioni video MP4 per telemetria guarDrone.")
    parser.add_argument("--log", type=str, required=True, help="Percorso del file .mat (es. sim_run.mat o hw_run.mat)")
    parser.add_argument("--var", type=str, choices=list(VARIABLE_CONFIGS.keys()) + ["all"], default="all",
                        help="Grandezza singola da animare (default: all)")
    parser.add_argument("--view-mode", type=str, choices=["rolling", "full", "both"], default="both",
                        help="Modalita' visuale: 'rolling' (finestra 15s con Y dinamico), 'full' (timeline fissa), o 'both'")
    parser.add_argument("--window", type=float, default=15.0, help="Larghezza della finestra temporale [s] (default: 15.0)")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate video (default: 30)")
    parser.add_argument("--parallel", type=int, default=4, help="Numero di processi paralleli per rendering rapido (default: 4)")
    parser.add_argument("--force", action="store_true", help="Forza la rigenerazione sovrascrivendo i video gia' esistenti")
    parser.add_argument("--out-dir", type=str, default=None, help="Cartella di output personalizzata per i video")
    args = parser.parse_args()

    signals = load_dataset(args.log)
    out_dir = args.out_dir if args.out_dir else os.path.join(os.path.dirname(os.path.abspath(args.log)), "videos", "single")
    os.makedirs(out_dir, exist_ok=True)

    target_vars = list(VARIABLE_CONFIGS.keys()) if args.var == "all" else [args.var]
    view_modes = ["rolling", "full"] if args.view_mode == "both" else [args.view_mode]

    # Prepara la lista di compiti da renderizzare (salta quelli gia' presenti su disco)
    tasks = []
    skipped = 0
    for var_key in target_vars:
        for vmode in view_modes:
            fname = f"{var_key}_{vmode}.mp4"
            out_path = os.path.join(out_dir, fname)
            if not args.force and os.path.exists(out_path) and os.path.getsize(out_path) > 100 * 1024:
                skipped += 1
                continue
            tasks.append((signals, var_key, out_path, vmode, args.window, args.fps))

    if skipped > 0:
        print(f"[RESUME] Saltati {skipped} video gia' presenti e completati.")

    if not tasks:
        print(f"[COMPLETO] Tutti i video per questo dataset sono gia' presenti in: {os.path.abspath(out_dir)}")
        return

    print(f"--- Avvio generazione di {len(tasks)} video animati (parallelismo={args.parallel}) ---")
    print(f"Dataset: {os.path.abspath(args.log)}")
    print(f"Output:  {os.path.abspath(out_dir)}\n")

    if args.parallel > 1 and len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=args.parallel) as executor:
            list(executor.map(_worker_render, tasks))
    else:
        for t in tasks:
            _worker_render(t)

    print(f"\n[COMPLETATO] Tutti i video generati con successo in: {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main()
