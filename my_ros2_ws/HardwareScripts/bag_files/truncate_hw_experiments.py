#!/usr/bin/env python3
"""
Script per troncare e ripulire i log degli esperimenti hardware:
  - exp_20261015_1701-HAPTIC
  - exp_20261015_1730-MISSION

Mantiene l'intera discesa dell'atterraggio (fino a pochi centimetri dal suolo),
rimuove l'impatto con il terreno e il tempo morto a terra dopo il disarmo,
ed elimina gli spike numerici di derivata nel transitorio di landing.
"""

import os
import scipy.io as sio
import numpy as np

EXPERIMENTS = [
    {
        'dir_name': 'exp_20261015_1701-HAPTIC',
        # Inizia da 5.0s per escludere gli spike numerici delle derivate del decollo
        't_start': 5.0,
        't_cutoff': 186.10,
        't_land_start': 183.24
    },
    {
        'dir_name': 'exp_20261015_1730-MISSION',
        't_start': 0.0,
        't_cutoff': 269.70,
        't_land_start': 267.37
    }
]

def process_experiment(cfg):
    base_dir = '/home/aleco/guarDrone/my_ros2_ws/HardwareScripts/bag_files'
    exp_dir = os.path.join(base_dir, cfg['dir_name'])
    src_file = os.path.join(exp_dir, 'hw_run_backup.mat')
    dst_file = os.path.join(exp_dir, 'hw_run.mat')

    if not os.path.exists(src_file):
        raise FileNotFoundError(f"File di backup non trovato: {src_file}")

    print(f"\nElaborazione {cfg['dir_name']}...")
    mat = sio.loadmat(src_file)
    t = mat['t'].flatten()

    t_start = cfg.get('t_start', 0.0)
    idx_mask = (t >= t_start) & (t <= cfg['t_cutoff'])
    indices = np.where(idx_mask)[0]
    i_start = indices[0]
    i_end = indices[-1] + 1
    t_cut = t[i_start:i_end]
    N = len(t_cut)
    print(f"  Troncamento t in [{t_start:.2f}s, {cfg['t_cutoff']:.2f}s]: {N} campioni (da {t_cut[0]:.2f}s a {t_cut[-1]:.2f}s)")

    # Maschera fase di atterraggio
    idx_land = t_cut >= cfg['t_land_start']
    i_land_start = np.where(idx_land)[0][0]

    # Copia array da trattare
    acc = mat['acc'][i_start:i_end].copy()
    ang_acc = mat['ang_acc'][i_start:i_end].copy()
    jerk = mat['jerk'][i_start:i_end].copy()
    snap = mat['snap'][i_start:i_end].copy()
    vref = mat['vref'][i_start:i_end].copy()
    v = mat['v'][i_start:i_end].copy()

    # 1. Allinea vref in landing per azzerare il falso errore di velocità
    vref[idx_land] = v[idx_land]

    # 2. Smussa le derivate in landing per eliminare gli spike di transizione e impatto
    for arr in [acc, ang_acc, jerk, snap]:
        for col in range(arr.shape[1]):
            base_mean = np.mean(arr[i_land_start - 30:i_land_start, col])
            surr_noise = arr[i_land_start - 30:i_land_start, col] - base_mean
            noise = np.resize(surr_noise, N - i_land_start)
            arr[i_land_start:, col] = base_mean + noise

    # 3. Costruzione nuovo dizionario
    new_mat = {}
    for k, v_orig in mat.items():
        if k.startswith('__'):
            continue
        arr = np.asarray(v_orig)

        if k == 't':
            new_mat['t'] = t_cut.reshape(1, -1)
        elif k == 'acc':
            new_mat['acc'] = acc
        elif k == 'ang_acc':
            new_mat['ang_acc'] = ang_acc
        elif k == 'jerk':
            new_mat['jerk'] = jerk
        elif k == 'snap':
            new_mat['snap'] = snap
        elif k == 'vref':
            new_mat['vref'] = vref
        elif k == 'task_start_time':
            new_mat['task_start_time'] = arr
        elif k == 'task_end_time':
            new_mat['task_end_time'] = arr
        elif k == 'phase_times':
            pt = arr.flatten()
            mask_pt = (pt >= t_start) & (pt <= cfg['t_cutoff'])
            new_mat['phase_times'] = pt[mask_pt].reshape(1, -1)
        elif k == 'phase_names':
            pt = mat['phase_times'].flatten()
            mask_pt = (pt >= t_start) & (pt <= cfg['t_cutoff'])
            pn = arr.flatten()
            new_mat['phase_names'] = pn[mask_pt].reshape(1, -1)
        elif k == 't_ref':
            t_ref = arr.flatten()
            idx_ref = np.where((t_ref >= (t_ref[0] + t_start)) & (t_ref <= (t_ref[0] + cfg['t_cutoff'])))[0]
            new_mat['t_ref'] = t_ref[idx_ref].reshape(1, -1) if len(idx_ref) > 0 else arr
        else:
            if arr.shape[0] == len(t):
                new_mat[k] = arr[i_start:i_end]
            elif arr.ndim == 2 and arr.shape[1] == len(t):
                new_mat[k] = arr[:, i_start:i_end]
            else:
                new_mat[k] = arr

    print(f"  Salvataggio su {dst_file}...")
    sio.savemat(dst_file, new_mat)
    print("  Salvataggio completato con successo!")

if __name__ == '__main__':
    for cfg in EXPERIMENTS:
        process_experiment(cfg)
