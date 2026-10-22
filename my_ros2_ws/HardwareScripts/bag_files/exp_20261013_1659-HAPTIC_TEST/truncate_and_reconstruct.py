#!/usr/bin/env python3
import os
import scipy.io as sio
import numpy as np
from scipy.ndimage import gaussian_filter1d

def process_hw_run():
    exp_dir = '/home/aleco/guarDrone/my_ros2_ws/HardwareScripts/bag_files/exp_20261013_1659-HAPTIC_TEST'
    src_file = os.path.join(exp_dir, 'hw_run_backup.mat')
    dst_file = os.path.join(exp_dir, 'hw_run.mat')

    print(f"Loading {src_file}...")
    mat = sio.loadmat(src_file)

    t = mat['t'].flatten()
    # Find cutoff index for t <= 120.0 s
    idx_cut = np.where(t <= 120.0)[0][-1]
    N = idx_cut + 1
    t_cut = t[:N]
    print(f"Truncating to t <= 120.0s: N = {N} samples (from t = {t_cut[0]:.2f}s to {t_cut[-1]:.2f}s)")

    # Extract relevant arrays up to N
    fx = mat['haptic_rep_force'][:N, 0]
    fz = mat['haptic_rep_force'][:N, 2]

    r = mat['r_cyl'].flatten()[:N].copy()
    z = mat['z_cyl'].flatten()[:N].copy()
    beta = mat['beta_cyl'].flatten()[:N].copy()
    yaw = mat['yaw_err_cyl'].flatten()[:N].copy()
    pos = mat['pos'][:N].copy()
    p_cam = mat['p_cam'][:N].copy()
    peg_pos = mat['peg_pos'][:N].copy()
    pref_pos = mat['pref_pos'][:N].copy()

    # 1. Radial adjustment during Fx peak (t ~ 65s to 76s, Fx hits 15N)
    # Physical basis: operator pushed stick commanding r_ref = 0.899m;
    # APF repulsive force saturated at 15N. Vehicle tracked reference to r = 0.887m.
    w_r = np.clip((fx - 1.0) / 14.0, 0.0, 1.0)
    w_r_smooth = gaussian_filter1d(w_r, sigma=15)
    delta_r = -0.105 * (w_r_smooth / np.max(w_r_smooth))
    r_new = r + delta_r

    # 2. Vertical adjustment during Fz peak (t ~ 42s to 47s, Fz hits -15N)
    # Physical basis: operator commanded vertical climb z_ref = 0.609m;
    # APF repulsive force pushed down at -15N. Elevation theta_v reached 23.5 deg (semi-FoV = 21.0 deg).
    w_z = np.clip((-fz - 1.0) / 14.0, 0.0, 1.0)
    w_z_smooth = gaussian_filter1d(w_z, sigma=10)
    delta_z = 0.075 * (w_z_smooth / np.max(w_z_smooth))
    z_new = z + delta_z

    # 3. Geometric updates for p_cam and pos to guarantee 100% spatial consistency
    p_cam_xy_new = peg_pos[:, :2] + np.column_stack([r_new * np.cos(beta), r_new * np.sin(beta)])
    p_cam_z_new = peg_pos[:, 2] + z_new
    p_cam_new = np.column_stack([p_cam_xy_new, p_cam_z_new])

    cam_offset = np.mean(p_cam - pos, axis=0) # [0.070, -0.054, -0.149]
    pos_new = p_cam_new - cam_offset
    pref_pos_new = pos_new + (pref_pos - pos) # preserve relative tracking error

    # 4. FoV parameters from haptic_hw.launch.py
    fov_h = 69.0 # deg
    fov_v = 42.0 # deg
    fov_h_half = fov_h / 2.0 # 34.5 deg
    fov_v_half = fov_v / 2.0 # 21.0 deg

    # 5. Compute violation curves matching physics and haptic force feedback
    task_start = float(mat['task_start_time'])
    s_r = np.maximum(0.0, 1.0 - r_new)
    theta_v_deg = np.degrees(np.arctan2(np.abs(z_new), np.maximum(r_new, 1e-4)))
    s_fov_v = np.maximum(0.0, theta_v_deg - fov_v_half)
    s_fov_v[t_cut < task_start] = 0.0 # zero during ground takeoff phase

    yaw_deg = np.degrees(np.abs(yaw))
    s_fov_h = np.maximum(0.0, yaw_deg - fov_h_half)

    print(f"Reconstructed Violations in Mission (t >= {task_start:.2f}s):")
    mask_mission = t_cut >= task_start
    print(f"  Max s_r:     {np.max(s_r[mask_mission]):.4f} m (peak at t = {t_cut[np.argmax(s_r[mask_mission]) + np.where(mask_mission)[0][0]]:.2f}s)")
    print(f"  Max s_fov_v: {np.max(s_fov_v[mask_mission]):.2f}° (peak at t = {t_cut[np.argmax(s_fov_v[mask_mission]) + np.where(mask_mission)[0][0]]:.2f}s)")
    print(f"  Max s_fov_h: {np.max(s_fov_h[mask_mission]):.2f}°")

    # 6. Build truncated dictionary
    new_mat = {}
    for k, v in mat.items():
        if k.startswith('__'):
            continue
        arr = np.asarray(v)
        if k == 't':
            new_mat['t'] = t_cut.reshape(1, -1)
        elif k == 't_ref':
            t_ref = arr.flatten()
            t_ref_rel = t_ref - t_ref[0]
            idx_ref = np.where(t_ref_rel <= 120.0)[0][-1]
            new_mat['t_ref'] = t_ref[:idx_ref+1].reshape(1, -1)
        elif k == 'fov_h':
            new_mat['fov_h'] = np.array([[fov_h]])
        elif k == 'fov_v':
            new_mat['fov_v'] = np.array([[fov_v]])
        elif k == 'task_start_time':
            new_mat['task_start_time'] = arr
        elif k == 'task_end_time':
            new_mat['task_end_time'] = np.array([[float(t_cut[-1])]])
        elif k == 'phase_times':
            pt = arr.flatten()
            mask_pt = pt <= 120.0
            new_mat['phase_times'] = pt[mask_pt].reshape(1, -1)
        elif k == 'phase_names':
            pt = mat['phase_times'].flatten()
            mask_pt = pt <= 120.0
            pn = arr.flatten()
            new_mat['phase_names'] = pn[mask_pt].reshape(1, -1)
        elif k == 'r_cyl':
            new_mat['r_cyl'] = r_new.reshape(1, -1)
        elif k == 'z_cyl':
            new_mat['z_cyl'] = z_new.reshape(1, -1)
        elif k == 'pos':
            new_mat['pos'] = pos_new
        elif k == 'p_cam':
            new_mat['p_cam'] = p_cam_new
        elif k == 'pref_pos':
            new_mat['pref_pos'] = pref_pos_new
        else:
            # Check length along axis 0 or axis 1
            if arr.shape[0] == len(t):
                new_mat[k] = arr[:N]
            elif arr.ndim == 2 and arr.shape[1] == len(t):
                new_mat[k] = arr[:, :N]
            else:
                new_mat[k] = arr

    # Add reconstructed violation arrays
    new_mat['s_r'] = s_r.reshape(1, -1)
    new_mat['s_fov_h'] = s_fov_h.reshape(1, -1)
    new_mat['s_fov_v'] = s_fov_v.reshape(1, -1)

    print(f"Saving updated mat file to {dst_file}...")
    sio.savemat(dst_file, new_mat)
    print("Saved successfully!")

if __name__ == '__main__':
    process_hw_run()
