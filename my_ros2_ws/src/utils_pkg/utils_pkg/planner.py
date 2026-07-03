#planner
import casadi as ca
import numpy as np


def generate_trapezoidal_trajectory(x0, x_ref, dt, v_max=1.0, a_max=2.0):
    """
    Genera una traiettoria p(t), rpy(t) con profilo trapezoidale lungo la distanza L.
    Il tempo totale T è calcolato in base alla cinematica (v_max, a_max).
    
    Args:
        x0: stato iniziale [x, y, z, roll, pitch, yaw]
        x_ref: stato finale [x, y, z, roll, pitch, yaw]
        dt: passo di campionamento (es. 0.02s per 50Hz)
        v_max: velocità massima [m/s]
        a_max: accelerazione massima [m/s^2]
        
    Returns:
        t_vec: array dei tempi
        p_vals: array delle posizioni (N, 3)
        rpy_vals: array degli angoli di eulero (N, 3)
    """
    p_in = np.array(x0[0:3])
    rpy_in = np.array(x0[3:6])
    p_f = np.array(x_ref[0:3])
    rpy_f = np.array(x_ref[3:6])

    dp = p_f - p_in
    L = np.linalg.norm(dp)
    
    # Se i punti sono troppo vicini, ritorna solo il punto finale
    if L < 1e-4:
        return np.array([0.0]), np.array([p_f]), np.array([rpy_f])

    # Calcolo tempi cinematici
    t_ramp = v_max / a_max
    d_ramp = 0.5 * a_max * t_ramp**2

    if L >= 2 * d_ramp:
        # Profilo Trapezoidale (raggiunge v_max)
        t_coast = (L - 2 * d_ramp) / v_max
        T = 2 * t_ramp + t_coast
    else:
        # Profilo Triangolare (non raggiunge v_max)
        t_ramp = np.sqrt(L / a_max)
        v_max = a_max * t_ramp
        t_coast = 0.0
        T = 2 * t_ramp

    t_vec = np.arange(0, T + dt, dt)
    s_vals = np.zeros_like(t_vec)

    for i, t in enumerate(t_vec):
        if t < t_ramp:
            s_vals[i] = 0.5 * a_max * t**2
        elif t < t_ramp + t_coast:
            s_vals[i] = d_ramp + v_max * (t - t_ramp)
        elif t <= T:
            dt_dec = t - (t_ramp + t_coast)
            s_vals[i] = (d_ramp + v_max * t_coast) + v_max * dt_dec - 0.5 * a_max * dt_dec**2
        else:
            s_vals[i] = L
            
    # Normalizza s in [0, 1]
    s_norm = np.clip(s_vals / L, 0.0, 1.0)
    s_rpy_norm = np.clip(t_vec / (T/4), 0.0, 1.0)

    p_vals = p_in + s_norm[:, np.newaxis] * dp
    
    # Interpolazione lineare (con attenzione allo sfasamento degli angoli se necessario, ma qui lineare va bene per rpy)
    drpy = rpy_f - rpy_in
    # Gestione rotazioni brevi su yaw
    drpy[2] = (drpy[2] + np.pi) % (2 * np.pi) - np.pi
    
    rpy_vals = rpy_in + s_rpy_norm[:, np.newaxis] * drpy

    return t_vec, p_vals, rpy_vals
