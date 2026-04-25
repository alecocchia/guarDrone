#planner
import casadi as ca
import numpy as np


def generate_trapezoidal_trajectory(x0, x_ref, t0, tf, dt, v_max=1.0, a_max=1.0):
    """
    Genera una traiettoria p(t), rpy(t) con profilo trapezoidale lungo p_f - p_in e interpolazione lineare in rpy.
    
    Args:
        p_in: posizione iniziale (3,)
        p_f: posizione finale (3,)
        rpy_in: rotazione iniziale in RPY (3,)
        rpy_f: rotazione finale in RPY (3,)
        t0: tempo iniziale
        tf: tempo finale
        dt: passo di campionamento
        v_max: velocità massima normalizzata lungo la curvilinea s
        a_max: accelerazione massima normalizzata lungo s
        
    Returns:
        Trajectory: oggetto contenente t_vec, p_func(t), rpy_func(t)
    """

    p_in=x0[0:3]
    rpy_in=x0[3:6]
    p_f=x_ref[0:3]
    rpy_f=x_ref[3:6]

    dp = np.array(p_f) - np.array(p_in)
    L = np.linalg.norm(dp)
    if L == 0:
        raise ValueError("Punti iniziale e finale coincidenti.")

    # Tempo
    t_vec = np.arange(t0, tf + dt, dt)
    T = tf - t0
    t_sym = ca.SX.sym('t')

    # Trapezoidal profile s(t)
    t_ramp = v_max / a_max
    if T < 2 * t_ramp:
        t_ramp = T / 2
        v_max = a_max * t_ramp

    def s_trapezoid_expr(t):
        s1 = 0.5 * a_max * t_ramp**2
        t2 = T - t_ramp
        s2 = s1 + v_max * (t2 - t_ramp)

        s = ca.if_else(
            t < t_ramp,
            0.5 * a_max * t**2,
            ca.if_else(
                t < t2,
                s1 + v_max * (t - t_ramp),
                s2 + v_max * (t - t2) - 0.5 * a_max * (t - t2)**2
            )
        )
        s_total = s2 + v_max * t_ramp - 0.5 * a_max * t_ramp**2  # = L in teoria
        s_norm = ca.fmin(s / s_total, 1.0)  # Clamp a 1.0 per sicurezza
        return s_norm


    s_expr = s_trapezoid_expr(t_sym - t0)
    s_func = ca.Function('s', [t_sym], [s_expr])

    # Posizione
    p_expr = ca.vertcat(*[p_in[i] + s_expr * (p_f[i] - p_in[i]) for i in range(3)])
    p_func = ca.Function('p_t', [t_sym], [p_expr])

    # Rotazione (RPY)
    rpy_expr = ca.vertcat(*[rpy_in[i] + s_expr * (rpy_f[i] - rpy_in[i]) for i in range(3)])
    rpy_func = ca.Function('rpy_t', [t_sym], [rpy_expr])

    p_vals = np.array([p_func(t).full().flatten() for t in t_vec])
    rpy_vals = np.array([rpy_func(t).full().flatten() for t in t_vec])
    return (t_vec, p_vals, rpy_vals)


