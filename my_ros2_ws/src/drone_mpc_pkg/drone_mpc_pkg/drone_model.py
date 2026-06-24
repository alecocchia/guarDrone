#DRONE MODEL
from acados_template import AcadosModel
import numpy as np
import casadi as ca
from drone_mpc_pkg.common import *

def export_quadrotor_ode_model(m, Ixx, Iyy, Izz, camera_offset=None, camera_rpy=None) -> AcadosModel:
    """Quadrotor ODE model con parametri sferici mondiali.
    camera_offset e camera_rpy sono mantenuti per compatibilità con setup_model()
    ma non vengono più usati nel costo (formulazione sferica mondiale).
    """

    model_name = 'quadrotor_ode'

    # Model parameters
    g = ca.vertcat(0,0,g0)   # gravity [m/s^2]
    J = ca.SX(np.diag([Ixx, Iyy, Izz])) #Inertia

    # States
    # Position
    px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
    p = ca.vertcat(px, py, pz)

    # Linear velocity
    vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
    v = ca.vertcat(vx, vy, vz)

    # Quaternion (orientation)
    qw = ca.SX.sym('qw')
    qx = ca.SX.sym('qx')
    qy = ca.SX.sym('qy')
    qz = ca.SX.sym('qz')
    q = ca.vertcat(qw, qx, qy, qz)

    # Angular velocity
    wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')
    w = ca.vertcat(wx, wy, wz)

    # Inputs (generalized forces) in body frame
    Fz = ca.SX.sym('Fz')
    tau_x = ca.SX.sym('tau_x')
    tau_y = ca.SX.sym('tau_y')
    tau_z = ca.SX.sym('tau_z')
    u = ca.vertcat(Fz, tau_x, tau_y, tau_z)

    # Rotation matrix from quaternion
    Rb = quat_to_R(q)

    # Model parameters (p) — coordinate sferiche mondiali
    # p[0:3] = p_obj   (posizione oggetto nel mondo)
    # p[3]   = r_ref   (distanza di riferimento [m])
    # p[4]   = beta_ref  (azimut di riferimento [rad], angolo drone->obj nel piano XY)
    # p[5]   = gamma_ref (elevazione di riferimento [rad], 0=piano, +pi/2=zenit)
    ref_sym = ca.SX.sym('p', 6)
    # (i simboli vengono usati direttamente in drone_MPC_settings.py tramite model.p[...])

    # Equations of motion (ODEs)
    p_dot = v
    # v_dot: nominal thrust + gravity
    v_dot = (1/m) * (ca.mtimes(Rb, ca.vertcat(0, 0, Fz))) - g
    q_dot = 0.5 * ca.mtimes(omega_matrix(w), q)
    J_inv = ca.inv(J)
    w_dot = ca.mtimes(J_inv, (ca.vertcat(tau_x, tau_y, tau_z) - ca.cross(w, ca.mtimes(J, w))))
    # Compose augmented state [p, v, q, w] (13 states)
    x = ca.vertcat(p, v, q, w)
    xdot = ca.SX.sym('xdot', x.shape)

    f_expl = ca.vertcat(p_dot, v_dot, q_dot, w_dot)
    f_impl = xdot - f_expl

    # Define model
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    
    model.p = ref_sym       # model.p = parameters 

    model.name = model_name
    model.m = m             # Salviamo la massa nel modello per poterla recuperare dall'MPC
    model.J = J             # Idem per inerzia

    # define in x_labels the roll, pitch and yaw instead of quaternion
    model.x_labels = [
        r'$x$', r'$y$', r'$z$',
        r'$v_x$', r'$v_y$', r'$v_z$',
        r'$q_w$', r'$q_x$', r'$q_y$', r'$q_z$',
        r'$\omega_x$', r'$\omega_y$', r'$\omega_z$'
    ]
    model.u_labels = [r'$F_z$', r'$\tau_x$', r'$\tau_y$', r'$\tau_z$']
    model.t_label = '$t$ [s]'

    return model

#Drone model rpy
def convert_to_rpy_model(model_quat,m,Ixx,Iyy,Izz):

    # Model parameters
    g = ca.vertcat(0,0,g0)   # gravity [m/s^2]
    J=ca.SX(np.diag([Ixx,Iyy,Izz]))

    # Nuove variabili di stato
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    rpy = ca.SX.sym('rpy', 3)
    omega = ca.SX.sym('omega', 3)
    x = ca.vertcat(p, v, rpy, omega)

    # Controlli
    u = model_quat.u
    Fz = u[0]
    tau = u[1:]

    F=ca.vertcat(0,0,Fz)

    # Rotazione da RPY
    phi = rpy[0]
    theta=rpy[1]
    psi=rpy[2]
    Rb=RPY_to_R(phi,theta,psi)

    dp = v
    dv = (1/m) * ca.mtimes(Rb ,F) - g

    # Derivata degli angoli di eulero
    #T = ca.SX(3,3)
    #T[0,:] = ca.horzcat(1, sin(phi)*tan(theta), cos(phi)*tan(theta))
    #T[1,:] = ca.horzcat(0, cos(phi),           -sin(phi))
    #T[2,:] = ca.horzcat(0, sin(phi)/cos(theta), cos(phi)/cos(theta))
    #drpy = T @ omega
    drpy = angularVel_to_EulerRates(phi,theta,psi,omega)

    domega = ca.mtimes(ca.inv(J), (tau - ca.cross(omega, ca.mtimes(J, omega))))

    xdot = ca.vertcat(dp, dv, drpy, domega)

    model_rpy = type('', (), {})()
    model_rpy.x = x
    model_rpy.u = u
    model_rpy.xdot = xdot
    model_rpy.f_expl_expr = xdot
    model_rpy.name = model_quat.name + "_rpy"
    ref_sym = ca.SX.sym('p', 3)  # simbolico per ref (p,rpy)
    model_rpy.p = ref_sym       #model.p = parameters 
    model_rpy.m = m
    model_rpy.g = g0
    model_rpy.J = J
        
    #define in x_labels the roll, pitch and yaw
    model_rpy.x_labels = [
        r'$x$', r'$y$', r'$z$',
        r'$v_x$', r'$v_y$', r'$v_z$',
        r'$\phi$', r'$\theta$', r'$\psi$',
        r'$\omega_x$', r'$\omega_y$', r'$\omega_z$'
    ]
    model_rpy.u_labels = [r'$F_z$', r'$\tau_x$', r'$\tau_y$', r'$\tau_z$']
    model_rpy.t_label = '$t$ [s]'

    return model_rpy
