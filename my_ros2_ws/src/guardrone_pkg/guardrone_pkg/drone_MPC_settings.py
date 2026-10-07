from acados_template import AcadosOcp, AcadosOcpSolver
from guardrone_pkg.drone_model import *
from utils_pkg.common import *
from utils_pkg.planner import *
#from scipy.linalg import solve_continuous_are
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation 

#################  AGGIUSTARE: ricavare snap, jerk, acc in qualche modo perché da y_expr non si può tramite get(...)
##############  Estendere lo stato con tutti gli stati

def build_yref_online(y_idx, vel_ref, u_ref=np.zeros(4)):
    """Costruisce il vettore di riferimento online per la formulazione cartesiana."""
    yref = np.zeros(y_idx["u"].stop)
    yref[y_idx["pos"]]     = np.array([0.0, 0.0, 0.0, 0.0])  # [ex, ey, ez, yaw_err] → tutti zero
    yref[y_idx["vel"]]     = vel_ref
    yref[y_idx["ang_vel"]] = np.array([0.0, 0.0, 0.0])
    yref[y_idx["acc"]]     = np.array([0.0, 0.0, 0.0])
    yref[y_idx["acc_ang"]] = np.array([0.0, 0.0, 0.0])
    yref[y_idx["jerk"]]    = np.array([0.0, 0.0, 0.0])
    yref[y_idx["snap"]]    = np.array([0.0, 0.0, 0.0])
    yref[y_idx["u"]]       = u_ref
    return yref

def build_yref_terminal(y_idx, vel_ref, ny_e, u_ref=np.zeros(4)):
    y = build_yref_online(y_idx, vel_ref, u_ref)
    return y[:ny_e]


def setup_model(m, Ixx, Iyy, Izz):
    model = export_quadrotor_ode_model(m, Ixx, Iyy, Izz)
    model_rpy = convert_to_rpy_model(model, m, Ixx, Iyy, Izz)
    return model, model_rpy

def setup_initial_conditions(start_x,start_y,start_z,start_phi,start_theta,start_psi) :
    xx = start_x
    y =  start_y
    z =  start_z
    
    vx  = 0
    vy  = 0
    vz  = 0

    roll =  start_phi
    pitch = start_theta
    yaw =   start_psi

    q=Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    qw,qx,qy,qz = np.roll(q,1)
    q=q/ca.norm_2(q)

    wx=0
    wy=0
    wz=0

    x0 = np.array([xx,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz])  # 13 stati totali
    x0_rpy=np.array([xx,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz])  # 12 stati totali
    return x0,x0_rpy

def set_initial_state(ocp_solver, xk):
    ocp_solver.set(0, "lbx", xk)
    ocp_solver.set(0, "ubx", xk)

def configure_mpc(model : AcadosModel, x0, p_target, Tf, ts, W, W_e,
                  u_min, u_max,
                  yaw_des = 0.0):
    """
    p_target : np.array (3,) — posizione target cartesiana per il CoM (o camera) del drone.
                               Calcolata esternamente da: p_obj + cylindrical_to_cartesian(cyl_ref).
    yaw_des  : float [rad]  — yaw desiderato precalcolato esternamente.
    """
    
    nx = model.x.rows()
    nu = model.u.rows()

    m=model.m
    J = ca.DM(model.J).full()
    Ixx = J[0,0]
    Iyy = J[1,1]
    Izz = J[2,2]
    N_horiz = int(Tf/ts)

    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.tf = Tf
    ocp.solver_options.N_horizon = N_horiz

    '''
                                            STATE & KINEMATICS
    '''
    # Position
    p_expr = model.x[0:3]

    # Quaternion
    q_expr = model.x[6:10]
    qw = q_expr[0]; qx = q_expr[1]; qy = q_expr[2]; qz = q_expr[3]

    # Normalizzazione numerica
    q_norm = q_expr / ca.norm_2(q_expr)
    rpy_expr = quat_to_RPY(q_norm)
    roll = rpy_expr[0]
    pitch = rpy_expr[1]
    yaw = rpy_expr[2]
    w_expr = model.x[10:13]
    ang_vel = w_expr

    # Rotazione body→world — IMPORTANTE: usa q_norm (normalizzato)
    # R_expr e yaw devono venire dallo stesso quaternione, altrimenti
    # p_cam_expr e yaw_desired divergono generando un yaw_err residuo artificiale.
    R_expr = quat_to_R(q_norm)

    # state dynamics vector
    xdot = model.f_expl_expr

    # Velocity
    v_expr = model.x[3:6]

    # Acceleration
    acc_expr = xdot[3:6]
    acc_ang_expr = xdot[10:13]

    '''
                                            PROBLEMA IN COORDINATE CARTESIANE
    Parametri del modello (10 totali):
      p[0:3] = p_target  — target cartesiano per il drone (CoM o camera), precalcolato esternamente
                           da: p_obj + cylindrical_to_cartesian(cyl_ref) - Rb @ cam_offset
      p[3]   = yaw_des   — yaw desiderato [rad] precalcolato esternamente
      p[4:7] = F_ext     — forza esterna stimata (ENU)
      p[7:10]= Tau_ext   — coppia esterna stimata (FLU)
    '''
    p_target_sym = model.p[0:3]   # target cartesiano precalcolato (3 scalari)
    yaw_des_sym  = model.p[3]     # yaw desiderato precalcolato [rad]
    F_ext   = model.p[4:7]
    Tau_ext = model.p[7:10]

    # Errori cartesiani: Jacobiano = identità, condizionamento costante
    pos_err = p_target_sym - p_expr   # [ex, ey, ez]

    # r_cyl = distanza 2D drone-target (usata solo per il vincolo di safety, non nel costo)
    r_cyl = ca.sqrt((p_expr[0] - p_target_sym[0])**2 +
                    (p_expr[1] - p_target_sym[1])**2)

    # Yaw error: differenza rispetto allo yaw desiderato precalcolato (zero atan2 simbolico nel solver)
    yaw_err = min_angle(yaw - yaw_des_sym)
    #########################################################################################################                   
    #Jerk
    j_expr = ca.jacobian(acc_expr, model.x) @ xdot                
    #j_expr= ca.SX.zeros(3,1)
    #                                                                                          
    # Snap 
    s_expr = ca.jacobian(j_expr, model.x) @ xdot
    #s_expr= ca.SX.zeros(3,1)             
    #########################################################################################################
    
    u_hovering = ca.DM([m*g0, 0, 0, 0])
    #u_hovering = ca.DM.zeros(4, 1)
    acc_hover = ca.substitute(acc_expr, model.u, u_hovering)
    acc_ang_hover = ca.substitute(acc_ang_expr, model.u, u_hovering)
    j_hover = ca.substitute(j_expr, model.u, u_hovering)
    s_hover = ca.substitute(s_expr, model.u, u_hovering)



    '''
                                            CONSTRAINTS             
    '''
    ocp.constraints.x0 = x0
    
    # Vincoli sullo stato:
    # - Z (hard): idx 2 → drone non va sotto il suolo
    ocp.constraints.idxbx = np.array([2])
    ocp.constraints.lbx = np.array([-0.2])
    ocp.constraints.ubx = np.array([100.0])

    # Vincoli sugli ingressi (Spinta e Coppie)
    # Ora usiamo i parametri passati dinamicamente dal nodo per coerenza fisica
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.arange(nu)

    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hpipm_mode = 'SPEED'            # Tolleranze KKT rilassate (~1e-4) per real-time 100 Hz
    ocp.solver_options.qp_solver_iter_max = 15         # Tetto massimo iterazioni QP solver per evitare ritardi (WCET)
    #ocp.solver_options.qp_solver_cond_N = 5 # Scommentare per abilitare un condensing parziale per velocizzare ulteriormente (fake, non funziona)
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    #ocp.solver_options.globalization = 'MERIT_BACKTRACKING'

    # --- Vincolo di sicurezza: distanza minima dall'oggetto ---
    # r_cyl qui è la distanza 2D drone-target (approssimazione conservativa)
    #r_min = 1.0  # [m] distanza minima di sicurezza
    #h_expr = ca.vertcat(
    #    r_cyl - r_min,   # r_cyl >= r_min  (indice 0)
    #)
    #model.con_h_expr = h_expr

    # Soft constraints: [r_min]
    #ocp.constraints.lh = np.array([0.0])
    #ocp.constraints.uh = np.array([1e6])
    #ocp.constraints.idxsh = np.array([0])

    # soft constraints (L2 quadratico + L1 lineare)
    # [r_min]
    #penalty_L2 = np.array([1e3])
    #penalty_L1 = np.array([1e2])#
#
    #ocp.cost.Zl = penalty_L2#
    #ocp.cost.Zu = penalty_L2#
    #ocp.cost.zl = penalty_L1
    #ocp.cost.zu = penalty_L1

    '''
                                        COST FUNCTION — formulazione cartesiana
    '''
    # [ex, ey, ez, yaw_err, vel, ang_vel, acc, acc_ang, u]
    # Jacobiano posizionale = identità → condizionamento uniforme, nessuna dipendenza da r o beta
    y_expr = ca.vertcat(
        pos_err,                        # Errore posizione cartesiana [ex, ey, ez]
        yaw_err,                        # Errore yaw (punta verso il target)
        v_expr,                         # Velocità
        ang_vel,                        # Velocità angolari
        acc_expr,                       # Accelerazione
        acc_ang_expr,                   # Accelerazione angolare
        j_expr,                        # Jerk
        s_expr,                        # Snap
        model.u                         # Controllo
    )

    # Terminal cost expression
    y_expr_e = ca.vertcat(
        pos_err,
        yaw_err,
        v_expr,
        ang_vel,
        acc_hover,
        acc_ang_hover,
        j_hover,
        s_hover,
    )
    
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e
    
    ocp.cost.W = W
    ocp.cost.W_e = W_e
    ocp.cost.set = True
    
    # Parametri del modello (10 totali):
    # [p_target(3), yaw_des(1), F_ext(3), Tau_ext(3)]
    ocp.parameter_values = np.zeros(10)
    ocp.parameter_values[0:3] = p_target      # target cartesiano iniziale
    ocp.parameter_values[3]   = yaw_des       # yaw desiderato iniziale
    ocp.parameter_values[4:7] = np.zeros(3)   # F_ext
    ocp.parameter_values[7:10] = np.zeros(3)  # Tau_ext

    '''
                                        REFERENCES
    '''
    
    u_ref = np.array(u_hovering.full().flatten())

    # Indici del vettore y (formulazione cartesiana)
    pos_ind     = slice(0, 4)                                    # [ex, ey, ez, yaw_err]
    vel_ind     = slice(pos_ind.stop,     pos_ind.stop + 3)
    ang_vel_ind = slice(vel_ind.stop,     vel_ind.stop + 3)
    acc_ind     = slice(ang_vel_ind.stop, ang_vel_ind.stop + 3)
    acc_ang_ind = slice(acc_ind.stop,     acc_ind.stop + 3)
    jerk_ind    = slice(acc_ang_ind.stop, acc_ang_ind.stop + 3)
    snap_ind    = slice(jerk_ind.stop,    jerk_ind.stop + 3)
    #u_ind       = slice(acc_ang_ind.stop, acc_ang_ind.stop + 4) # CASO NO JERK e SNAP
    u_ind       = slice(snap_ind.stop,    snap_ind.stop + 4)

    y_idx = {
        "pos":     pos_ind,      # [ex, ey, ez, yaw_err]
        "vel":     vel_ind,
        "ang_vel": ang_vel_ind,
        "acc":     acc_ind,
        "acc_ang": acc_ang_ind,
        "jerk":    jerk_ind,
        "snap":    snap_ind,
        "u":       u_ind,
    }
    ny   = y_expr.numel()
    ny_e = y_expr_e.numel()

    # Tutti gli errori cilindrici sono già espressi come errore → riferimento = 0
    yref   = np.zeros(ny)
    yref[u_ind] = u_ref

    yref_e = yref[:ny_e]

    ocp.cost.yref = yref
    ocp.cost.yref_e = yref_e

    #ocp.solver_options.nlp_solver_max_iter=200
    ocp_solver = AcadosOcpSolver(ocp, build=True, generate=True)

    return ocp_solver, N_horiz, nx, nu, y_idx, ny, ny_e