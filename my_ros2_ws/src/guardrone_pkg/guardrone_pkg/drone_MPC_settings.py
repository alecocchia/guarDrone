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
    """Costruisce il vettore di riferimento online per la formulazione cilindrica.
    Gli errori cilindrici (r_cyl_err, beta_err, z_err, yaw_err) hanno riferimento 0
    perché sono già espressi come errore nel modello.
    """
    yref = np.zeros(y_idx["u"].stop)
    yref[y_idx["cyl"]]     = np.array([0.0, 0.0, 0.0, 0.0])  # [r_cyl_err, beta_err, z_err, yaw_err] → tutti zero
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


def setup_model(m, Ixx, Iyy, Izz, camera_offset, camera_rpy):
    model = export_quadrotor_ode_model(m, Ixx, Iyy, Izz, camera_offset, camera_rpy)
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

    wx=0
    wy=0
    wz=0

    x0 = np.array([xx,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz])  # 13 stati totali
    x0_rpy=np.array([xx,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz])
    return x0,x0_rpy

def set_initial_state(ocp_solver, xk):
    ocp_solver.set(0, "lbx", xk)
    ocp_solver.set(0, "ubx", xk)

def configure_mpc(model : AcadosModel, x0, p_obj, Tf, ts, W, W_e,
                  u_min, u_max,
                  cyl_ref = np.zeros(3),
                  cam_offset_body = np.zeros(3)):
    
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
                                            PROBLEMA IN COORDINATE CILINDRICHE
    Parametri del modello (12 totali):
      p[0:3] = p_obj   (posizione oggetto nel mondo)
      p[3]   = r_cyl_ref (distanza di riferimento orizzontale)
      p[4]   = beta_ref  (azimut di riferimento, angolo nel piano XY)
      p[5]   = z_ref     (quota di riferimento relativa)
      p[6:9] = F_ext
      p [9:12] = Tau_ext
    '''
    p_obj_expr = model.p[0:3]
    r_cyl_ref_sym = model.p[3]
    beta_ref_sym  = model.p[4]
    z_ref_sym     = model.p[5]

    F_ext = model.p[6:9]
    Tau_ext = model.p[9:12]

    # Posizione della telecamera nel mondo
    p_cam_expr = p_expr + R_expr @ ca.DM(cam_offset_body)

    # Vettore telecamera → oggetto nel frame mondo
    p_rel = p_obj_expr - p_cam_expr

    # Distanza 2D (sul piano orizzontale): sempre > 0
    r_cyl = ca.sqrt(p_rel[0]**2 + p_rel[1]**2)
    r_cyl_err = r_cyl_ref_sym - r_cyl

    # Azimut: angolo del vettore drone -> obj nel piano XY (rad)
    beta_raw = ca.atan2(p_rel[1], p_rel[0])
    beta_err = min_angle(beta_ref_sym - beta_raw)

    # Quota relativa: differenza lungo Z
    z_err = z_ref_sym - p_rel[2]
    # Yaw error: il drone deve puntare verso l'oggetto
    # La direzione desiderata è -p_rel (da drone verso oggetto)
<<<<<<< HEAD
    yaw_desired = beta_raw + np.pi
    yaw_err = min_angle(yaw - yaw_desired)
=======
    yaw_desired = ca.atan2(p_rel[1], p_rel[0])
    yaw_err = min_angle(yaw_desired - yaw)
>>>>>>> 0f53cea52a0c23dbbd293d9dd0c87b9e0c449241

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
    #ocp.solver_options.qp_solver_cond_N = 5 # Scommentare per abilitare un condensing parziale per velocizzare ulteriormente (fake, non funziona)
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    #ocp.solver_options.globalization = 'MERIT_BACKTRACKING'

    # --- Vincolo di sicurezza: distanza minima dall'oggetto ---
    r_min = 1.5  # [m] distanza minima di sicurezza
    h_expr = ca.vertcat(
        r_cyl - r_min,   # r_cyl >= r_min  (indice 0)
    )
    model.con_h_expr = h_expr

    # Soft constraints: [r_min]
    ocp.constraints.lh = np.array([0.0])
    ocp.constraints.uh = np.array([1e6])
    ocp.constraints.idxsh = np.array([0])

    # Pesi soft (L2 quadratico + L1 lineare)
    # [r_min, roll, pitch]
    penalty_L2 = np.array([5e2])
    penalty_L1 = np.array([1e1])

    ocp.cost.Zl = penalty_L2
    ocp.cost.Zu = penalty_L2
    ocp.cost.zl = penalty_L1
    ocp.cost.zu = penalty_L1

    '''
                                        COST FUNCTION               
    '''
    # Cost function quantities — formulazione cilindrica mondiale
    # [r_cyl_err, beta_err, z_err, yaw_err, vel, ang_vel, acc, acc_ang, jerk, snap, u]
    y_expr = ca.vertcat(
        r_cyl_err,                      # Errore distanza orizzontale
        beta_err,                       # Errore azimut (orbita orizzontale)
        z_err,                          # Errore quota verticale
        yaw_err,                        # Errore yaw (punta verso l'oggetto)
        v_expr,                         # Velocità
        ang_vel,                        # Velocità angolari
        acc_expr,                       # Accelerazione
        acc_ang_expr,                   # Accelerazione angolare
        j_expr,                         # Jerk
        s_expr,                         # Snap
        model.u                         # Controllo
    )

    # Terminal cost expression
    y_expr_e = ca.vertcat(
        r_cyl_err,
        beta_err,
        z_err,
        yaw_err,
        v_expr,
        ang_vel,
        acc_hover,
        acc_ang_hover,
        #j_hover,                        # jerk
        #s_hover,                        # snap
    )
    
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e
    
    ocp.cost.W = W
    ocp.cost.W_e = W_e
    ocp.cost.set = True
    
    # Parametri del modello (12 totali):
    # [p_obj(3), r_cyl_ref(1), beta_ref(1), z_ref(1), F_ext(3), Tau_ext(3)]
    ocp.parameter_values = np.zeros(12)
    ocp.parameter_values[0:3] = p_obj[0,:]
    ocp.parameter_values[3]   = cyl_ref[0]   # r_cyl_ref
    ocp.parameter_values[4]   = cyl_ref[1]   # beta_ref
    ocp.parameter_values[5]   = cyl_ref[2]   # z_ref
    ocp.parameter_values[6:9] = np.array([0,0,0]) # F_ext
    ocp.parameter_values[9:12] = np.array([0,0,0]) # Tau_ext

    '''
                                        REFERENCES
    '''
    
    u_ref = np.array(u_hovering.full().flatten())

    # Indici del vettore y (formulazione cilindrica)
    cyl_ind     = slice(0, 4)                                    # [r_cyl_err, beta_err, z_err, yaw_err]
    vel_ind     = slice(cyl_ind.stop,     cyl_ind.stop + 3)
    ang_vel_ind = slice(vel_ind.stop,     vel_ind.stop + 3)
    acc_ind     = slice(ang_vel_ind.stop, ang_vel_ind.stop + 3)
    acc_ang_ind = slice(acc_ind.stop,     acc_ind.stop + 3)
    jerk_ind    = slice(acc_ang_ind.stop, acc_ang_ind.stop + 3)
    snap_ind    = slice(jerk_ind.stop,    jerk_ind.stop + 3)
    u_ind       = slice(snap_ind.stop,    snap_ind.stop + 4)

    y_idx = {
        "cyl":     cyl_ind,      # [r_cyl_err, beta_err, z_err, yaw_err]
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
    ocp_solver = AcadosOcpSolver(ocp)

    return ocp_solver, N_horiz, nx, nu, y_idx, ny, ny_e