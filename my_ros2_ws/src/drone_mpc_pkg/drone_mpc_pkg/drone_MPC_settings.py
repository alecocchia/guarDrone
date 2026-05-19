from acados_template import AcadosOcp, AcadosOcpSolver
from drone_mpc_pkg.drone_model import *
from drone_mpc_pkg.common import *
from drone_mpc_pkg.planner import *
#from scipy.linalg import solve_continuous_are
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation 

#################  AGGIUSTARE: ricavare snap, jerk, acc in qualche modo perché da y_expr non si può tramite get(...)
##############  Estendere lo stato con tutti gli stati

def build_yref_online(y_idx, visual_ref, vel_ref, u_ref=np.zeros(4)):
    yref = np.zeros(y_idx["u"].stop) 
    yref[y_idx["pan"]]     = 0.0                    # Errore di pan centrato in 0 (gestito da min_angle nel modello)
    yref[y_idx["vel"]]     = vel_ref
    yref[y_idx["rp"]]      = np.array([0,0])        # X_c, Y_c (posizione dell'oggetto rispetto alla camera, nella terna camera)
    yref[y_idx["visual"]]  = visual_ref
    yref[y_idx["integral"]]= np.array([0,0,0])
    yref[y_idx["ang_vel"]] = np.array([0,0,0])
    yref[y_idx["acc"]]     = np.array([0,0,0])
    yref[y_idx["acc_ang"]] = np.array([0,0,0])
    yref[y_idx["jerk"]]    = np.array([0,0,0])
    yref[y_idx["snap"]]    = np.array([0,0,0])
    yref[y_idx["u"]]       = u_ref
    return yref

def build_yref_terminal(y_idx, visual_ref, vel_ref, ny_e, u_ref=np.zeros(4)):
    y = build_yref_online(y_idx, visual_ref, vel_ref, u_ref)
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

    x0 = np.array([xx,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz])
    x0_rpy=np.array([xx,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz])
    return x0,x0_rpy

def set_initial_state(ocp_solver, xk):
    ocp_solver.set(0, "lbx", xk)
    ocp_solver.set(0, "ubx", xk)

def configure_mpc(model : AcadosModel, x0, camera_offset, p_obj, rpy_obj, Tf, ts, W, W_e, 
                  u_min, u_max,
                  pan_ref = 0.0, visual_ref = np.zeros(3), vel_ref = np.zeros(3),
                  cam_rpy = np.zeros(3), fov_h = 80.0, fov_v = 60.0,
                  rp_limit = 35.0 * np.pi / 180.0):
    
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
    # Position - Cartesiana Pura
    p_expr = model.x[0:3]

    # Quaternione di stato - Orientamento Puro
    q_expr = model.x[6:10]
    
    # Derivata per Euler rates e Yaw
    qw = q_expr[0]
    qx = q_expr[1]
    qy = q_expr[2]
    qz = q_expr[3]
    # yaw_expr = ca.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)) # Non usato
    
    # Normalizzazione per sicurezza numerica
    q_norm = q_expr / ca.norm_2(q_expr)
    rpy_expr = quat_to_RPY(q_norm)
    roll = rpy_expr[0]
    pitch = rpy_expr[1]
    yaw = rpy_expr[2]
    w_expr = model.x[10:13]
    ang_vel=w_expr

    # Stati integrali
    xi_expr = model.x[13:16]

    # Rotazione attuale del drone rispetto al world
    R_expr = quat_to_R(q_expr)

    # --- Parte visuale --> Sistema camera ---  
    d_cam = ca.DM(camera_offset[0:3]).reshape((3,1))

    # posizione camera = posizione drone nel mondo + posa camera-body ruotata nel mondo
    p_cam = p_expr + R_expr @ d_cam   # Posizione della camera nel mondo

    # Orientamento della camera rispetto al body del drone (FLU)
    R_cam_body = RPY_to_R(cam_rpy[0], cam_rpy[1], cam_rpy[2])

    fov_h_rad = fov_h * ca.pi / 180.0
    fov_v_rad = fov_v * ca.pi / 180.0

    T_h = ca.tan(fov_h_rad / 2.0)
    T_v = ca.tan(fov_v_rad / 2.0)

    p_obj_expr = model.p[0:3]
    p_rel_world = p_obj_expr - p_cam

    # Posa relativa dell'oggetto rispetto alla camera, nella terna camera
    # Mondo -> body_drone -> camera
    P_c = R_cam_body.T @ R_expr.T @ p_rel_world    

    X_c = P_c[0]
    Y_c = P_c[1]
    Z_c = P_c[2]

    # Pan mutuo: angolo del vettore (P_cam - P_obj) nel piano XY globale
    # Usiamo min_angle per calcolare la distanza minima rispetto al riferimento passato come parametro p[3]
    pan_raw = ca.atan2(p_cam[1] - p_obj_expr[1], p_cam[0] - p_obj_expr[0])
    pan_expr = min_angle(pan_raw - model.p[3])

    # state dynamics vector
    xdot = model.f_expl_expr  


    # Velocity
    v_expr = model.x[3:6]  

    # Acceleration 
    acc_expr = xdot[3:6]
    acc_ang_expr = xdot[10:13]

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
    acc_hover = ca.substitute(acc_expr, model.u, u_hovering)
    acc_ang_hover = ca.substitute(acc_ang_expr, model.u, u_hovering)
    j_hover = ca.substitute(j_expr, model.u, u_hovering)
    s_hover = ca.substitute(s_expr, model.u, u_hovering)



    '''
                                            CONSTRAINTS             
    '''
    ocp.constraints.x0 = x0
    
    # Vincolo su Z (Hard)
    ocp.constraints.idxbx = np.array([2])
    ocp.constraints.lbx = np.array([-1.0])
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

    visual_constr_expr = ca.vertcat(
        Y_c - T_h * X_c,  # Limite Destro:  Y_c <= T_h * X_c
        Y_c + T_h * X_c,  # Limite Sinistro: Y_c >= -T_h * X_c
        Z_c - T_v * X_c,  # Limite Alto:    Z_c <= T_v * X_c
        Z_c + T_v * X_c,  # Limite Basso:   Z_c >= -T_v * X_c
        X_c               # Profondità:     X_c >= X_min
    )

    # Vincolo su Roll e Pitch (Soft Constraints)
    h_expr = ca.vertcat(
        visual_constr_expr,
        roll,
        pitch
    )
    
    model.con_h_expr = h_expr
    
    X_min = 1.5 
    # [Y_right, Y_left, Z_up, Z_down, X_min, roll, pitch]
    ocp.constraints.lh = np.array([-1000,  0.0, -1000,  0.0, X_min, -rp_limit, -rp_limit])
    ocp.constraints.uh = np.array([ 0.0,  1000,  0.0,  1000, 100,    rp_limit,  rp_limit])

    # Slacks per vincoli visuali (5) + roll/pitch (2) = 7
    n_soft_h = 7
    ocp.constraints.idxsh = np.array(range(n_soft_h))

    # Pesi per i soft constraints
    # [Visx4, dist_sicurezza, roll, pitch]
    penalty_L1 = 1e-1
    penalty_L2 = 1e0 
    weights_costs = np.array([1, 1, 1, 1, 100, 1, 1])

    ocp.cost.Zl = penalty_L2 * weights_costs
    ocp.cost.Zu = penalty_L2 * weights_costs
    ocp.cost.zl = penalty_L1 * weights_costs
    ocp.cost.zu = penalty_L1 * weights_costs

  #  # --- Fine parte visuale --- 

    '''
                                        COST FUNCTION               
    '''
    # Cost function quantities (expressed with respect to state and control)
    y_expr = ca.vertcat(
        pan_expr,                       # Pan Mutuo
        X_c,                            # Posizione X dell'oggetto rispetto alla camera
        Y_c,                            # Posizione Y dell'oggetto rispetto alla camera
        Z_c,                            # Posizione Z dell'oggetto rispetto alla camera
        xi_expr,                        # INTEGRALI (ix, iy, iz)
        v_expr,                         # velocity
        roll,                           # Roll
        pitch,                          # Pitch
        ang_vel,                        # Euler rates (non è vero, ora sono velocità angolari)
        acc_expr,                       # acceleration
        acc_ang_expr,                   # angular acceleration
        j_expr,                         # jerk
        s_expr,                         # snap
        model.u                         # control
    )
    
    # Terminal cost exrpession
    y_expr_e = ca.vertcat(
        pan_expr,                       # Pan Mutuo
        X_c,
        Y_c,
        Z_c,
        xi_expr,                        # INTEGRALI
        v_expr,                         # velocity
        roll,
        pitch,                         
        ang_vel,                        # Euler rates
        acc_hover,                      # acceleration
        acc_ang_hover,
        j_hover,                        # jerk
        s_hover,                        # snap
    )
    
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e
    
    ocp.cost.W = W
    ocp.cost.W_e = W_e
    ocp.cost.set = True
    
    # I parametri ora passati al modello (p) sono 7: 
    # [p_obj (3), pan_ref (1), visual_ref (3)]
    ocp.parameter_values = np.zeros(7)
    ocp.parameter_values[0:3]   = p_obj[0,:] 
    ocp.parameter_values[3]     = pan_ref
    ocp.parameter_values[4:7]   = visual_ref

    '''
                                        REFERENCES
    '''
    
    # Definition of constant references
    rp_ref = np.array([0,0]) #roll and pitch refs
    ang_vel_ref = np.array([0,0,0])
    acc_ref=np.array([0,0,0])
    acc_ang_ref = np.array([0,0,0])
    jerk_ref=np.array([0,0,0])
    snap_ref=np.array([0,0,0])
    u_ref=np.array(u_hovering.full().flatten())


    # Indexes (Aggiornati per le nuove dimensioni)
    pan_ind = slice(0,1) # pan
    visual_ind = slice(pan_ind.stop,pan_ind.stop+3) # X_c, Y_c, Z_c
    integral_ind = slice(visual_ind.stop, visual_ind.stop+3) # INTEGRALI
    vel_ind = slice(integral_ind.stop, integral_ind.stop+3)
    rp_ind = slice(vel_ind.stop, vel_ind.stop+2)
    #quat_ind = slice(vel_ind.stop, vel_ind.stop+4)
    ang_vel_ind = slice(rp_ind.stop,rp_ind.stop+3)
    acc_ind = slice(ang_vel_ind.stop,ang_vel_ind.stop+3)
    acc_ang_ind = slice(acc_ind.stop,acc_ind.stop+3)
    jerk_ind = slice(acc_ang_ind.stop,acc_ang_ind.stop+3)   
    snap_ind = slice(jerk_ind.stop,jerk_ind.stop+3)
    u_ind = slice(snap_ind.stop,snap_ind.stop+4)

    y_idx = {
        "pan": pan_ind,
        "visual": visual_ind,
        "integral": integral_ind,
        "vel": vel_ind,
        "rp": rp_ind,
        "ang_vel": ang_vel_ind,
        "acc": acc_ind,
        "acc_ang": acc_ang_ind,
        "jerk": jerk_ind,
        "snap": snap_ind,
        "u": u_ind,
    }
    ny   = y_idx["u"].stop   
    ny_e = y_idx["u"].start  
    
    yref = np.zeros(y_expr.numel())
    yref_e = np.zeros(y_expr_e.numel())

    # ASSIGN REFERENCES
    yref[pan_ind]= 0.0                  # L'errore di pan è già gestito nel modello (distanza minima da p[9])
    yref[visual_ind]=visual_ref
    yref[vel_ind]=vel_ref
    yref[rp_ind]= rp_ref
    yref[ang_vel_ind]= ang_vel_ref
    yref[acc_ind]=acc_ref
    yref[acc_ang_ind]=acc_ang_ref
    yref[jerk_ind]=jerk_ref         
    yref[snap_ind]=snap_ref         
    yref[u_ind]=u_ref               

    #new_ref = yref.copy()
    #new_ref[pos_ind]=final_ref[0:3]
    #new_ref[quat_ind]=final_ref[3:7]

    yref_e = yref[:y_expr_e.numel()]  

    ocp.cost.yref = yref
    ocp.cost.yref_e = yref_e

    #ocp.solver_options.nlp_solver_max_iter=200
    ocp_solver = AcadosOcpSolver(ocp)

    return ocp_solver, N_horiz, nx, nu, y_idx, ny, ny_e