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

def build_yref_online(y_idx, pan_ref, visual_ref, u_ref=np.zeros(4)):
    yref = np.zeros(y_idx["u"].stop) 
    yref[y_idx["pan"]]     = pan_ref                # Pan Mutuo
    yref[y_idx["vel"]]     = np.array([0,0,0])
    #yref[y_idx["quat"]]    = xy_pos_ref[3:7]          # Quaternione puro w, x, y, z
    yref[y_idx["rp"]]      = np.array([0,0])        # X_c, Y_c (posizione dell'oggetto rispetto alla camera, nella terna camera)
    yref[y_idx["visual"]]  = visual_ref
    yref[y_idx["dot_rpy"]] = np.array([0,0,0])
    yref[y_idx["acc"]]     = np.array([0,0,0])
    yref[y_idx["acc_ang"]] = np.array([0,0,0])
    yref[y_idx["jerk"]]    = np.array([0,0,0])
    yref[y_idx["snap"]]    = np.array([0,0,0])
    yref[y_idx["u"]]       = u_ref
    return yref

def build_yref_terminal(y_idx, pan_ref, visual_ref, ny_e, u_ref=np.zeros(4)):
    y = build_yref_online(y_idx, pan_ref, visual_ref, u_ref)
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
                  pan_ref = 0.0, visual_ref = np.zeros(3),
                  cam_rpy = np.zeros(3), fov_h = 80.0, fov_v = 60.0):
    
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
    
    rp_expr = q_expr[1:3]
    w_expr = model.x[10:13]
    dot_rpy=w_expr

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
    pan_expr = ca.atan2(p_cam[1] - p_obj_expr[1], p_cam[0] - p_obj_expr[0])

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
    ocp.constraints.lbx = np.array([-100.0])  # zmin molto basso per evitare fallimenti del solver
    ocp.constraints.ubx = np.array([100.0])  # zmax
    ocp.constraints.idxbx = np.array([2])   

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

    # ==========================================================
    # Constraints for camera
    # ==========================================================
    visual_constr_expr = ca.vertcat(
        Y_c - T_h * X_c,  # Limite Destro:  Y_c <= T_h * X_c
        Y_c + T_h * X_c,  # Limite Sinistro: Y_c >= -T_h * X_c
        Z_c - T_v * X_c,  # Limite Alto:    Z_c <= T_v * X_c
        Z_c + T_v * X_c,  # Limite Basso:   Z_c >= -T_v * X_c
        X_c               # Profondità:     X_c >= X_min
    )
    
    model.con_h_expr = visual_constr_expr
    
    X_min = 1.5 # Il peg deve stare almeno a X_min DAVANTI alla telecamera (distanza di sicurezza)
    ocp.constraints.lh = np.array([-1000,  0.0, -1000,  0.0, X_min])
    ocp.constraints.uh = np.array([ 0.0,  1000,  0.0,  1000, 100])

    # ==========================================================
    # SOFT CONSTRAINTS (Slack Variables)
    # ==========================================================
    n_soft_h = 5
    ocp.constraints.idxsh = np.array(range(n_soft_h))

    # Riduciamo drasticamente i pesi dei soft constraints visuali
    # in modo che il drone non impazzisca se il peg esce temporaneamente dal FOV
    # Aumentiamo le penalità per rendere i vincoli più rigidi ed evitare collisioni
    penalty_L1 = 1e-2
    penalty_L2 = 1e-1  
    weights_costs = np.array([1, 1, 1, 1, 100])

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
        v_expr,                         # velocity
        rp_expr,                         # Roll e pitch (non è vero, ora sono qx,qy)
        dot_rpy,                        # Euler rates (non è vero, ora sono velocità angolari)
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
        v_expr,                         # velocity
        rp_expr,                         # Orientamento attuale (w,x,y,z)
        dot_rpy,                        # Euler rates
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
    
    # I parametri ora passati al modello (p) sono [p_obj, f_ext, tau_ext]
    ocp.parameter_values = np.zeros(9)
    ocp.parameter_values[0:3] = p_obj[0,:] 

    '''
                                        REFERENCES
    '''
    
    # Definition of constant references
    rp_ref = np.array([0,0]) #roll and pitch refs
    dot_rpy_ref = np.array([0,0,0])
    v_ref=np.array([0,0,0])
    acc_ref=np.array([0,0,0])
    acc_ang_ref = np.array([0,0,0])
    jerk_ref=np.array([0,0,0])
    snap_ref=np.array([0,0,0])
    u_ref=np.array(u_hovering.full().flatten())


    # Indexes (Aggiornati per le nuove dimensioni)
    pan_ind = slice(0,1) # pan
    visual_ind = slice(pan_ind.stop,pan_ind.stop+3) # X_c, Y_c, Z_c
    vel_ind = slice(visual_ind.stop,visual_ind.stop+3)
    rp_ind = slice(vel_ind.stop, vel_ind.stop+2)
    #quat_ind = slice(vel_ind.stop, vel_ind.stop+4)
    dot_rpy_ind = slice(rp_ind.stop,rp_ind.stop+3)
    acc_ind = slice(dot_rpy_ind.stop,dot_rpy_ind.stop+3)
    acc_ang_ind = slice(acc_ind.stop,acc_ind.stop+3)
    jerk_ind = slice(acc_ang_ind.stop,acc_ang_ind.stop+3)   
    snap_ind = slice(jerk_ind.stop,jerk_ind.stop+3)
    u_ind = slice(snap_ind.stop,snap_ind.stop+4)

    y_idx = {
        "pan": pan_ind,
        "visual": visual_ind,
        "vel": vel_ind,
        "rp": rp_ind,
        "dot_rpy": dot_rpy_ind,
        "acc": acc_ind,
        "acc_ang": acc_ang_ind,
        "jerk": jerk_ind,
        "snap": snap_ind,
        "u": slice(snap_ind.stop, snap_ind.stop+4),
    }
    ny   = y_idx["u"].stop   
    ny_e = y_idx["u"].start  
    
    yref = np.zeros(y_expr.numel())
    yref_e = np.zeros(y_expr_e.numel())

    # ASSIGN REFERENCES
    yref[pan_ind]= pan_ref              # Target Pan
    yref[visual_ind]=visual_ref
    yref[vel_ind]=v_ref
    yref[rp_ind]= rp_ref
    yref[dot_rpy_ind]=dot_rpy_ref
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