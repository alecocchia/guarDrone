#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPC_planner_node.py — PLANNER ONLY / CONTROLLER
- Configurazione MPC in configure_mpc(), chiamata allo start (/peg_pose).
- Risoluzione MPC in solve_MPC(xk), richiamata nel timer.
- GESTIONE PX4 INTEGRATA: Pubblica OffboardControlMode in modo sincrono a Thrust e Torque.
- SICUREZZA: Attende la prima odometria valida da PX4 prima di calcolare i setpoint.

Dipendenze progetto: drone_MPC_settings.py, MPC_main.py, common.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Wrench
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64MultiArray

# --- PX4 MESSAGES IMPORTS ---
from px4_msgs.msg import VehicleOdometry, VehicleThrustSetpoint, VehicleTorqueSetpoint, OffboardControlMode, VehicleCommand, VehicleControlMode
import numpy as np
import casadi as ca
from casadi import pi as pi
from scipy.spatial.transform import Rotation
import time

from drone_mpc_pkg.drone_MPC_settings import (
    setup_model, setup_initial_conditions, configure_mpc, set_initial_state, build_yref_online
)
from drone_mpc_pkg.common import quat_to_RPY, g0

import tf2_ros


class MpcPlannerNode(Node):
    def __init__(self):
        super().__init__('mpc_planner_node')

        # === Modello e condizioni iniziali ===
        self.declare_parameter('mass', 2.064)
        self.declare_parameter('ixx', 0.0216)
        self.declare_parameter('iyy', 0.0216)
        self.declare_parameter('izz', 0.040)
        self.declare_parameter('cf', 8.0e-4)
        self.declare_parameter('ct', 1.0e-5)
        self.declare_parameter('f_max', 34.0)
        self.declare_parameter('w_min', 150.0)
        self.declare_parameter('w_max', 1000.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('start_roll', 0.0)
        self.declare_parameter('start_pitch', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('cam_x',0.0)
        self.declare_parameter('cam_y',0.0)
        self.declare_parameter('cam_z',0.0)
        self.declare_parameter('cam_roll',0.0)
        self.declare_parameter('cam_pitch',0.0)
        self.declare_parameter('cam_yaw',0.0)
        self.declare_parameter('fov_h', 80.0)
        self.declare_parameter('fov_v', 60.0)
        self.declare_parameter('arm_l_x', 0.174)
        self.declare_parameter('arm_l_y', 0.174)
        self.declare_parameter('moment_const', 0.016)
        self.declare_parameter('use_momentum_observer', True)

        mass = self.get_parameter('mass').value
        ixx = self.get_parameter('ixx').value
        iyy = self.get_parameter('iyy').value
        izz = self.get_parameter('izz').value
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        start_z = self.get_parameter('start_z').value
        start_roll = self.get_parameter('start_roll').value
        start_pitch = self.get_parameter('start_pitch').value
        start_yaw = self.get_parameter('start_yaw').value
        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        cam_roll = self.get_parameter('cam_roll').value
        cam_pitch = self.get_parameter('cam_pitch').value
        cam_yaw = self.get_parameter('cam_yaw').value

        self.get_logger().info(f"Parametri caricati: m={mass}, I=[{ixx}, {iyy}, {izz}]")

        self.mass = mass
        self.Ixx = ixx
        self.Iyy = iyy
        self.Izz = izz

        self.model, self.model_rpy = setup_model(mass, ixx, iyy, izz)
        self.x0, self.x0_rpy = setup_initial_conditions(start_x,start_y,start_z,start_roll,start_pitch,start_yaw)
        # === Tempo/Orizzonte ===
        self.Tf = 25.0
        num_campioni = 20
        self.ts = 0.01  # 100 Hz
        self.Tp = num_campioni*self.ts
        self.ts_peg = 0.005
        self.N_horiz = int(self.Tf / self.ts)
        self.path_pub_counter = 0  # Contatore per limitare la frequenza di pubblicazione del path

        self.t_prev = 0.0

        # === Stato MPC / loop ===
        self.acados_solver_ready = False
        self.path_received = False
        self.start_received = False
        self.first_odom_received = False  
        self.planner_ready_published = False   
        self.startup_counter = 0               
        self.k = 0
        self.mpc_path_published = False

        self.u_prev = None
        self.x_prev = None
        self.last_u0 = None
        self.px4_timestamp = 0 # OROLOGIO SINCRONIZZATO CON PX4

        self.p_obj = None
        self.rpy_obj = None
        self.camera_offset = np.array([cam_x, cam_y, cam_z])
        self.camera_rpy = np.array([cam_roll, cam_pitch, cam_yaw])

        # --- Momentum Observer  (Ruggiero style) ---
        self.p_momentum = mass * np.zeros(3)  # Linear momentum integrated
        self.L_momentum = np.zeros(3)         # Angular momentum integrated
        self.f_ext_est  = np.zeros(3)         # Estimated external force (world frame)
        self.tau_ext_est = np.zeros(3)        # Estimated external torque (body frame)
        self.K_f = 0.1                        # Observer gain linear
        self.K_tau = 0.5                      # Observer gain angular
        self.J_matrix = np.diag([ixx, iyy, izz])
        self.use_momentum_observer = self.get_parameter('use_momentum_observer').value
        
        self.fov_h = self.get_parameter('fov_h').value
        self.fov_v = self.get_parameter('fov_v').value

        # Target visivo di default (PoV: Xc, Yc, Zc, Pan)
        self.pov_target = np.array([2.0, 0.0, 0.0, 0.0])

        self.declare_parameter('control_flag',  1)  
        self.control_flag_val = self.get_parameter('control_flag').get_parameter_value().integer_value
        
        # --- PUBLISHERS COMANDI E PX4 INTEGRATION ---
        self.is_armed = False      
        self.is_offboard = False   

        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, px4_qos_profile
        )
        
        # PUBBLICATORI PER IL LOGGER
        self.wrench_cmd_pub = self.create_publisher(Wrench, '/wrench_cmd', 1)
        self.wrench_ref_pub = self.create_publisher(Wrench, '/wrench_reference', 1) # <--- Nuova

        self.single_wrench_pub = self.create_publisher(Wrench, '/optimal_wrench', 1)

        if self.control_flag_val == 1:
            self.get_logger().info("MPC in modalità PX4 Controller Integrato: attiva pub offboard, thrust e torque.")
            self.thrust_pub = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', 1)
            self.torque_pub = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', 1)
            self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 1)
            self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 1)
        else:
            self.get_logger().info("MPC pubblica Wrench standard su: /optimal_wrench")


        self.current_position = np.zeros(3)
        self.current_rpy = np.zeros(3)
        self.current_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.current_raw_vel = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.current_ang_vel = np.zeros(3)

        qos_latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.ready_publisher  = self.create_publisher(Bool, '/drone_planner_ready',  qos_latched)
        self.optimal_path_pub = self.create_publisher(Path, '/optimal_drone_path', qos_latched)

        self.peg_path_subscription = self.create_subscription(
            Path, '/peg_path', self.peg_path_callback, qos_latched)
        

        self.odom_subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, px4_qos_profile
        )

        self.single_pose_pub  = self.create_publisher(PoseStamped,  '/optimal_drone_pose',  1)
        self.drone_pose_pub   = self.create_publisher(PoseStamped,  '/drone_pose',          1) # <--- Per RViz
        self.single_twist_pub = self.create_publisher(TwistStamped, '/optimal_drone_twist', 1)
        self.tf_broadcaster   = tf2_ros.TransformBroadcaster(self)
        self.ref_pub = self.create_publisher(Float64MultiArray, '/online_spherical_ref', 1)
        self.visual_ref_pub = self.create_publisher(Float64MultiArray, '/online_visual_ref', 1)

        self.control_timer = self.create_timer(self.ts, self.control_step)
        self.start_subscription = self.create_subscription(PoseStamped, '/peg_pose', self.start_callback, 10)
        self.pov_target_sub = self.create_subscription(Float64MultiArray, '/pov_target', self.pov_target_callback, 10)

        self.get_logger().info("MPC Node avviato. In attesa...")

    # ==================== Callbacks I/O ====================

    def control_mode_callback(self, msg: VehicleControlMode):
        self.is_armed = msg.flag_armed
        self.is_offboard = msg.flag_control_offboard_enabled

    def peg_path_callback(self, msg: Path):
        p_obj_list, rpy_obj_list = [], []
        count = 0
        times_ratio = max(1, int(round(self.ts / self.ts_peg)))

        for pose_stamped in msg.poses:
            if count % times_ratio == 0:
                p = pose_stamped.pose.position
                q = pose_stamped.pose.orientation
                rpy = quat_to_RPY([q.w, q.x, q.y, q.z])  
                p_obj_list.append([p.x, p.y, p.z])
                rpy_obj_list.append(np.squeeze(np.array(rpy)))
            count += 1

        self.p_obj = np.array(p_obj_list)
        self.rpy_obj = np.squeeze(np.array(rpy_obj_list))
        self.path_received = True


    def planner_configure(self):
        if self.acados_solver_ready == True:
            return
        
        # prima configurazione dell' MPC
        self.configure_mpc()
        
        # se questo nodo viene usato solo come planner allora è pronto e può partire già
        if self.control_flag_val != 1 and not self.planner_ready_published:
            self.ready_publisher.publish(Bool(data=True))
            self.planner_ready_published = True

        if self.acados_solver_ready :
            self.publish_predicted_path_from_buffers()
        
        self.acados_solver_ready=True

    def start_callback(self, _msg: PoseStamped):
        if self.start_received:
            return
        self.start_received = True
        self.get_logger().info("Ricevuta posa iniziale del peg.")
        # Il timer è già stato avviato in peg_path_callback
        self.destroy_subscription(self.start_subscription)


    def odom_callback(self, msg: VehicleOdometry):
        # Quaternione PX4: rappresenta R_frd2ned (body FRD → world NED)
        # SciPy usa l'ordine [x, y, z, w]
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]] 
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

        # Matrici fisse di conversione frame
        M_ned2enu = np.array([[0.0, 1.0, 0.0], 
                              [1.0, 0.0, 0.0], 
                              [0.0, 0.0, -1.0]])
                              
        M_frd2flu = np.array([[1.0, 0.0, 0.0], 
                              [0.0, -1.0, 0.0], 
                              [0.0, 0.0, -1.0]])

        # R_flu2enu (body→world per MPC): catena FLU→FRD→NED→ENU
        # M_frd2flu ortogonale, quindi M_flu2frd = M_frd2flu
        R_flu2enu = M_ned2enu @ R_frd2ned @ M_frd2flu
        
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        q_flu2enu = rot_flu2enu.as_quat() # [x, y, z, w] (formato scipy)

        # Posizione: NED → ENU (M_ned2enu @ [N,E,D] = [E,N,-D])
        self.current_position[:] = M_ned2enu @ np.array([msg.position[0], msg.position[1], msg.position[2]])

        # Assegnazione all'MPC (che si aspetta [w, x, y, z])
        q_w = q_flu2enu[3]
        q_x = q_flu2enu[0]
        q_y = q_flu2enu[1]
        q_z = q_flu2enu[2]
        
        q = [q_w, q_x, q_y, q_z]
        self.current_quat = q / np.linalg.norm(q)    # norma quaternione ad 1
        self.current_rpy[:] = rot_flu2enu.as_euler('xyz')

        # Velocità lineare: NED → ENU
        self.current_raw_vel[:] = M_ned2enu @ np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]]).T
        
        self.current_ang_vel[:] = M_frd2flu @ np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]]).T
        self.px4_timestamp = msg.timestamp # <--- SINCRONIZZAZIONE OROLOGIO
        
        # --- INIZIALIZZAZIONE DINAMICA X0 ---
        if not self.first_odom_received:
            self.x0 = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_raw_vel[0],  self.current_raw_vel[1],  self.current_raw_vel[2],
                q_w, q_x, q_y, q_z,
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2]
            ])
            self.x0_rpy = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_raw_vel[0],  self.current_raw_vel[1],  self.current_raw_vel[2],
                self.current_rpy[0],      self.current_rpy[1],      self.current_rpy[2],
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2]
            ])
            self.get_logger().info(f"Posa iniziale inizializzata da odometria: {self.current_position}")
            self.first_odom_received = True 

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'drone_base_link'
        t.transform.translation.x = float(self.current_position[0])
        t.transform.translation.y = float(self.current_position[1])
        t.transform.translation.z = float(self.current_position[2])
        t.transform.rotation.w = float(q_w)
        t.transform.rotation.x = float(q_x)
        t.transform.rotation.y = float(q_y)
        t.transform.rotation.z = float(q_z)
        self.tf_broadcaster.sendTransform(t)

        # Pubblicazione posa reale per RViz
        drone_pose_msg = PoseStamped()
        drone_pose_msg.header = t.header
        drone_pose_msg.pose.position.x = t.transform.translation.x
        drone_pose_msg.pose.position.y = t.transform.translation.y
        drone_pose_msg.pose.position.z = t.transform.translation.z
        drone_pose_msg.pose.orientation = t.transform.rotation
        self.drone_pose_pub.publish(drone_pose_msg)

    def pov_target_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.pov_target = np.array([msg.data[0], msg.data[1], msg.data[2], msg.data[3]], dtype=float)

    # ==================== Configurazione e Solve ====================
    def configure_mpc(self):

        # g0 importato da common.py

        X = 2; Y = 2; Z = 5; V = 5.0; QUAT = 1            
        ANG_DOT = 3.0; ACC = 10.0; ACC_ANG = 11.0     
        JERK = 20.0; SNAP = 200.0
        U_F = self.get_parameter('f_max').value
        
        # Le accelerazioni angolari massime teoriche (rad/s^2) dipendono dalla fisica del drone.
        # Per un quadricottero tipico, l'autorità di roll/pitch è altissima, quella di yaw è molto bassa.
        acc_ang_max_xy = 30.0 # rad/s^2
        acc_ang_max_z = 5.0  # rad/s^2
        
        # Definiamo U_TAU come la coppia fisica massima 
        U_TAU_XY = self.Ixx * acc_ang_max_xy
        U_TAU_Z  = self.Izz * acc_ang_max_z

        PesoVis = 10
        PesoPan = PesoVis/2
        PesoRot = PesoVis
        PesoVel = PesoVis / 20
        PesoAngVel = PesoRot / 10
        PesoAcc = PesoVel * 2
        PesoAngAcc = PesoAngVel * 2
        PesoJerk = PesoAcc 
        PesoSnap = PesoJerk
        PesoForce = PesoPan / 100
        PesoTorque = PesoForce  * 2

        Q_pan = np.diag([PesoPan]) / (ca.pi)**2
        Q_visual = np.diag([PesoVis,PesoVis,PesoVis]) / np.array([X,Y,Z])**2 
        Q_vel = np.diag([PesoVel, PesoVel, PesoVel/4]) / V**2
        Q_rot = np.diag([PesoRot, PesoRot]) / QUAT**2  
        
        Q_ang_dot = np.diag([PesoAngVel, PesoAngVel, PesoAngVel/4]) / ANG_DOT**2
        Q_acc = np.diag([PesoAcc, PesoAcc, PesoAcc/10]) / ACC**2
        Q_acc_ang = np.diag([PesoAngAcc, PesoAngAcc, PesoAngAcc/10]) / ACC_ANG**2
        Q_jerk = np.diag([PesoJerk, PesoJerk, PesoJerk]) / JERK**2
        Q_snap = np.diag([PesoSnap, PesoSnap, PesoSnap]) / SNAP**2
        
        R_f = np.diag([PesoForce]) / U_F**2
        R_tau = ca.diagcat(PesoTorque / U_TAU_XY**2, PesoTorque / U_TAU_XY**2, PesoTorque / U_TAU_Z**2)
        
        R = ca.diagcat(R_f, R_tau)
        Q = ca.diagcat(Q_pan, Q_visual, Q_vel, Q_rot, Q_ang_dot, Q_acc, Q_acc_ang, Q_jerk, Q_snap)

        # Definiamo i limiti fisici reali da passare al solver
        u_min = np.array([0.0, -U_TAU_XY, -U_TAU_XY, -U_TAU_Z])
        u_max = np.array([U_F, U_TAU_XY, U_TAU_XY, U_TAU_Z]) 

        W   = ca.diagcat(Q, R).full()
        W_e = 20* Q.full()

        (self.ocp_solver, self.N_horiz, self.nx, self.nu, self.y_idx, self.ny, self.ny_e) = configure_mpc(
            model=self.model, x0=self.x0, camera_offset=self.camera_offset,
            p_obj=self.p_obj, rpy_obj=self.rpy_obj, Tf=self.Tp, ts=self.ts,
            W=W, W_e=W_e, u_min=u_min, u_max=u_max,
            pan_ref=0.0, visual_ref = np.array([2.0,0.0,0.0]),
            cam_rpy=self.camera_rpy, fov_h=self.fov_h, fov_v=self.fov_v
        )

        self.u_hover = np.array([self.get_parameter('mass').value * g0, 0.0, 0.0, 0.0])
        
        self.u_prev = [self.u_hover.copy() for _ in range(self.N_horiz)]        
        self.x_prev = [self.x0.copy() for _ in range(self.N_horiz+1)]

        for i in range(self.N_horiz):
            self.ocp_solver.set(i, "u", self.u_prev[i])
            self.ocp_solver.set(i, "x", self.x_prev[i])
        self.ocp_solver.set(self.N_horiz, "x", self.x_prev[self.N_horiz])

        self.k = 0 
        self.publish_predicted_path_from_buffers()

    def solve_MPC(self, xk, pov_target):
        yref0 = None
        set_initial_state(self.ocp_solver, xk)
        t0_idx = self.k
        M = len(self.p_obj)

        online_visual_ref = pov_target[0:3]
        pan_target = pov_target[3]

        # Calcoliamo il pan attuale rispetto all'oggetto per il wrapping
        p_obj_now = self.p_obj[t0_idx]
        current_pan = np.arctan2(xk[1] - p_obj_now[1], xk[0] - p_obj_now[0])

        # SHORTEST PATH WRAP: evitiamo che il drone faccia un giro di 350 gradi 
        # quando attraversa la barriera di pi/-pi.
        diff_pan = (pan_target - current_pan + np.pi) % (2 * np.pi) - np.pi
        pan_target = current_pan + diff_pan

        # PROTEZIONE DECOLLO: Se il drone è molto basso (< 0.6m), forziamo il pan_target
        # a quello attuale per evitare inclinazioni violente dovute all'orbita.
        if self.current_position[2] < 0.6:
            pan_target = current_pan

        for i in range(self.N_horiz + 1):
            idx = min(t0_idx + i, M - 1)
            p_i   = self.p_obj[idx]
            
            params = np.zeros(9)
            params[0:3] = p_i               # Target object position
            params[3:6] = self.f_ext_est    # Est. external force (world)
            params[6:9] = self.tau_ext_est  # Est. external torque (body)
            
            self.ocp_solver.set(i, "p", params)
            
            if i < self.N_horiz:
                yref_val = build_yref_online(self.y_idx, pan_target, online_visual_ref, self.u_hover)
                self.ocp_solver.set(i, "yref", yref_val)
                if i == 0: yref0 = yref_val
            elif i == self.N_horiz:
                self.ocp_solver.set(self.N_horiz, "yref", build_yref_online(self.y_idx, pan_target, online_visual_ref, self.u_hover)[:self.ny_e])

        status = self.ocp_solver.solve()
        if status != 0:
            u0=self.u_prev[0].copy()
            x_seq = [self.x_prev[i].copy() for i in range(self.N_horiz + 1)]
            for i in range(self.N_horiz - 1):
                self.u_prev[i] = self.u_prev[i+1].copy()
                self.x_prev[i] = self.x_prev[i+1].copy()
            if self.N_horiz > 1:
                self.u_prev[self.N_horiz - 1] = self.u_prev[self.N_horiz - 2].copy()
            else:
                self.u_prev[0] = self.u_prev[-1].copy() 
            self.x_prev[self.N_horiz] = self.x_prev[-1].copy()
            return u0, x_seq, yref0

        u0 = self.ocp_solver.get(0, "u")
        x_seq = [self.ocp_solver.get(i, "x") for i in range(self.N_horiz + 1)]


        for i in range(self.N_horiz - 1):
            self.u_prev[i] = self.ocp_solver.get(i + 1, "u")
            self.x_prev[i] = self.ocp_solver.get(i + 1, "x")
        if self.N_horiz > 1:
            self.u_prev[self.N_horiz - 1] = self.u_prev[self.N_horiz - 2].copy()
        else:
            self.u_prev[0] = self.ocp_solver.get(0, "u").copy()
        self.x_prev[self.N_horiz] = self.ocp_solver.get(self.N_horiz, "x")

        return u0, x_seq, yref0

    # ==================== Ciclo planner ====================
    def control_step(self):
        if not self.path_received :
            self.get_logger().info("In attesa della ricezione del percorso del peg da peg_planner_node...", throttle_duration_sec=2.0)
            return
        
        if not self.first_odom_received:
            self.get_logger().info("In attesa della prima odometria da PX4...", throttle_duration_sec=2.0)
            return

        if not (self.acados_solver_ready):
            self.get_logger().info("Configurazione solver ACADOS con posa iniziale reale...", throttle_duration_sec=2.0)
            self.planner_configure()
            return

        # --- Momentum Observer (Fabio Ruggiero style) ---
        self.use_momentum_observer = self.get_parameter('use_momentum_observer').value
        if self.use_momentum_observer and self.u_prev is not None and len(self.u_prev) > 0:
            # Se disarmato, la forza/coppia prodotta è zero. 
            # Usiamo self.is_armed (aggiornato da VehicleControlMode) per la coerenza online.
            u_actual = self.u_prev[0] if self.is_armed else np.zeros(4)

            # 1. Osservatore Lineare (World Frame)
            # thrust_body = [0, 0, Thrust]
            thrust_body = np.array([0, 0, u_actual[0]])
            # Ruotiamo la spinta nel frame mondo usando il quaternione attuale [w, x, y, z]
            q = self.current_quat
            Rb = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
            thrust_world = Rb @ thrust_body
            
            # Dinamica del momento lineare: p_dot = f_thrust + f_ext - m*g
            self.p_momentum += (thrust_world - self.mass * np.array([0, 0, g0]) + self.f_ext_est) * self.ts
            self.f_ext_est = self.K_f * (self.mass * self.current_vel - self.p_momentum)
            
            # 2. Osservatore Angolare (Body Frame)
            # Dinamica del momento angolare: L_dot = tau_cmd - w x (J w) + tau_ext
            tau_cmd = u_actual[1:4]
            w = self.current_ang_vel
            coriolis_tau = np.cross(w, self.J_matrix @ w)
            self.L_momentum += (tau_cmd - coriolis_tau + self.tau_ext_est) * self.ts
            self.tau_ext_est = self.K_tau * (self.J_matrix @ w - self.L_momentum)
        else:
            # Se disabilitato, resettiamo le stime a zero per non influenzare l'MPC
            self.f_ext_est = np.zeros(3)
            self.tau_ext_est = np.zeros(3)
            # Resettiamo anche gli integratori del momento per evitare "salti" alla riattivazione
            self.p_momentum = self.mass * self.current_vel
            self.L_momentum = self.J_matrix @ self.current_ang_vel

        self.R = Rotation.from_euler('xyz',self.current_rpy).as_matrix()
        self.current_vel[:] = self.current_raw_vel[:]

        # Stato xk a 13 elementi (senza integratori)
        xk = np.array([
            self.current_position[0], self.current_position[1], self.current_position[2],
            self.current_vel[0], self.current_vel[1], self.current_vel[2],
            self.current_quat[0], self.current_quat[1], self.current_quat[2], self.current_quat[3],
            self.current_ang_vel[0], self.current_ang_vel[1], self.current_ang_vel[2]
        ])

        # Pubblicazione riferimenti per RViz/Logger (manteniamo formato compatibile se necessario)
        # Il pov_target è [Xc, Yc, Zc, Pan]
        online_visual_ref = self.pov_target[0:3]
        pan_target = self.pov_target[3]
        
        # Per mantenere la compatibilità col logger creiamo uno spherical fake: [Xc, Pan, 0]
        online_spherical_ref = np.array([online_visual_ref[0], pan_target, 0.0])
        ref_msg = Float64MultiArray(data=[float(x) for x in online_spherical_ref])
        self.ref_pub.publish(ref_msg)
        self.visual_ref_pub.publish(Float64MultiArray(data=[float(x) for x in online_visual_ref]))

        t_start = time.perf_counter()
        u0, x_seq, yref0 = self.solve_MPC(xk, self.pov_target)
        t_end = time.perf_counter()
        if (t_end - t_start) > self.ts:
            self.get_logger().info(f"Solver time: {(t_end - t_start) * 1000:.2f} ms")
        
        # PUBBLICAZIONE OUTPUT
        if x_seq is not None and len(x_seq) >= 2:
            self.publish_pose_and_twist(x_seq[1])

        if u0 is not None and len(u0) >= 2:
            self.publish_optimal_wrench(u0)
            
            # Pubblichiamo anche il riferimento (yref) usato dall'ottimizzatore per plotting
            if yref0 is not None:
                yref_u = yref0[self.y_idx["u"]]
                w_ref_msg = Wrench()
                w_ref_msg.force.z = float(yref_u[0])
                w_ref_msg.torque.x = float(yref_u[1])
                w_ref_msg.torque.y = float(yref_u[2])
                w_ref_msg.torque.z = float(yref_u[3])
                self.wrench_ref_pub.publish(w_ref_msg)

        # Pubblicazione del path ottimale a 10Hz (ogni 5 cicli a 50Hz)
        self.path_pub_counter += 1
        if x_seq is not None and self.path_pub_counter >= 5:
            self.publish_predicted_path(x_seq)
            self.path_pub_counter = 0

        if self.planner_ready_published:
            self.k = min(self.k + 1, len(self.p_obj) - 1)

    # ==================== Funzioni PX4 Controllo Integrato ====================

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.timestamp = self.px4_timestamp # SYNC CON PX4
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.px4_timestamp # SYNC CON PX4
        self.vehicle_command_publisher.publish(msg)

    def manage_offboard_state(self):
        self.startup_counter += 1
        if self.startup_counter < 50:
            return
            
        if self.startup_counter % 50 == 0:
            if not self.is_offboard:
                self.get_logger().info("Setpoints stabili! Richiesta Offboard a PX4...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif not self.is_armed:
                self.get_logger().info("Siamo in Offboard! Armamento motori...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                if not self.planner_ready_published:
                    self.get_logger().info("Decollo completato! Sblocco il movimento del peg in Gazebo.")
                    self.ready_publisher.publish(Bool(data=True))
                    self.planner_ready_published = True

    # ==================== Pubblicazione ====================

    def publish_pose_and_twist(self, x_vec):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(x_vec[0])
        pose_msg.pose.position.y = float(x_vec[1])
        pose_msg.pose.position.z = float(x_vec[2])
        quat = x_vec[6:10]  
        pose_msg.pose.orientation.w = float(quat[0])
        pose_msg.pose.orientation.x = float(quat[1])
        pose_msg.pose.orientation.y = float(quat[2])
        pose_msg.pose.orientation.z = float(quat[3])
        self.single_pose_pub.publish(pose_msg)

        # Aggiunta pubblicazione Twist ottimale per il logger
        twist_msg = TwistStamped()
        twist_msg.header = pose_msg.header
        twist_msg.twist.linear.x = float(x_vec[3])
        twist_msg.twist.linear.y = float(x_vec[4])
        twist_msg.twist.linear.z = float(x_vec[5])
        twist_msg.twist.angular.x = float(x_vec[10])
        twist_msg.twist.angular.y = float(x_vec[11])
        twist_msg.twist.angular.z = float(x_vec[12])
        self.single_twist_pub.publish(twist_msg)

    def publish_optimal_wrench(self, u0) :
        # FIX LOGGER: Pubblichiamo sempre il wrench calcolato per passarlo al logger
        wrench_msg = Wrench()
        wrench_msg.force.z = float(u0[0])
        wrench_msg.torque.x = float(u0[1])
        wrench_msg.torque.y = float(u0[2])
        wrench_msg.torque.z = float(u0[3])
        self.wrench_cmd_pub.publish(wrench_msg)

        # Pubblichiamo SEMPRE su /optimal_wrench per il logger
        self.single_wrench_pub.publish(wrench_msg)

        if self.control_flag_val == 1:
            self.publish_offboard_control_mode()
 
            
            f_max = self.get_parameter('f_max').value
            w_min = self.get_parameter('w_min').value
            w_max = self.get_parameter('w_max').value
            
            # Linearizzazione del comando: Gazebo è quadratico (T = k*w^2)
            # Calcoliamo la velocità angolare desiderata [0, w_max] rad/s
            w_target = w_max * np.sqrt(max(0.0, float(u0[0])) / f_max)
            # Mappatura su norm_thrust [0, 1] considerando il range dell'airframe [w_min, w_max]
            norm_thrust = (w_target - w_min) / (w_max - w_min)
            norm_thrust = max(0.0, min(1.0, norm_thrust)) 
            #norm_thrust = max(0.0, min(1.0, u0[0]/f_max))
            
            thrust_msg = VehicleThrustSetpoint()
            thrust_msg.timestamp = self.px4_timestamp
            thrust_msg.xyz[0] = 0.0
            thrust_msg.xyz[1] = 0.0
            thrust_msg.xyz[2] = -norm_thrust 
            self.thrust_pub.publish(thrust_msg)

            # Calcoliamo U_TAU corrispondente al 100% dell'autorità di PX4.
            # U_TAU_XY: Coppia massima prodotta da 2 motori su 4 (quad x)
            # U_TAU_Z:  Coppia prodotta dal trascinamento (moment_constant)
            arm_l = max(self.get_parameter('arm_l_x').value, self.get_parameter('arm_l_y').value)
            moment_const = self.get_parameter('moment_const').value
            
            U_TAU_XY = arm_l * f_max / 2.0
            U_TAU_Z  = moment_const * f_max
            
            torque_msg = VehicleTorqueSetpoint()
            torque_msg.timestamp = self.px4_timestamp # SYNC CON PX4
            
            # Segni FLU -> FRD (PX4): 
            # Roll (X): CCW FLU = CCW FRD (stesso verso) -> u0[1]
            # Pitch (Y): CCW FLU = CW FRD (opposto) -> -u0[2]
            # Yaw (Z): CCW FLU = CW FRD (opposto) -> -u0[3]
            torque_msg.xyz[0] = float(np.clip((u0[1]/U_TAU_XY),-1.0, 1.0)) 
            torque_msg.xyz[1] = -float(np.clip((u0[2]/U_TAU_XY),-1.0, 1.0))
            torque_msg.xyz[2] = -float(np.clip((u0[3]/U_TAU_Z),-1.0, 1.0))
            self.torque_pub.publish(torque_msg)

            self.manage_offboard_state()
            
        else:
            self.single_wrench_pub.publish(wrench_msg)

    def publish_predicted_path(self, x_seq):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"
        for xi in x_seq:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(xi[0])
            ps.pose.position.y = float(xi[1])
            ps.pose.position.z = float(xi[2])
            quat = xi[6:10]
            ps.pose.orientation.w = float(quat[0])
            ps.pose.orientation.x = float(quat[1])
            ps.pose.orientation.y = float(quat[2])
            ps.pose.orientation.z = float(quat[3])
            path_msg.poses.append(ps)
        self.optimal_path_pub.publish(path_msg)

    def publish_predicted_path_from_buffers(self):
        if self.x_prev is not None:
            self.publish_predicted_path(self.x_prev)

def main(args=None):
    rclpy.init(args=args)
    node = MpcPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()