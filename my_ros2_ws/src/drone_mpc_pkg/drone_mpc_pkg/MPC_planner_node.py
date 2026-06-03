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
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float64MultiArray, String

# --- PX4 MESSAGES IMPORTS ---
from px4_msgs.msg import VehicleOdometry, VehicleThrustSetpoint, VehicleTorqueSetpoint, OffboardControlMode, VehicleCommand, VehicleControlMode
import numpy as np
import casadi as ca
from casadi import pi as pi
from scipy.spatial.transform import Rotation
import time
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from drone_mpc_pkg.drone_MPC_settings import (
    setup_model, setup_initial_conditions, configure_mpc, set_initial_state, build_yref_online
)
from drone_mpc_pkg.common import quat_to_RPY, g0

import tf2_ros


class MpcPlannerNode(Node):
    def __init__(self):
        super().__init__('mpc_planner_node')

        # === Threading e Callback Groups ===
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.solver_is_running = False

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
        self.declare_parameter('peg_x', 0.0)
        self.declare_parameter('peg_y', 0.0)
        self.declare_parameter('peg_z', 0.0)
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
        self.declare_parameter('rp_limit', 45.0)

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
        self.peg_offset = np.array([
            self.get_parameter('peg_x').value,
            self.get_parameter('peg_y').value,
            self.get_parameter('peg_z').value
        ])
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
        self.camera_offset = np.array([cam_x, cam_y, cam_z])
        self.camera_rpy = np.array([cam_roll, cam_pitch, cam_yaw])

        self.model, self.model_rpy = setup_model(mass, ixx, iyy, izz, self.camera_offset, self.camera_rpy)
        self.x0, self.x0_rpy = setup_initial_conditions(start_x,start_y,start_z,start_roll,start_pitch,start_yaw)
        # === Tempo/Orizzonte ===
        self.ts = 0.01             # 100 Hz
        self.N_horiz = 80          # Orizzonte di predizione (numero di campioni)
        self.Tp = self.N_horiz * self.ts  # Tempo totale dell'orizzonte 
        self.ts_peg = 0.005

        self.path_pub_counter = 0  # Contatore per limitare la frequenza di pubblicazione del path

        self.t_prev = 0.0

        # === Stato MPC / loop ===
        self.acados_solver_ready = False
        self.obj_state_received = False
        self.start_received = False
        self.first_odom_received = False  
        self.planner_ready_published = False   
        self.startup_counter = 0               

        self.mpc_path_published = False

        self.u_prev = None
        self.x_prev = None
        self.last_u0 = None
        self.last_u0_applied = None  # Controllo effettivamente inviato al drone al passo precedente
        self.px4_odom_timestamp_us = 0   # Timestamp corrente dall'odometria PX4 (µs)
        self.last_odom_timestamp_us = 0  # Ultimo timestamp processato dal control loop (µs)
        self.armed_counter = 0           # Contatore per attendere il decollo effettivo


        self.current_obj_pos = np.zeros(3)
        self.current_obj_vel = np.zeros(3)
        self.current_obj_ang_vel = np.zeros(3)
        self.current_obj_rpy = np.zeros(3)

        self.fov_h = self.get_parameter('fov_h').value
        self.fov_v = self.get_parameter('fov_v').value
        self.declare_parameter('haptic_transition_duration', 3.0)
        self.haptic_transition_duration = self.get_parameter('haptic_transition_duration').value

        # Target visivo di default (PoV: Xc, Yc, Zc, Pan)
        self.pan_target = np.pi/2
        self.radius_target = 3.0
        self.pov_target = np.array([self.radius_target, 0.0, 0.0, self.pan_target])
        xc_hand = 0.5
        yc_hand = -0.3
        zc_hand = -0.5
        scale_factor = 1.0
        self.final_target=np.array([xc_hand, yc_hand, zc_hand]) * scale_factor #aggiungere pan

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
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, px4_qos_profile, callback_group=self.callback_group
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

        self.peg_odom_subscription = self.create_subscription(
            VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', self.peg_odom_callback, px4_qos_profile, callback_group=self.callback_group)
        

        self.odom_subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, px4_qos_profile, callback_group=self.callback_group
        )

        self.single_pose_pub  = self.create_publisher(PoseStamped,  '/optimal_drone_pose',  1)
        self.drone_pose_pub   = self.create_publisher(PoseStamped,  '/drone_pose',          1) # <--- Per RViz
        self.peg_pose_pub     = self.create_publisher(PoseStamped,  '/peg_pose',            1) # <--- Per RViz (Peg)
        self.single_twist_pub = self.create_publisher(TwistStamped, '/optimal_drone_twist', 1)
        self.vel_ref_pub      = self.create_publisher(TwistStamped, '/velocity_reference',  1)
        self.tf_broadcaster   = tf2_ros.TransformBroadcaster(self)
        self.ref_pub = self.create_publisher(Float64MultiArray, '/online_spherical_ref', 1)
        self.visual_ref_pub = self.create_publisher(Float64MultiArray, '/online_visual_ref', 1)
        self.actual_pov_pub = self.create_publisher(Float64MultiArray, '/actual_pov', 1)

        self.control_timer = self.create_timer(self.ts, self.control_step, callback_group=self.callback_group)
        self.start_subscription = self.create_subscription(PoseStamped, '/peg_pose', self.start_callback, 10, callback_group=self.callback_group)
        self.pov_target_sub = self.create_subscription(Float64MultiArray, '/pov_target', self.pov_target_callback, 10, callback_group=self.callback_group)
        self.haptic_ref_sub = self.create_subscription(Float64MultiArray, '/haptic_ref', self.haptic_ref_callback, 10, callback_group=self.callback_group)

        # Haptic state
        self.haptic_pov = None
        self.haptic_pov_dot = None
        self.haptic_timestamp = None

        # Joy state
        self.joy_ref_sub = self.create_subscription(Float64MultiArray, '/joy_ref', self.joy_ref_callback, 10, callback_group=self.callback_group)
        self.joy_pov = None
        self.joy_pov_dot = None
        self.joy_timestamp = None
        self.declare_parameter('joy_transition_duration', 3.0)
        self.joy_transition_duration = self.get_parameter('joy_transition_duration').value

        # === Integrazione Behavior Tree ===
        self.bt_task_running = False
        self.bt_start_sub = self.create_subscription(
            Bool, '/mpc_task/start', self.bt_start_callback, 10, callback_group=self.callback_group)
        self.bt_status_pub = self.create_publisher(String, '/mpc_task/status', 10)

        self.thrust_out_sub = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            self.thrust_out_cb,
            px4_qos_profile,
            callback_group=self.callback_group
        )
        self.torque_out_sub = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.torque_out_cb,
            px4_qos_profile,
            callback_group=self.callback_group
        )

        self.current_px4_thrust = np.zeros(3)
        self.current_px4_torque = np.zeros(3)
        self.safety_switch_passed = False

        self.get_logger().info("MPC Node avviato. In attesa...")

    # ==================== Callbacks I/O ====================

    def bt_start_callback(self, msg: Bool):
        if msg.data == True:
            self.get_logger().info("Ricevuto comando dal Behavior Tree / Supervisor: Inizio MPC Task!")
            self.bt_task_running = True
            self.task_started = True
            self.planner_configure()

    def control_mode_callback(self, msg: VehicleControlMode):
        with self.state_lock:
            self.is_armed = msg.flag_armed
            self.is_offboard = msg.flag_control_offboard_enabled

    def peg_odom_callback(self, msg: VehicleOdometry):
        # Quaternione PX4: rappresenta R_frd2ned (body FRD → world NED)
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]] 
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

        # Matrici fisse di conversione frame
        M_ned2enu = np.array([[0.0, 1.0, 0.0], 
                              [1.0, 0.0, 0.0], 
                              [0.0, 0.0, -1.0]])
                              
        M_frd2flu = np.array([[1.0, 0.0, 0.0], 
                              [0.0, -1.0, 0.0], 
                              [0.0, 0.0, -1.0]])

        R_flu2enu = M_ned2enu @ R_frd2ned @ M_frd2flu
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        
        with self.state_lock:
            # Posizione: NED → ENU con offset
            pos_enu = M_ned2enu @ np.array([msg.position[0], msg.position[1], msg.position[2]])
            self.current_obj_pos = pos_enu + self.peg_offset
            
            # Velocità: NED → ENU
            self.current_obj_vel[:] = M_ned2enu @ np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
            
            # Velocità angolare: FRD → FLU
            self.current_obj_ang_vel[:] = M_frd2flu @ np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])
            
            # Orientamento in RPY
            self.current_obj_rpy[:] = rot_flu2enu.as_euler('xyz')
            
            self.obj_state_received = True

        # --- Pubblica /peg_pose per RViz ---
        peg_pose_msg = PoseStamped()
        peg_pose_msg.header.stamp = self.get_clock().now().to_msg()
        peg_pose_msg.header.frame_id = 'world'
        peg_pose_msg.pose.position.x = float(self.current_obj_pos[0])
        peg_pose_msg.pose.position.y = float(self.current_obj_pos[1])
        peg_pose_msg.pose.position.z = float(self.current_obj_pos[2])
        
        # Scipy quaternion to msg orientation (w is at index 3 in as_quat, but as_quat returns x,y,z,w?)
        # rot_flu2enu.as_quat() restituisce [x, y, z, w]
        q_flu = rot_flu2enu.as_quat()
        peg_pose_msg.pose.orientation.x = float(q_flu[0])
        peg_pose_msg.pose.orientation.y = float(q_flu[1])
        peg_pose_msg.pose.orientation.z = float(q_flu[2])
        peg_pose_msg.pose.orientation.w = float(q_flu[3])
        
        self.peg_pose_pub.publish(peg_pose_msg)


    def planner_configure(self):
        with self.state_lock:
            if self.acados_solver_ready == True:
                return
            
            # prima configurazione dell' MPC
            self.configure_mpc()
            
            if not self.planner_ready_published:
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


    def thrust_out_cb(self, msg):
        # I valori in uscita da PX4 sono normalizzati [-1, 1]. In NED, Z=-1 significa full thrust verso l'alto.
        self.current_px4_thrust[0] = msg.xyz[0] * self.U_F
        self.current_px4_thrust[1] = msg.xyz[1] * self.U_F
        self.current_px4_thrust[2] = -msg.xyz[2] * self.U_F

    def torque_out_cb(self, msg):
        self.current_px4_torque[0] = msg.xyz[0] * self.U_TAU_X
        self.current_px4_torque[1] = msg.xyz[1] * self.U_TAU_Y
        self.current_px4_torque[2] = msg.xyz[2] * self.U_TAU_Z

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

        with self.state_lock:
            # Posizione: NED → ENU (M_ned2enu @ [N,E,D] = [E,N,-D])
            self.current_position[:] = M_ned2enu @ np.array([msg.position[0], msg.position[1], msg.position[2]])
            # Aggiungiamo lo spawn offset per posizionare il drone globalmente (fix RViz e Planner)
            self.current_position[0] += self.get_parameter('start_x').value
            self.current_position[1] += self.get_parameter('start_y').value
            self.current_position[2] += self.get_parameter('start_z').value

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
            
            self.px4_odom_timestamp_us = msg.timestamp  # Timestamp PX4 in microsecondi
            
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
            with self.state_lock:
                self.pov_target = np.array([msg.data[0], msg.data[1], msg.data[2], msg.data[3]], dtype=float)

    def haptic_ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            with self.state_lock:
                self.haptic_pov = np.array(msg.data[0:4], dtype=float)
                self.haptic_pov_dot = np.array(msg.data[4:8], dtype=float)
                self.haptic_timestamp = self.get_clock().now()

    def joy_ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            with self.state_lock:
                self.joy_pov = np.array(msg.data[0:4], dtype=float)
                self.joy_pov_dot = np.array(msg.data[4:8], dtype=float)
                self.joy_timestamp = self.get_clock().now()

    # ==================== Configurazione e Solve ====================
    def get_current_ref(self, xk):
        """
        Calcola i riferimenti (visuali, pan, velocità) con interpolazione tra 
        Manual (Haptic/Joypad) e Traiettoria Autonoma per una transizione fluida.
        """
        now = self.get_clock().now()
        # soglia di inattività (s): dopo dt_off si passa da riferimento manuale a autonomo
        dt_off = 0.2 

        # 1) Valutazione Haptic
        alpha_h = 1.0
        h_active = False
        if self.haptic_timestamp is not None:
            dt_h = (now - self.haptic_timestamp).nanoseconds / 1e9
            h_active = (dt_h < dt_off)
            if h_active:
                alpha_h = 0.0
            else:
                alpha_h = min(1.0, max(0.0, (dt_h - dt_off) / self.haptic_transition_duration))

        # 2) Valutazione Joypad
        alpha_j = 1.0
        j_active = False
        if self.joy_timestamp is not None:
            dt_j = (now - self.joy_timestamp).nanoseconds / 1e9
            j_active = (dt_j < dt_off)
            if j_active:
                alpha_j = 0.0
            else:
                alpha_j = min(1.0, max(0.0, (dt_j - dt_off) / self.joy_transition_duration))

        # 3) Selezione del riferimento manuale (Haptic ha priorità)
        if h_active:
            alpha = 0.0
            manual_pov = self.haptic_pov
            manual_d_pov = self.haptic_pov_dot
        elif j_active:
            alpha = 0.0
            manual_pov = self.joy_pov
            manual_d_pov = self.joy_pov_dot
        else:
            # Entrambi in transizione o inattivi: usiamo quello più "fresco" (alpha minore)
            if alpha_h < alpha_j:
                alpha = alpha_h
                manual_pov = self.haptic_pov
                manual_d_pov = self.haptic_pov_dot
            else:
                alpha = alpha_j
                manual_pov = self.joy_pov
                manual_d_pov = self.joy_pov_dot

        # 4) Riferimento Autonomo
        auto_visual_ref = self.pov_target[0:3]
        auto_pan_target = self.pov_target[3]

        if manual_pov is None:
            manual_pov = np.concatenate([auto_visual_ref, [auto_pan_target]])
            manual_d_pov = np.zeros(4)
            alpha = 1.0

        # 5) Interpolazione finale
        visual_ref = (1 - alpha) * manual_pov[0:3] + alpha * auto_visual_ref
        
        p_obj_now = self.current_obj_pos
        # pan : atan2(y-y_obj,x-x_obj)
        current_pan = np.arctan2(xk[1] - p_obj_now[1], xk[0] - p_obj_now[0])
        # normalizzo angoli di pan (caso manuale e autonomo) in [-pi, pi]
        diff_m = (manual_pov[3] - current_pan + np.pi) % (2 * np.pi) - np.pi
        diff_a = (auto_pan_target - current_pan + np.pi) % (2 * np.pi) - np.pi

        # Interpolazione su pan e velocità relative comandate
        pan_target = current_pan + (1 - alpha) * diff_m + alpha * diff_a
        d_pov = (1 - alpha) * manual_d_pov

        # Componente lineare (Xc, Yc, Zc) trasformata in mondo
        R_body_world = Rotation.from_quat([xk[7], xk[8], xk[9], xk[6]]).as_matrix() # x,y,z,w
        R_cam_body = Rotation.from_euler('xyz', self.camera_rpy).as_matrix()
        R_world_cam = R_body_world @ R_cam_body
        v_linear_rel_world = R_world_cam @ d_pov[0:3]
        
        # Componente di orbita (Pan) - velocità tangenziale in mondo
        p_rel_xy = xk[0:2] - self.current_obj_pos[0:2]
        dist_xy = np.linalg.norm(p_rel_xy)
        v_orbit_world = np.zeros(3)
        if dist_xy > 0.1:
            # segno - per far ruotare il drone in senso antiorario con un d_pov[3] (pan speed) > 0
            tangent = np.array([-p_rel_xy[1], p_rel_xy[0]]) / dist_xy
            v_orbit_world[0:2] = tangent * (dist_xy * d_pov[3])
        
        vel_ref = self.current_obj_vel + v_orbit_world - v_linear_rel_world

        return visual_ref, pan_target, vel_ref


    def configure_mpc(self):

        # g0 importato da common.py

        X = 1; Y = 2; Z = 2; V = np.array([1, 1, 1.5]); PAN = ca.pi/3
        RP_ANG = 0.1; ANG_DOT = np.array([0.5, 0.5, 1.5])
        ACC = np.array([2.0, 2.0, 4.0]); ACC_ANG = np.array([2.0,2.0,8.0])     
        JERK = 10.0; SNAP = 200.0
        
        
        f_max = self.get_parameter('f_max').value
        arm_l_x = self.get_parameter('arm_l_x').value
        arm_l_y = self.get_parameter('arm_l_y').value
        moment_const = self.get_parameter('moment_const').value
        
        self.U_F = self.get_parameter('f_max').value
        self.U_TAU_X = arm_l_y * f_max / 2.0
        self.U_TAU_Y = arm_l_x * f_max / 2.0
        self.U_TAU_Z = moment_const * f_max


        PesoVis = 100
        PesoPan = PesoVis
        #PesoRot = PesoVis / 500
        PesoVel = PesoVis / 2
        PesoAngVel = PesoVel / 5
        PesoAcc = PesoVel / 5
        PesoAngAcc = PesoAngVel / 5
        PesoJerk = PesoAcc / 20
        PesoSnap = PesoJerk / 2
        PesoForce = PesoVis / 1000
        PesoTorque = PesoForce * 2

        Q_pan = np.diag([PesoPan]) / PAN**2
        Q_visual = np.diag([PesoVis,PesoVis,PesoVis]) / np.array([X,Y,Z])**2 
        Q_vel = np.diag([PesoVel, PesoVel, PesoVel]) / V**2
        #Q_rot = np.diag([PesoRot, PesoRot]) / RP_ANG**2  
        
        Q_ang_dot = np.diag([PesoAngVel, PesoAngVel, PesoAngVel]) / ANG_DOT**2
        Q_acc = np.diag([PesoAcc, PesoAcc, PesoAcc]) / ACC**2
        Q_acc_ang = np.diag([PesoAngAcc, PesoAngAcc, PesoAngAcc]) / ACC_ANG**2
        Q_jerk = np.diag([PesoJerk, PesoJerk, PesoJerk]) / JERK**2
        Q_snap = np.diag([PesoSnap, PesoSnap, PesoSnap]) / SNAP**2
        
        R_f = np.diag([PesoForce]) / self.U_F**2
        R_tau = ca.diagcat(PesoTorque / self.U_TAU_X**2, PesoTorque / self.U_TAU_Y**2, PesoTorque / self.U_TAU_Z**2)
        
        R = ca.diagcat(R_f, R_tau)
        Q = ca.diagcat(Q_pan, Q_visual, Q_vel, Q_ang_dot, Q_acc, Q_acc_ang, Q_jerk, Q_snap)
        Q_visual_e = np.diag([2*PesoVis,2*PesoVis,2*PesoVis]) / np.array([X,Y,Z])**2 
        Q_pan_e = np.diag([2*PesoPan]) / PAN**2

        Q_e = ca.diagcat(Q_pan_e, Q_visual_e, 2*Q_vel, 2*Q_ang_dot, 2*Q_acc, 2*Q_acc_ang)

        # Definiamo i limiti fisici reali da passare al solver
        u_min = np.array([0.0, -self.U_TAU_X, -self.U_TAU_Y, -self.U_TAU_Z])
        u_max = np.array([self.U_F, self.U_TAU_X, self.U_TAU_Y, self.U_TAU_Z]) 

        W   = ca.diagcat(Q, R).full()
        W_e = Q_e.full()

        rp_limit_rad = self.get_parameter('rp_limit').value * np.pi / 180.0

        (self.ocp_solver, self.N_horiz, self.nx, self.nu, self.y_idx, self.ny, self.ny_e) = configure_mpc(
            model=self.model, x0=self.x0, camera_offset=self.camera_offset,
            p_obj=np.array([self.current_obj_pos]), rpy_obj=np.array([self.current_obj_rpy]), Tf=self.Tp, ts=self.ts,
            W=W, W_e=W_e, u_min=u_min, u_max=u_max,
            pan_ref=self.pov_target[3], visual_ref = self.pov_target[0:3],
            vel_ref=np.zeros(3),
            cam_rpy=self.camera_rpy, fov_h=self.fov_h, fov_v=self.fov_v,
            rp_limit=rp_limit_rad
        )

        self.u_hover = np.array([self.get_parameter('mass').value * g0, 0.0, 0.0, 0.0])
        
        self.u_prev = [self.u_hover.copy() for _ in range(self.N_horiz)]        
        self.x_prev = [self.x0.copy() for _ in range(self.N_horiz+1)]

        for i in range(self.N_horiz):
            self.ocp_solver.set(i, "u", self.u_prev[i])
            self.ocp_solver.set(i, "x", self.x_prev[i])
        self.ocp_solver.set(self.N_horiz, "x", self.x_prev[self.N_horiz])

        self.publish_predicted_path_from_buffers()

    def solve_MPC(self, xk, visual_ref, pan_target, vel_ref):
        yref0 = None
        set_initial_state(self.ocp_solver, xk)
        
        # Il pan_ref viene ora passato solo come parametro del modello, yref[pan] è fisso a 0
        yref_val = build_yref_online(self.y_idx, visual_ref, vel_ref, u_ref=self.u_hover)
        yref_e = yref_val[:self.ny_e]
        yref0 = yref_val

        # Parametri (7): [p_obj(3), pan_ref(1), visual_ref(3)]
        params = np.zeros(7)
        params[3]     = pan_target        
        params[4:7]   = visual_ref

        for i in range(self.N_horiz + 1):
            p_i = self.current_obj_pos + self.current_obj_vel * (i * self.ts)
            params[0:3] = p_i               # Target object position
            self.ocp_solver.set(i, "p", params)
            
            if i < self.N_horiz:
                self.ocp_solver.set(i, "yref", yref_val)
            elif i == self.N_horiz:
                self.ocp_solver.set(self.N_horiz, "yref", yref_e)

        status = self.ocp_solver.solve()
        if status != 0:
            # Se il solver fallisce, usiamo l'ultimo comando valido o l'hover
            u0 = self.u_prev[0].copy() if self.u_prev is not None else self.u_hover.copy()
            x_seq = [self.x_prev[i].copy() for i in range(self.N_horiz + 1)] if self.x_prev is not None else None
            
            # Forniamo piani di fallback prelevati dall'ultimo piano valido
            u_plan_fallback = np.array(self.u_prev) if self.u_prev is not None else np.tile(u0, (self.N_horiz, 1))
            x_plan_fallback = np.array(self.x_prev) if self.x_prev is not None else np.tile(self.x0, (self.N_horiz + 1, 1))
            
            # Shiftiamo i buffer per l'iterazione successiva
            if self.u_prev is not None and self.x_prev is not None:
                self.u_prev = list(self.u_prev[1:]) + [self.u_prev[-1]]
                self.x_prev = list(self.x_prev[1:]) + [self.x_prev[-1]]
            
            self.get_logger().warn(f"MPC Solver failed (Status {status}). Using fallback.", throttle_duration_sec=1.0)
            return u0, x_seq, yref0, u_plan_fallback, x_plan_fallback

        # --- SUCCESS: Get results and shift buffers ---
        u0 = self.ocp_solver.get(0, "u")
        x_seq = [self.ocp_solver.get(i, "x") for i in range(self.N_horiz + 1)]

        # Estraiamo l'intera sequenza predetta per l'uso in "Hold and Shift"
        new_u_plan = []
        new_x_plan = []
        for i in range(self.N_horiz):
            new_u_plan.append(self.ocp_solver.get(i, "u"))
            new_x_plan.append(self.ocp_solver.get(i, "x"))
        new_x_plan.append(self.ocp_solver.get(self.N_horiz, "x")) # ultimo stato

        # Aggiornamento buffer per warm-start (manteniamo la logica originale per il solver)
        for i in range(self.N_horiz - 1):
            self.u_prev[i] = new_u_plan[i+1]
            self.x_prev[i] = new_x_plan[i+1]
        self.u_prev[self.N_horiz-1] = new_u_plan[-1]
        self.x_prev[self.N_horiz] = new_x_plan[-1]

        return u0, x_seq, yref0, np.array(new_u_plan), np.array(new_x_plan)

    # ==================== Ciclo planner ====================
    def control_step(self):
        if not self.obj_state_received :
            self.get_logger().info("In attesa della ricezione dell'odometria del peg...", throttle_duration_sec=2.0)
            return
        
        if not self.first_odom_received:
            self.get_logger().info("In attesa della prima odometria da PX4...", throttle_duration_sec=2.0)
            return

        with self.state_lock:
            ready = self.acados_solver_ready

        if not ready:
            self.get_logger().info("Compilazione MPC in corso...", throttle_duration_sec=5.0)
            self.planner_configure()
            return

        # Il decollo è gestito da offboard_trajectory_planner.
        # L'MPC aspetta silente finché il supervisor non pubblica /mpc_task/start
        if not getattr(self, 'task_started', False):
            return

        # --- LOGICA HOLD AND SHIFT ---
        with self.state_lock:
            if not getattr(self, 'safety_switch_passed', False):
                # Eseguiamo un solve fittizio per calcolare u0 senza applicarlo
                pass # prosegue sotto, ma lo intercettiamo prima del publish


            if self.solver_is_running:
                if hasattr(self, 'u_plan') and len(self.u_plan) > 1:
                    self.u_plan = np.roll(self.u_plan, -1, axis=0)
                    self.x_plan = np.roll(self.x_plan, -1, axis=0)
                    
                    u0 = self.u_plan[0]
                    next_x = self.x_plan[1]
                    
                    self.publish_optimal_wrench(u0)
                    self.publish_pose_and_twist(next_x)
                return
                return

            self.solver_is_running = True
            
            self.R = Rotation.from_euler('xyz',self.current_rpy).as_matrix()
            self.current_vel[:] = self.current_raw_vel[:]

            # Costruzione dello stato aumentato [p, v, q, w] (13 componenti)
            xk = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_vel[0], self.current_vel[1], self.current_vel[2],
                self.current_quat[0], self.current_quat[1], self.current_quat[2], self.current_quat[3],
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2]
            ])
            # Calcolo dei riferimenti (disaccoppiato dal solver)
            online_visual_ref, pan_target, vel_ref = self.get_current_ref(xk)

            # Pubblicazione riferimenti per debugging/monitoring
            online_spherical_ref = np.array([online_visual_ref[0], pan_target, 0.0])
            self.ref_pub.publish(Float64MultiArray(data=[float(x) for x in online_spherical_ref]))
            self.visual_ref_pub.publish(Float64MultiArray(data=[float(x) for x in online_visual_ref]))

            p_drone = xk[0:3]
            q_drone = xk[6:10]
            Rb = Rotation.from_quat([q_drone[1], q_drone[2], q_drone[3], q_drone[0]]).as_matrix()
            p_cam = p_drone + Rb @ self.camera_offset
            p_obj_now = self.current_obj_pos
            p_rel_world = p_obj_now - p_cam
            R_cam_body = Rotation.from_euler('xyz', self.camera_rpy).as_matrix()
            P_c = R_cam_body.T @ Rb.T @ p_rel_world
            actual_pan = np.arctan2(p_cam[1] - p_obj_now[1], p_cam[0] - p_obj_now[0])
            
            actual_pov_msg = Float64MultiArray()
            actual_pov_msg.data = [float(P_c[0]), float(P_c[1]), float(P_c[2]), float(actual_pan)]
            self.actual_pov_pub.publish(actual_pov_msg)
        
        try:
            t_start = time.perf_counter() # questo è un wall time (non dipende dal simulatore)
            u0_new, x_seq_new, yref0, u_plan_new, x_plan_new = self.solve_MPC(xk, online_visual_ref, pan_target, vel_ref)
            t_end = time.perf_counter()
            
            # Calcolo tempo di risoluzione dell'iterazione corrente dell'MPC 
            dt_solve = t_end - t_start
            
            with self.state_lock:
                self.u_plan = u_plan_new
                self.x_plan = x_plan_new
                
                n_skip = int(dt_solve / self.ts)
                if n_skip > 0:
                    self.get_logger().warn(
                        f"Solver time ({dt_solve:.4f}s) exceeded sampling time ({self.ts}s)! Skipping {n_skip} steps.",
                        throttle_duration_sec=1.0
                    )
                    # Se ci sono salti, shiftiamo gli array di u e x di n_skip posizioni per mantenere la predizione corretta
                    if n_skip < self.N_horiz:
                        self.u_plan = np.roll(self.u_plan, -n_skip, axis=0)
                        self.x_plan = np.roll(self.x_plan, -n_skip, axis=0)
                
                u0 = self.u_plan[0]
                next_x = self.x_plan[1]
                self.solver_is_running = False

            # Safe Switch Check
            if not getattr(self, 'safety_switch_passed', False):
                u_px4 = np.array([self.current_px4_thrust[2], self.current_px4_torque[0], self.current_px4_torque[1], self.current_px4_torque[2]])
                
                # Se non riceviamo dati da PX4, usiamo hover come fallback
                if self.current_px4_thrust[2] == 0.0:
                    u_px4 = self.u_hover
                    
                err_u = np.linalg.norm(u0 - u_px4)
                if err_u < 5.0: # Soglia tolleranza Newton/Nm
                    self.get_logger().info(f"Safe Switch OK! (err_u = {err_u:.2f}). L'MPC prende il controllo di PX4!")
                    self.safety_switch_passed = True
                else:
                    self.get_logger().warn(f"Safe Switch FALLITO. (err_u = {err_u:.2f} > 5.0). Attendo convergenza MPC...", throttle_duration_sec=1.0)
                    return

            self.publish_optimal_wrench(u0)
            self.publish_pose_and_twist(next_x)

            self.path_pub_counter += 1
            if self.path_pub_counter >= 5:
                self.publish_predicted_path(x_plan_new)
                self.path_pub_counter = 0

            # Pubblicazione riferimenti per plotting
            if yref0 is not None:
                yref_u = yref0[self.y_idx["u"]]
                w_ref_msg = Wrench()
                w_ref_msg.force.z = float(yref_u[0])
                w_ref_msg.torque.x = float(yref_u[1])
                w_ref_msg.torque.y = float(yref_u[2])
                w_ref_msg.torque.z = float(yref_u[3])
                self.wrench_ref_pub.publish(w_ref_msg)

                v_ref_msg = TwistStamped()
                v_ref_msg.header.stamp = self.get_clock().now().to_msg()
                v_ref_msg.header.frame_id = "world"
                v_ref_msg.twist.linear.x = float(yref0[self.y_idx["vel"]][0])
                v_ref_msg.twist.linear.y = float(yref0[self.y_idx["vel"]][1])
                v_ref_msg.twist.linear.z = float(yref0[self.y_idx["vel"]][2])
                self.vel_ref_pub.publish(v_ref_msg)

        except Exception as e:
            self.get_logger().error(f"Errore nel solver: {e}")
            with self.state_lock:
                self.solver_is_running = False
            return



    # ==================== Funzioni PX4 Controllo Integrato ====================

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.timestamp = 0  # PX4 auto-compila con hrt_absolute_time()
        self.offboard_control_mode_publisher.publish(msg)

    # (manage_offboard_state rimosso perché delegato al supervisor)

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
        # Aggiorna il controllo applicato per l'osservatore di Luenberger
        self.last_u0_applied = np.array(u0, dtype=float)

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
 
            
            # Parametri per la linearizzazione (recuperati dai parametri del nodo)
            w_max = self.get_parameter('w_max').value
            w_min = self.get_parameter('w_min').value
            # Calcoliamo la velocità angolare desiderata [0, w_max] rad/s
            w_target = w_max * np.sqrt(max(0.0, float(u0[0])) / self.U_F)
            # Mappatura su norm_thrust [0, 1] considerando il range dell'airframe [w_min, w_max]
            norm_thrust = (w_target - w_min) / (w_max - w_min)   
            norm_thrust = max(0.0, min(1.0, norm_thrust))
            #norm_thrust = max(0.0, float(u0[0])/self.U_F)

            thrust_msg = VehicleThrustSetpoint()
            thrust_msg.timestamp = 0           # PX4 auto-compila con hrt_absolute_time()
            thrust_msg.timestamp_sample = 0    # PX4 auto-compila con hrt_absolute_time()
            thrust_msg.xyz[0] = 0.0
            thrust_msg.xyz[1] = 0.0
            thrust_msg.xyz[2] = -float(norm_thrust) 
            self.thrust_pub.publish(thrust_msg)

            # Usiamo self.U_TAU_X, self.U_TAU_Y, self.U_TAU_Z definiti in configure_mpc
            torque_msg = VehicleTorqueSetpoint()
            torque_msg.timestamp = 0           # PX4 auto-compila con hrt_absolute_time()
            torque_msg.timestamp_sample = 0    # PX4 auto-compila con hrt_absolute_time()
            
            # Segni FLU -> FRD (PX4): 
            # Roll (X): CCW FLU = CCW FRD (stesso verso) -> u0[1]
            # Pitch (Y): CCW FLU = CW FRD (opposto) -> -u0[2]
            # Yaw (Z): CCW FLU = CW FRD (opposto) -> -u0[3]
            torque_msg.xyz[0] = float(np.clip((u0[1]/self.U_TAU_X),-1.0, 1.0)) 
            torque_msg.xyz[1] = -float(np.clip((u0[2]/self.U_TAU_Y),-1.0, 1.0))
            torque_msg.xyz[2] = -float(np.clip((u0[3]/self.U_TAU_Z),-1.0, 1.0))
            self.torque_pub.publish(torque_msg)
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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()