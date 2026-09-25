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
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Wrench, Vector3, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64MultiArray, String, Float64

# --- PX4 MESSAGES IMPORTS ---
from px4_msgs.msg import VehicleOdometry, VehicleThrustSetpoint, VehicleTorqueSetpoint, OffboardControlMode, VehicleCommand, VehicleControlMode, TrajectorySetpoint
import numpy as np
import casadi as ca
from casadi import pi as pi
from scipy.spatial.transform import Rotation
import time
import threading
#from rclpy.callback_groups import ReentrantCallbackGroup
#from rclpy.executors import MultiThreadedExecutor


from guardrone_pkg.drone_MPC_settings import (
    setup_model, setup_initial_conditions, configure_mpc, set_initial_state, build_yref_online
)
from utils_pkg.common import quat_to_RPY, g0, wrap_pi
from utils_pkg.utils_np import min_angle, cartesian_to_cylindrical, cylindrical_to_cartesian
from utils_pkg.momentum_based_estimator import MomentumBasedEstimator

import tf2_ros


class MpcPlannerNode(Node):
    def __init__(self):
        super().__init__('mpc_planner_node')

        # === Modello e condizioni iniziali ===
        self.declare_parameter('controller', 1)  # 1: Controller (thrust/torque), 0: Planner (pos/vel setpoints)
        self.is_controller = bool(self.get_parameter('controller').value)

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

        self.declare_parameter('use_mbe', True)


        self.mass = self.get_parameter('mass').value
        self.ixx = self.get_parameter('ixx').value 
        self.iyy = self.get_parameter('iyy').value 
        self.izz = self.get_parameter('izz').value 
        arm_l_x = self.get_parameter('arm_l_x').value
        arm_l_y = self.get_parameter('arm_l_y').value
        moment_const = self.get_parameter('moment_const').value        
        self.U_F = self.get_parameter('f_max').value
        self.U_TAU_X = arm_l_y * self.U_F / 2.0
        self.U_TAU_Y = arm_l_x * self.U_F / 2.0
        #self.U_TAU_X = self.U_TAU_X/1.5        ## MODIFICA PER HARDWARE
        #self.U_TAU_Y = self.U_TAU_Y/1.5        ## MODIFICA PER HARDWARE
        #self.U_TAU_X = self.U_TAU_X
        #self.U_TAU_Y = self.U_TAU_Y
        self.U_TAU_Z = moment_const * self.U_F
        #self.U_TAU_Z = self.U_TAU_Z / 1.25         ## MODIFICA PER HARDWARE
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.start_roll = self.get_parameter('start_roll').value
        self.start_pitch = self.get_parameter('start_pitch').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.peg_drone_spawn = np.array([
            self.get_parameter('peg_x').value,
            self.get_parameter('peg_y').value,
            self.get_parameter('peg_z').value
        ])
        self.fov_h = self.get_parameter('fov_h').value
        self.fov_v = self.get_parameter('fov_v').value
        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        cam_roll = self.get_parameter('cam_roll').value
        cam_pitch = self.get_parameter('cam_pitch').value
        cam_yaw = self.get_parameter('cam_yaw').value

        self.get_logger().info(f"Parametri caricati: m={self.mass}, I=[{self.ixx}, {self.iyy}, {self.izz}]")

        self.camera_offset = np.array([cam_x, cam_y, cam_z])
        self.camera_rpy = np.array([cam_roll, cam_pitch, cam_yaw])
        
        """
        Setup problema di ottimizzazione: modello, condizioni iniziali, ts, N_horiz
        """
        self.model, self.model_rpy = setup_model(self.mass, self.ixx, self.iyy, self.izz)
        self.x0, self.x0_rpy = setup_initial_conditions(self.start_x,self.start_y,self.start_z,self.start_roll,self.start_pitch,self.start_yaw)


        # === Tempo/Orizzonte ===
        self.Hz = 100.0
        self.ts = 1.0/self.Hz             # 10 ms
        self.N_horiz = 15        # Orizzonte di predizione (numero di campioni) # SIMULAZIONE CONTROLLER
        self.Tp = self.N_horiz * self.ts  # Tempo totale dell'orizzonte 

        self.path_pub_counter = 0  # Contatore per limitare la frequenza di pubblicazione del path

        self.t_prev = 0.0

        # === Stato MPC / loop ===
        self.acados_solver_ready = False    # true quando la configurazione dell'MPC è pronta 
        self.obj_state_received = False     # true quando ricevuto lo stato del peg
        self.start_received = False         # true quando ricevuto il comando di start dal supervisor
        self.first_odom_received = False    # true quando ricevuto la prima odometria del drone
        self.planner_ready_published = False    # true quando pubblicato il segnale di pronto al supervisor
        self.startup_counter = 0                # contatore per limitare la frequenza di pubblicazione del path
        
        """
            Inizializzazione MBE
        """
        # In modalità planner (controller=0), l'MBE è disabilitato in quanto superfluo
        self.use_mbe = self.get_parameter('use_mbe').value and self.is_controller
        if self.use_mbe:
            self.mbe = MomentumBasedEstimator(self.mass, self.ixx, self.iyy, self.izz, self.ts, g0)
            self.get_logger().info("MBE abilitato.")
        else:
            self.mbe = None
            self.get_logger().info("MBE DISABILITATO — F_ext e Tau_ext saranno zero.")


        # Matrici fisse di conversione frame
        self.M_ned2enu = np.array([[0.0, 1.0, 0.0], 
                              [1.0, 0.0, 0.0], 
                              [0.0, 0.0, -1.0]])
                              
        self.M_frd2flu = np.array([[1.0, 0.0, 0.0], 
                              [0.0, -1.0, 0.0], 
                              [0.0, 0.0, -1.0]])
        self.M_enu2ned = self.M_ned2enu.T
        self.M_flu2frd = self.M_frd2flu.T
        

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
        self.e_int = np.zeros(3)  # Accumulatore errore integrale [e_int_r, e_int_beta, e_int_z]


        # Target PoV in coordinate cilindriche: [r_cyl_ref, beta_ref, z_rel_ref]
        # r_cyl = distanza 2D dall'oggetto nel piano XY [m]
        # beta  = azimut del vettore drone->obj nel piano XY [rad] (pan attorno all'oggetto)
        # z_rel     = quota relativa rispetto all'oggetto [m]
        self.pov_target = np.array([3.0, np.pi, 0.0])  # default: 3m dietro, stessa quota
        self.pov_yaw_offset = 0.0  # 4° elemento: decentramento yaw nel FoV [rad]

        # --- PUBLISHERS COMANDI E PX4 INTEGRATION ---
        self.is_armed = False      
        self.is_offboard = False   
        self.task_started = False
        self._first_mpc_solve = True   # flag per warm-start allo switch
        self._last_yref_val = None     # cache yref per evitare chiamate ctypes ridondanti
        self.last_yaw_desired = 0.0    # cache ultimo yaw desiderato per modalità planner
        self.last_u0_applied = None
        self.safety_switch_passed = False
        self.current_F_ext = np.zeros(3)
        self.current_Tau_ext = np.zeros(3)
        self.last_quat = None
        self.u_plan = None
        self.x_plan = None
        self.current_px4_thrust = np.array([0.0, 0.0, self.mass * 9.81])
        self.current_px4_torque = np.zeros(3)

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
        self.wrench_ref_pub = self.create_publisher(Wrench, '/wrench_reference', 1) # <--- Nuova
        self.estimated_wrench_pub = self.create_publisher(Wrench, '/estimated_wrench', 1)

        self.single_wrench_pub = self.create_publisher(Wrench, '/optimal_wrench', 1)

        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 1)
        if self.is_controller:
            self.get_logger().info("MPC in modalità CONTROLLER: attiva pub offboard, thrust e torque.")
            self.thrust_pub = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', 1)
            self.torque_pub = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', 1)
        else:
            self.get_logger().info("MPC in modalità PLANNER: attiva pub offboard e trajectory_setpoint (pos/vel).")
            self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 1)



        self.current_position = np.zeros(3) # ENU (World)
        self.current_rpy = np.zeros(3)  # FLU (Body)
        self.current_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.current_vel = np.zeros(3)  # ENU
        self.current_vel = np.zeros(3)  # ENU
        self.current_ang_vel = np.zeros(3)  # FLU

        qos_latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.ready_publisher  = self.create_publisher(Bool, '/drone_planner_ready',  qos_latched)
        self.optimal_path_pub = self.create_publisher(Path, '/optimal_drone_path', qos_latched)

        self.peg_odom_subscription = self.create_subscription(
            VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', self.peg_odom_callback, px4_qos_profile)
        

        self.odom_subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, px4_qos_profile
        )

        self.single_pose_pub  = self.create_publisher(PoseStamped,  '/optimal_drone_pose',  1)
        self.drone_pose_pub   = self.create_publisher(PoseStamped,  '/drone_pose',          1) # <--- Per RViz
        self.drone_rpy_pub    = self.create_publisher(Vector3,      '/drone_rpy',           1)
        self.peg_pose_pub     = self.create_publisher(PoseStamped,  '/peg_pose',            1) # <--- Per RViz (Peg)
        self.single_twist_pub = self.create_publisher(TwistStamped, '/optimal_drone_twist', 1)
        self.vel_ref_pub      = self.create_publisher(TwistStamped, '/velocity_reference',  1)
        self.drone_vel_pub    = self.create_publisher(TwistStamped, '/drone_velocity',      1)  # ENU vel + FLU omega — per logger
        self.drone_cam_pose_pub = self.create_publisher(PoseStamped, '/drone_cam_pose',  1)  # posizione camera nel mondo — per logger e RViz
        self.tf_broadcaster   = tf2_ros.TransformBroadcaster(self)
        self.ref_pub = self.create_publisher(Float64MultiArray, '/online_cylindrical_ref', 1)
        self.actual_pov_pub = self.create_publisher(Float64MultiArray, '/actual_pov', 1)
        self.integral_action_pub = self.create_publisher(Vector3, '/integral_action', 1)
        self.solve_time_pub = self.create_publisher(Float64, '/mpc_solve_time', 1)

        self.control_timer = self.create_timer(self.ts, self.control_step)
        self.start_subscription = self.create_subscription(PoseStamped, '/peg_pose', self.start_callback, 10)
        self.pov_target_sub = self.create_subscription(Float64MultiArray, '/pov_target', self.pov_target_callback, 10)
        self.haptic_ref_sub = self.create_subscription(Float64MultiArray, '/haptic_ref', self.haptic_ref_callback, 10)

        # Haptic state
        self.haptic_pov = None
        self.haptic_pov_dot = None
        self.haptic_timestamp = None

        # Joy state
        self.joy_ref_sub = self.create_subscription(Float64MultiArray, '/joy_ref', self.joy_ref_callback, 10)
        self.joy_pov = None
        self.joy_pov_dot = None
        self.joy_timestamp = None


        # === Integrazione con Supervisor ===
        self.supervisor_task_running = False
        self.supervisor_start_sub = self.create_subscription(
            Bool, '/mpc_task/start', self.supervisor_start_callback, 10)
        self.supervisor_status_pub = self.create_publisher(String, '/mpc_task/status', 10)

        # Sottoscrizione comandi PX4 solo per modalità controller (necessari a safe switch e MBE pre-switch)
        if self.is_controller:
            self.thrust_out_sub = self.create_subscription(
                VehicleThrustSetpoint,
                '/fmu/out/vehicle_thrust_setpoint',
                self.thrust_out_cb,
                px4_qos_profile,
            )
            self.torque_out_sub = self.create_subscription(
                VehicleTorqueSetpoint,
                '/fmu/out/vehicle_torque_setpoint',
                self.torque_out_cb,
                px4_qos_profile,
            )

        self.current_px4_thrust = np.zeros(3)   # FLU
        self.current_px4_torque = np.zeros(3)   # FLU
        self.safety_switch_passed = False

        self.get_logger().info("MPC Node avviato. In attesa...")

    # ==================== Callbacks I/O ====================

    def supervisor_start_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Ricevuto comando dal Supervisor: Inizio MPC Task!")
            self.task_started = True
            self.planner_configure()
        else:
            self.get_logger().info("Ricevuto comando dal Supervisor: Stop MPC Task!")
            self.task_started = False

    def control_mode_callback(self, msg: VehicleControlMode):
        self.is_armed = msg.flag_armed
        self.is_offboard = msg.flag_control_offboard_enabled

    def peg_odom_callback(self, msg: VehicleOdometry):
        # Quaternione PX4: rappresenta R_frd2ned (body FRD → world NED)
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]] 
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

        R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        
        # Posizione: NED → ENU con offset
        pos_enu = self.M_ned2enu @ np.array([msg.position[0], msg.position[1], msg.position[2]])
        peg_offset = np.array([0.455, 0.0, 0.22])
        self.current_obj_pos = pos_enu + self.peg_drone_spawn + peg_offset
        
        # Velocità: NED → ENU
        self.current_obj_vel[:] = self.M_ned2enu @ np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        
        # Velocità angolare: FRD → FLU
        self.current_obj_ang_vel[:] = self.M_frd2flu @ np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])
        
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
        if self.acados_solver_ready == True:
            return
        
        # prima configurazione dell' MPC
        self.configure_mpc()
            
        if not self.planner_ready_published:
            self.ready_publisher.publish(Bool(data=True))
            self.planner_ready_published = True

        self.acados_solver_ready = True
        self.publish_predicted_path_from_buffers()

    def start_callback(self, _msg: PoseStamped):
        if self.start_received:
            return
        self.start_received = True
        self.get_logger().info("Ricevuta posa iniziale del peg.")
        # Il timer è già stato avviato in peg_path_callback
        self.destroy_subscription(self.start_subscription)


    # Prox due funzioni sono usate per check sullo switch all'MPC
    def thrust_out_cb(self, msg):
        # I valori in uscita da PX4 sono normalizzati [-1, 1]. In NED, Z=-1 significa full thrust verso l'alto.
        self.current_px4_thrust =  self.M_frd2flu @ msg.xyz     # da FRD in FLU
        
        # DENORMALIZZAZIONE IN FLU
        self.current_px4_thrust[0] *= self.U_F
        self.current_px4_thrust[1] *= self.U_F
        self.current_px4_thrust[2] *= self.U_F


    def torque_out_cb(self, msg):
        # FRD --> FLU
        self.current_px4_torque =  self.M_frd2flu @ msg.xyz 
        # DENORMALIZZAZIONE IN FLU
        self.current_px4_torque[0] *=  self.U_TAU_X
        self.current_px4_torque[1] *=  self.U_TAU_Y
        self.current_px4_torque[2] *=  self.U_TAU_Z


    def odom_callback(self, msg: VehicleOdometry):
        # Quaternione PX4: rappresenta R_frd2ned (body FRD → world NED)
        # SciPy usa l'ordine [x, y, z, w]
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]] 
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

        # R_flu2enu (body→world per MPC): catena FLU→FRD→NED→ENU
        # M_frd2flu ortogonale, quindi M_flu2frd = M_frd2flu.T
        R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
        
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        q_flu2enu = rot_flu2enu.as_quat() # [x, y, z, w] (formato scipy)

        # Posizione: NED → ENU (M_ned2enu @ [N,E,D] = [E,N,-D])
        self.current_position[:] = self.M_ned2enu @ np.array([msg.position[0], msg.position[1], msg.position[2]])
        # Aggiungiamo lo spawn offset per posizionare il drone globalmente (fix RViz e Planner)
        self.current_position[0] += self.start_x
        self.current_position[1] += self.start_y
        self.current_position[2] += self.start_z

        # Assegnazione all'MPC (che si aspetta [w, x, y, z])
        q_w = q_flu2enu[3]
        q_x = q_flu2enu[0]
        q_y = q_flu2enu[1]
        q_z = q_flu2enu[2]
            
        q = [q_w, q_x, q_y, q_z]
        q = q / np.linalg.norm(q)    # norma quaternione ad 1

        # Evitiamo salti discontinui del quaternione (q vs -q)
        if self.last_quat is not None:
            if np.dot(q, self.last_quat) < 0:
                q = -q

        self.current_quat = q
        self.last_quat = q.copy()

        self.current_rpy[:] = rot_flu2enu.as_euler('xyz')

        # Velocità lineare: NED → ENU
        self.current_vel[:] = self.M_ned2enu @ np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]]).T
            
        self.current_ang_vel[:] = self.M_frd2flu @ np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]]).T
            
        self.px4_odom_timestamp_us = msg.timestamp  # Timestamp PX4 in microsecondi
            
        # --- INIZIALIZZAZIONE DINAMICA X0 ---
        if not self.first_odom_received:
            self.e_int = np.zeros(3)
            self.x0 = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_vel[0],  self.current_vel[1],  self.current_vel[2],
                q_w, q_x, q_y, q_z,
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2],
                0.0, 0.0, 0.0
                ])
            self.x0_rpy = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_vel[0],  self.current_vel[1],  self.current_vel[2],
                self.current_rpy[0],      self.current_rpy[1],      self.current_rpy[2],
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2],
                0.0, 0.0, 0.0
                ])

            # --- INITIALIZATION OF THE ESTIMATOR ---
            if self.use_mbe:
                self.mbe.initialize(self.current_vel, self.current_ang_vel)
                self.get_logger().info(f"Estimator initialized with initial velocity: {self.current_vel}, initial angular velocity: {self.current_ang_vel}")
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
        
        drone_rpy_msg = Vector3()
        drone_rpy_msg.x = self.current_rpy[0]
        drone_rpy_msg.y = self.current_rpy[1]
        drone_rpy_msg.z = self.current_rpy[2]
        self.drone_rpy_pub.publish(drone_rpy_msg)

        # Pubblica velocità ENU + omega FLU per il logger (già convertite, nessun calcolo extra)
        drone_vel_msg = TwistStamped()
        drone_vel_msg.header.stamp = t.header.stamp
        drone_vel_msg.header.frame_id = 'world'
        drone_vel_msg.twist.linear.x  = float(self.current_vel[0])
        drone_vel_msg.twist.linear.y  = float(self.current_vel[1])
        drone_vel_msg.twist.linear.z  = float(self.current_vel[2])
        drone_vel_msg.twist.angular.x = float(self.current_ang_vel[0])
        drone_vel_msg.twist.angular.y = float(self.current_ang_vel[1])
        drone_vel_msg.twist.angular.z = float(self.current_ang_vel[2])
        self.drone_vel_pub.publish(drone_vel_msg)

        # Pubblica posizione camera nel mondo (ENU) — sempre, non solo durante il task,
        # così p_cam è disponibile fin dall'inizio del logging (pre-task)
        Rb_odom = Rotation.from_quat([q_x, q_y, q_z, q_w]).as_matrix()
        p_cam_odom = self.current_position + Rb_odom @ self.camera_offset
        cam_pose_msg = PoseStamped()
        cam_pose_msg.header.stamp = t.header.stamp
        cam_pose_msg.header.frame_id = 'world'
        cam_pose_msg.pose.position.x = float(p_cam_odom[0])
        cam_pose_msg.pose.position.y = float(p_cam_odom[1])
        cam_pose_msg.pose.position.z = float(p_cam_odom[2])
        cam_pose_msg.pose.orientation = t.transform.rotation
        self.drone_cam_pose_pub.publish(cam_pose_msg)

        # Calcola e pubblica il PoV attuale (coordinate cilindriche telecamera rispetto al peg)
        # sempre, non appena lo stato del peg è disponibile, così che il logger abbia valori validi
        # prima dello switch al task MPC
        if self.obj_state_received:
            p_rel = p_cam_odom - self.current_obj_pos
            cyl_act = cartesian_to_cylindrical(p_rel)
            r_cyl_act, beta_act, z_act = float(cyl_act[0]), float(cyl_act[1]), float(cyl_act[2])

            yaw_actual = float(self.current_rpy[2])
            p_cam_to_obj = self.current_obj_pos - p_cam_odom
            yaw_desired = float(wrap_pi(np.arctan2(p_cam_to_obj[1], p_cam_to_obj[0])))
            yaw_rel_act = float(min_angle(yaw_actual - yaw_desired))

            actual_pov_msg = Float64MultiArray()
            actual_pov_msg.data = [r_cyl_act, beta_act, z_act, yaw_rel_act]
            self.actual_pov_pub.publish(actual_pov_msg)

            if not self.task_started:
                self.last_yaw_desired = yaw_desired
                # Pubblica riferimento cilindrico attivo (/online_cylindrical_ref) anche pre-task
                cyl_ref = self.get_current_ref(None)  # 4 elementi: [r, beta, z, yaw_offset]
                self.ref_pub.publish(Float64MultiArray(data=[float(x) for x in cyl_ref]))

    """
    ===================== REFERENCE CALLBACKS ==============================
    """
    def pov_target_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            # Formato: [r_cyl_ref, beta_ref, z_rel_ref, yaw_offset]
            self.pov_target = np.array([msg.data[0], msg.data[1], msg.data[2]], dtype=float)
            self.pov_yaw_offset = float(msg.data[3])
            self.ref_pub.publish(Float64MultiArray(data=[
                float(self.pov_target[0]), float(self.pov_target[1]),
                float(self.pov_target[2]), self.pov_yaw_offset]))

    def haptic_ref_callback(self, msg: Float64MultiArray):
        # Formato atteso: [r, beta, z_rel]
        if len(msg.data) >= 3:
            self.haptic_pov = np.array(msg.data[0:3], dtype=float)  # [r, beta, z_rel]
            self.haptic_timestamp = self.get_clock().now()
            self.pov_target = self.haptic_pov.copy()

    def joy_ref_callback(self, msg: Float64MultiArray):
        # Formato atteso: [r, beta, z_rel]
        if len(msg.data) >= 3:
            self.joy_pov = np.array(msg.data[0:3], dtype=float)  # [r, beta, z_rel]
            self.joy_timestamp = self.get_clock().now()
            self.pov_target = self.joy_pov.copy()

    # ==================== Configurazione e Solve ====================
    def get_current_ref(self, xk):
        """
        Calcola il riferimento cilindrico [r_cyl_ref, beta_ref, z_rel_ref] con interpolazione
        tra controllo Manuale (Haptic/Joypad) e Traiettoria Autonoma.

        Coordinate cilindriche centrate sull'oggetto, vettore drone->oggetto:
          r_cyl = distanza 2D nel piano XY [m]
          beta  = azimut nel piano XY [rad]  (orbita orizzontale)
          z_rel     = quota relativa [m]

          Al rilascio del comando manuale, il drone resta nella posizione lasciata
          (pov_target viene aggiornato ad ogni messaggio haptic/joy).
        """
        now = self.get_clock().now()
        dt_off = 0.2  # soglia inattività [s]

        # --- Valutazione haptic ---
        h_active = False
        if self.haptic_timestamp is not None:
            dt_h = (now - self.haptic_timestamp).nanoseconds / 1e9
            h_active = (dt_h < dt_off)

        # --- Valutazione joypad ---
        j_active = False
        if self.joy_timestamp is not None:
            dt_j = (now - self.joy_timestamp).nanoseconds / 1e9
            j_active = (dt_j < dt_off)

        # --- Selezione priorità ---
        if h_active:
            ref = self.haptic_pov
        elif j_active:
            ref = self.joy_pov
        else:
            ref = self.pov_target

        r_cyl_ref = ref[0]
        beta_ref  = wrap_pi(ref[1])
        z_rel_ref = ref[2]

        # Yaw offset: oggetto decentrato nel FoV (0 = centro immagine)
        yaw_offset = self.pov_yaw_offset

        return np.array([r_cyl_ref, beta_ref, z_rel_ref, yaw_offset])

    def configure_mpc(self):

        # =========================================================================
        # 1. SCALE FISICHE (Denominatori Regola di Bryson)
        # =========================================================================
        X_CART     = 0.2                          # [m] tolleranza errore posizione piano XY
        Y_CART     = 0.2                          # [m]
        Z_CART     = 0.2                          # [m] tolleranza errore quota
        Y_CYL      = np.pi / 3.0                  # [rad] tolleranza puntamento yaw (~60 deg)
        E_INT_CART = np.array([0.1, 0.1, 0.1])    # [m*s] tolleranza errore integrale
        V          = np.array([0.2, 0.2, 0.2])    # [m/s] velocità max attesa
        ANG_DOT    = np.array([0.15, 0.15, 0.3])  # [rad/s] velocità angolare max
        ACC        = np.array([0.3, 0.3, 0.3])    # [m/s^2] accelerazione max
        ACC_ANG    = np.array([0.4, 0.4, 0.4])    # [rad/s^2] accelerazione angolare max

        # =========================================================================
        # 2. SELEZIONE PARAMETRICA DEI PESI (Proporzioni relative su PesoVis)
        # =========================================================================
        PesoVis = 10.0
        PesoX = PesoY = PesoZ = PesoYaw = PesoVis

        if not self.is_controller:
            # ---------------------------------------------------------------------
            # CASO 1:  PLANNER (setpoint pos/vel a PX4)
            # ---------------------------------------------------------------------
            self.get_logger().info("[MPC Tuning] Modalità: PLANNER (Integrale: OFF, Controllo PX4: Pos/Vel)")
            PesoInt    = 0.0              # Integrale OFF: tracking di regime affidato ai PID di PX4
            PesoVel    = PesoVis / 5.0   
            PesoAngVel = PesoVis / 50.0
            PesoAcc    = PesoVis / 30.0   
            PesoAngAcc = PesoVis / 100.0
            PesoForce  = PesoVis / 100.0  
            PesoTorque = PesoVis / 100.0
            scale_e = [10.0, 0.0, 10.0, 10.0, 2.0, 1.0] # Vis

        elif self.use_mbe:
            # ---------------------------------------------------------------------
            # CASO 2: CONTROLLER WRENCH CON MBE (disturbi compensati via feedforward)
            # ---------------------------------------------------------------------
            self.get_logger().info("[MPC Tuning] Modalità: CONTROLLER + MBE (Integrale: OFF, Feedforward: ON)")
            PesoInt    = 0.0              # Integrale OFF: disturbo compensato direttamente da MBE
            PesoVel    = PesoVis / 7.0   
            PesoAngVel = PesoVis / 10.0
            PesoAcc    = PesoVis / 20.0
            PesoAngAcc = PesoVis / 50.0
            PesoForce  = PesoVis / 10.0   
            PesoTorque = PesoVis / 10.0
            scale_e = [5.0, 0.0, 5.0, 5.0, 2.0, 1.0]

        else:
            # ---------------------------------------------------------------------
            # CASO 3: CONTROLLER WRENCH SENZA MBE (con Azione Integrale) ------------NON USARE
            # ---------------------------------------------------------------------
            self.get_logger().info("[MPC Tuning] Modalità: CONTROLLER (Integrale: ON, MBE: OFF)")
            PesoInt    = PesoVis / 2   # Integrale ON: cancella errore a regime
            PesoVel    = PesoVis / 5.0
            PesoAngVel = PesoVis / 5.0
            PesoAcc    = PesoVis / 20.0
            PesoAngAcc = PesoVis / 20.0
            PesoForce  = PesoVis / 10000.0
            PesoTorque = PesoVis / 10000.0
            scale_e = [10.0, 50.0, 10.0, 10.0, 2.0, 1.0]

        # =========================================================================
        # 3. COSTRUZIONE MATRICI Q, R, Q_e
        # =========================================================================
        Q_cart = np.diag([
            PesoX   / X_CART**2,
            PesoY   / Y_CART**2,
            PesoZ   / Z_CART**2,
            PesoYaw / Y_CYL**2
        ])
        Q_int     = np.diag([PesoInt]*3) / np.array(E_INT_CART)**2
        Q_vel     = np.diag([PesoVel] * 3) / np.array(V)**2
        Q_ang_dot = np.diag([PesoAngVel] * 3) / np.array(ANG_DOT)**2
        Q_acc     = np.diag([PesoAcc] * 3) / np.array(ACC)**2
        Q_acc_ang = np.diag([PesoAngAcc] * 3) / np.array(ACC_ANG)**2

        R_f   = np.diag([PesoForce / 0.5**2])
        R_tau = np.diag([
            PesoTorque / 0.2**2,
            PesoTorque / 0.2**2,
            PesoTorque / 0.5**2
        ])
        R   = ca.diagcat(R_f, R_tau)
        Q   = ca.diagcat(Q_cart, Q_int, Q_vel, Q_ang_dot, Q_acc, Q_acc_ang)
        Q_e = ca.diagcat(
            scale_e[0] * Q_cart,
            scale_e[1] * Q_int,
            scale_e[2] * Q_vel,
            scale_e[3] * Q_ang_dot,
            scale_e[4] * Q_acc,
            scale_e[5] * Q_acc_ang
        )

        u_min = np.array([0.0, -self.U_TAU_X, -self.U_TAU_Y, -self.U_TAU_Z])
        u_max = np.array([self.U_F,  self.U_TAU_X,  self.U_TAU_Y,  self.U_TAU_Z])

        W   = ca.diagcat(Q, R).full()
        W_e = Q_e.full()

        # Converte il riferimento cilindrico in target cartesiano per il solver
        p_target_init = self.current_obj_pos + cylindrical_to_cartesian(self.pov_target)
        yaw_des_init = float(wrap_pi(self.pov_target[1] + np.pi))

        (self.ocp_solver, self.N_horiz, self.nx, self.nu, self.y_idx, self.ny, self.ny_e) = configure_mpc(
            model=self.model, x0=self.x0,
            p_target=p_target_init, Tf=self.Tp, ts=self.ts,
            W=W, W_e=W_e, u_min=u_min, u_max=u_max,
            yaw_des=yaw_des_init)
        cond = np.linalg.cond(W)
        self.get_logger().info(f"Condition number di W: {cond:.2e}")
        w_diag = np.diag(W)
        self.get_logger().info(f"W min={w_diag.min():.2e} ({w_diag.argmin()}), max={w_diag.max():.2e} ({w_diag.argmax()})")

        self.u_hover = np.array([self.mass * g0, 0.0, 0.0, 0.0])
        
        self.u_prev = [self.u_hover.copy() for _ in range(self.N_horiz)]        
        self.x_prev = [self.x0.copy() for _ in range(self.N_horiz+1)]

        for i in range(self.N_horiz):
            self.ocp_solver.set(i, "u", self.u_prev[i])
            self.ocp_solver.set(i, "x", self.x_prev[i])
        self.ocp_solver.set(self.N_horiz, "x", self.x_prev[self.N_horiz])

        self.publish_predicted_path_from_buffers()

    def solve_MPC(self, xk, p_target, yaw_des, vel_ref, F_ext=None, Tau_ext=None, v_obj=None):
        """p_target: posizione target cartesiana (CoM o camera) nel frame mondo, precalcolata esternamente.
        yaw_des: yaw desiderato precalcolato esternamente (la telecamera punta all'oggetto).
        v_obj: velocità dell'oggetto (ENU) per propagare p_target sull'orizzonte. Se None, usa self.current_obj_vel.
        """
        set_initial_state(self.ocp_solver, xk)

        # Prendo il wrench esterno per creare un delta_u di riferimento per il controller
        u_ref_dynamic = np.array([
            self.mass * g0 - F_ext[2],  
            -Tau_ext[0],                
            -Tau_ext[1],                
            -Tau_ext[2]                 
        ])

        yref_val = build_yref_online(self.y_idx, vel_ref, u_ref=u_ref_dynamic)
        yref_e   = yref_val[:self.ny_e]

        if F_ext is None:
            F_ext = np.zeros(3)
        if Tau_ext is None:
            Tau_ext = np.zeros(3)
        if v_obj is None:
            v_obj = self.current_obj_vel

        # Parametri (10): [p_target(3), yaw_des(1), F_ext(3), Tau_ext(3)]
        params = np.zeros(10)
        params[3]    = yaw_des
        params[4:7]  = F_ext
        params[7:10] = Tau_ext

        # Propaga p_target sull'orizzonte con la velocità dell'oggetto
        for i in range(self.N_horiz + 1):
            params[0:3] = p_target + v_obj * (i * self.ts)
            self.ocp_solver.set(i, "p", params)

        # Riferimenti yref: aggiorna nel solver solo se variati (risparmia N+1 chiamate ctypes a ciclo)
        if self._last_yref_val is None or not np.array_equal(self._last_yref_val, yref_val):
            for i in range(self.N_horiz):
                self.ocp_solver.set(i, "yref", yref_val)
            self.ocp_solver.set(self.N_horiz, "yref", yref_e)
            self._last_yref_val = yref_val.copy()

        t0 = time.perf_counter()
        status = self.ocp_solver.solve()

        ###### DEBUG ######
        #self.get_logger().info(f"Solve time: {(time.perf_counter()-t0)*1e3:.2f} ms")
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
            return u0, x_seq, yref_val, u_plan_fallback, x_plan_fallback

        # --- SUCCESS: Get results and shift buffers ---
        # Lettura unica dei buffer dal solver (evita doppie chiamate ctypes per u0 e x_seq)
        new_u_plan = [self.ocp_solver.get(i, "u") for i in range(self.N_horiz)]
        new_x_plan = [self.ocp_solver.get(i, "x") for i in range(self.N_horiz + 1)]
        u0 = new_u_plan[0]
        x_seq = new_x_plan

        for i in range(self.N_horiz - 1):
            self.u_prev[i] = new_u_plan[i+1]
            self.x_prev[i] = new_x_plan[i+1]
        self.u_prev[self.N_horiz-1] = new_u_plan[-1]
        self.x_prev[self.N_horiz]   = new_x_plan[-1]

        return u0, x_seq, yref_val, np.array(new_u_plan), np.array(new_x_plan)

    # ==================== Ciclo planner ====================
    def control_step(self):
        if not self.obj_state_received :
            self.get_logger().info("In attesa della ricezione dell'odometria del peg...", throttle_duration_sec=2.0)
            return
        
        if not self.first_odom_received:
            self.get_logger().info("In attesa della prima odometria da PX4...", throttle_duration_sec=2.0)
            return

        # === AGGIORNAMENTO MBE — attivo SEMPRE, anche prima della configurazione solver ===
        # Massimizza il tempo di pre-convergenza durante il decollo PX4:
        # al momento dello switch all'MPC, F_ext e Tau_ext sono già stabilizzati.
        F_ext = np.zeros(3)
        Tau_ext = np.zeros(3)
        if self.use_mbe and self.mbe is not None:
            if self.task_started and self.last_u0_applied is not None:
                # MPC attivo: usa l'ultimo ingresso pianificato
                f_cmd = self.last_u0_applied[0]
                tau_cmd = self.last_u0_applied[1:4]
            else:
                # MPC silente / decollo PX4: usa i comandi denormalizzati di PX4 e portati in FLU
                f_cmd = self.current_px4_thrust[2]
                tau_cmd = self.current_px4_torque

            F_ext, Tau_ext = self.mbe.update(
                self.current_vel,
                self.current_ang_vel,
                self.current_quat,
                f_cmd,
                tau_cmd
            )
            est_w = Wrench()
            est_w.force.x, est_w.force.y, est_w.force.z = float(F_ext[0]), float(F_ext[1]), float(F_ext[2])
            est_w.torque.x, est_w.torque.y, est_w.torque.z = float(Tau_ext[0]), float(Tau_ext[1]), float(Tau_ext[2])
            self.estimated_wrench_pub.publish(est_w)

            self.current_F_ext = F_ext
            self.current_Tau_ext = Tau_ext

        # Solver non ancora pronto: MBE è già aggiornato, torna al prossimo tick
        ready = self.acados_solver_ready

        if not ready:
            self.get_logger().info("Compilazione MPC in corso...", throttle_duration_sec=5.0)
            self.planner_configure()
            return

        # Il decollo è gestito da offboard_trajectory_planner.
        # Se l'MPC non è ancora attivo o non abbiamo un u_plan, fermiamoci qui.
        if not self.task_started:
            return
        
        # --- Calcolo stato, orientamento e riferimenti ---
        self.R = Rotation.from_euler('xyz', self.current_rpy).as_matrix()

        # Costruzione dello stato aumentato [p, v, q, w, e_int] (16 componenti)
        # In modalità planner (is_controller=False), usiamo la predizione nominale al passo precedente
        if not self.is_controller and self.x_plan is not None and not self._first_mpc_solve:
            xk = self.x_plan[1].copy()
            q_norm = np.linalg.norm(xk[6:10])
            if q_norm > 1e-6:
                xk[6:10] /= q_norm
        else:
            xk = np.array([
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_vel[0], self.current_vel[1], self.current_vel[2],
                self.current_quat[0], self.current_quat[1], self.current_quat[2], self.current_quat[3],
                self.current_ang_vel[0],  self.current_ang_vel[1],  self.current_ang_vel[2],
                self.e_int[0], self.e_int[1], self.e_int[2]
            ])

        F_ext = self.current_F_ext
        Tau_ext = self.current_Tau_ext

        # Riferimento cilindrico → target cartesiano (fuori dal solver)
        ref_array = self.get_current_ref(xk)
        cyl_ref = ref_array[0:3]
        yaw_ref = ref_array[3]
        beta_ref = cyl_ref[1]
        vel_ref = np.zeros(3)

        # Conversione cyl→cart: il target cartesiano include l'offset camera in world frame
        # Rb corrente usato solo per la correzione camera al passo k;
        # sull'orizzonte p_target si propaga con v_obj (errore O(N*ts*dRb/dt) ≈ 0)
        cam_world_offset = self.R @ self.camera_offset
        p_target = self.current_obj_pos + cylindrical_to_cartesian(cyl_ref) - cam_world_offset

        # Pubblica riferimento cilindrico per monitoraggio (include yaw_offset)
        self.ref_pub.publish(Float64MultiArray(data=[float(x) for x in ref_array]))

        # Calcola p_cam e p_rel per l'errore integrale cartesiano
        p_drone = xk[0:3]
        q_drone = xk[6:10]
        # Scipy Rotation.from_quat usa [x, y, z, w], CasADi usa [w, x, y, z]
        Rb = Rotation.from_quat([q_drone[1], q_drone[2], q_drone[3], q_drone[0]]).as_matrix()
        p_cam = p_drone + Rb @ self.camera_offset

        p_obj_now = self.current_obj_pos
        p_rel = p_cam - p_obj_now

        # yaw_desired: la telecamera punta verso l'oggetto (distanza ~ r_cyl > 0, mai singolarità)
        p_cam_to_obj = self.current_obj_pos - p_cam
        yaw_desired = float(wrap_pi(np.arctan2(
            p_cam_to_obj[1],
            p_cam_to_obj[0]
        ) + yaw_ref))
        self.last_yaw_desired = yaw_desired

        # Aggiornamento accumulatore errore integrale cartesiano [ex, ey, ez] con anti-windup
        # Attivo solo in modalità controller senza MBE; altrimenti mantenuto a zero
        if self.is_controller and not self.use_mbe:
            p_rel_target = cylindrical_to_cartesian(cyl_ref)
            err_cart_now = p_rel_target - p_rel
            self.e_int += err_cart_now * self.ts
            self.e_int = np.clip(self.e_int, -2.0, 2.0)
        else:
            self.e_int = np.zeros(3)
        
        # Pubblica l'azione integrale
        int_msg = Vector3()
        int_msg.x = float(self.e_int[0])
        int_msg.y = float(self.e_int[1])
        int_msg.z = float(self.e_int[2])
        self.integral_action_pub.publish(int_msg)

        try:
            t_start = time.perf_counter()
            # Warm-start al primo solve dopo lo switch
            if self._first_mpc_solve:
                self._first_mpc_solve = False
                for i in range(self.N_horiz + 1):
                    self.ocp_solver.set(i, "x", xk)
                    self.x_prev[i] = xk.copy()
                for i in range(self.N_horiz):
                    self.ocp_solver.set(i, "u", self.u_hover)
                    self.u_prev[i] = self.u_hover.copy()
                self.get_logger().info("[MPC] Warm-start con stato corrente applicato.")

            u0_new, x_seq_new, yref0, u_plan_new, x_plan_new = self.solve_MPC(
                xk, p_target, yaw_desired, vel_ref, F_ext, Tau_ext)
            t_end = time.perf_counter()
            
            # Calcolo tempo di risoluzione dell'iterazione corrente dell'MPC 
            dt_solve = t_end - t_start
            
            # Pubblica tempo di solve [ms] per monitoraggio e logger
            st_msg = Float64()
            st_msg.data = float(dt_solve * 1e3)
            self.solve_time_pub.publish(st_msg)
            
            self.u_plan = u_plan_new
            self.x_plan = x_plan_new
            u0 = u0_new
            next_x = self.x_plan[1]
                
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

            # Safe Switch Check
            if not self.safety_switch_passed:
                if not self.is_controller:
                    # In modalità planner non controlliamo spinta/coppie: switch immediato
                    self.get_logger().info("Safe Switch OK (Planner mode: pos/vel setpoints)! L'MPC prende il controllo di PX4.")
                    self.safety_switch_passed = True
                else:
                    u_px4 = np.array([self.current_px4_thrust[2], self.current_px4_torque[0], self.current_px4_torque[1], self.current_px4_torque[2]])
                    
                    # Se non riceviamo dati da PX4, usiamo hover come fallback
                    if self.current_px4_thrust[2] == 0.0:
                        u_px4 = self.u_hover
                        
                    err_thrust = abs(u0[0] - u_px4[0])
                    err_torque = np.linalg.norm(u0[1:4] - u_px4[1:4])
                    
                    # Thresholds
                    thrust_thresh = 2  # Newton (circa 10% della spinta di hovering)
                    torque_thresh = 0.2  # Nm (margine sufficiente per evitare scatti angolari)
                    
                    if err_thrust < thrust_thresh and err_torque < torque_thresh:
                        self.get_logger().info(f"Safe Switch OK! (err_thrust={err_thrust:.2f}N, err_torque={err_torque:.3f}Nm). L'MPC prende il controllo di PX4!")
                        self.safety_switch_passed = True
                    else:
                        self.get_logger().warn(f"Safe Switch FALLITO. (err_thrust={err_thrust:.2f}N > {thrust_thresh} o err_torque={err_torque:.3f}Nm > {torque_thresh}). Attendo convergenza...", throttle_duration_sec=1.0)
                        return

            if self.is_controller:
                self.publish_optimal_wrench(u0)
            else:
                self.publish_trajectory_setpoint(next_x, yaw_desired)
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

                # vel_ref = zeros: lo pubblichiamo per il logger per non farlo rimanere "incastrato" all'ultimo valore del planner
                twist_ref_msg = TwistStamped()
                twist_ref_msg.header.stamp = self.get_clock().now().to_msg()
                twist_ref_msg.header.frame_id = 'world'
                twist_ref_msg.twist.linear.x = float(vel_ref[0])
                twist_ref_msg.twist.linear.y = float(vel_ref[1])
                twist_ref_msg.twist.linear.z = float(vel_ref[2])
                twist_ref_msg.twist.angular.x = 0.0
                twist_ref_msg.twist.angular.y = 0.0
                twist_ref_msg.twist.angular.z = 0.0
                self.vel_ref_pub.publish(twist_ref_msg)

        except Exception as e:
            self.get_logger().error(f"Errore nel solver: {e}")
            return



    # ==================== Funzioni PX4 Controllo Integrato ====================

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        if self.is_controller:
            msg.position = False
            msg.velocity = False
            msg.thrust_and_torque = True
        else:
            msg.position = True
            msg.velocity = True
            msg.thrust_and_torque = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        #msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = 0  # PX4 auto-compila con hrt_absolute_time()
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, next_x, yaw_desired_enu):
        """Pubblica setpoint di posizione e velocità per PX4 (modalità Planner)."""
        self.publish_offboard_control_mode()

        # Conversione ENU -> NED tramite matrice self.M_enu2ned già esistente
        local_enu = np.array([
            next_x[0] - self.start_x,
            next_x[1] - self.start_y,
            next_x[2] - self.start_z
        ])
        pos_ned = self.M_enu2ned @ local_enu
        vel_ned = self.M_enu2ned @ next_x[3:6]

        # Convenzione yaw NED
        yaw_ned = float(-yaw_desired_enu + np.pi / 2.0)

        msg = TrajectorySetpoint()
        #msg.timestamp = 0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        msg.yaw = yaw_ned
        self.trajectory_setpoint_pub.publish(msg)

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
        # Aggiorna il controllo applicato per il MBE
        self.last_u0_applied = u0.copy()

        # Pubblichiamo su /optimal_wrench per il logger
        wrench_msg = Wrench()
        wrench_msg.force.z = float(u0[0])
        wrench_msg.torque.x = float(u0[1])
        wrench_msg.torque.y = float(u0[2])
        wrench_msg.torque.z = float(u0[3])
        self.single_wrench_pub.publish(wrench_msg)

        self.publish_offboard_control_mode()
 
            
        # Parametri per la linearizzazione (recuperati dai parametri del nodo)
        #w_max = self.get_parameter('w_max').value
        #Maw_min = self.get_parameter('w_min').value
        # Calcoliamo la velocità angolare desiderata [0, w_max] rad/s
        #w_target = w_max * np.sqrt(max(0.0, float(u0[0])) / self.U_F)
        # Mappatura su norm_thrust [0, 1] considerando il range dell'airframe [w_min, w_max]
        #norm_thrust = (w_target - w_min) / (w_max - w_min)   
        #norm_thrust = max(0.0, min(1.0, norm_thrust))
        norm_thrust = max(0.0, float(u0[0])/self.U_F)

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
        
        # normalizziamo e clippiamo tra [-1,1] rispetto al massimo per ogni asse (in FLU)
        normalized_torques_flu = np.array([
            np.clip(u0[1] / self.U_TAU_X, -1.0, 1.0),
            np.clip(u0[2] / self.U_TAU_Y, -1.0, 1.0),
            np.clip(u0[3] / self.U_TAU_Z, -1.0, 1.0)
        ])
            
        # rotazione FLU -> FRD
        torques_frd = self.M_flu2frd @ normalized_torques_flu
            
        # message PX4 
        torque_msg.xyz[0] = float(torques_frd[0]) 
        torque_msg.xyz[1] = float(torques_frd[1])
        torque_msg.xyz[2] = float(torques_frd[2])
        self.torque_pub.publish(torque_msg)

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
    #executor = MultiThreadedExecutor()
    #executor.add_node(node)
    try:
        #executor.spin()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
