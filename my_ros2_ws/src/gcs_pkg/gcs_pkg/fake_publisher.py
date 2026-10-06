#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleControlMode, VehicleCommand
from std_msgs.msg import Bool, Float64MultiArray, String, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from scipy.spatial.transform import Rotation
from utils_pkg.planner import generate_trapezoidal_trajectory

class FakePublisherNode(Node):
    def __init__(self):
        super().__init__('fake_publisher_node')
        
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        qos_latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # --- Publishers ---
        #  Odometria fittizia per il peg
        self.odom_pub = self.create_publisher(VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', px4_qos_profile)
        #  Inizio task mpc e logging
        self.task_start_pub = self.create_publisher(Bool, '/mpc_enabled', qos_latched)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)
        #  Offboard Trajectory Planner e POV
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)
        # Stato FSM per il logger e plot_script
        self.state_pub = self.create_publisher(String, '/supervisor/state', qos_latched)
        # PX4 Commands (Arm, Offboard)
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos_profile)
        # Stato attuale drone peg (ENU) — in sim: posizione fissa di hovering
        # In real: pubblicato da offboard_admittance_planner
        self.peg_actual_pose_pub     = self.create_publisher(PoseStamped,  '/peg_actual_pose',     10)
        self.peg_actual_vel_pub      = self.create_publisher(TwistStamped, '/peg_actual_velocity', 10)
        self.peg_actual_yaw_pub     = self.create_publisher(Float64,      '/peg_actual_yaw',      10)
        self.peg_actual_omega_z_pub = self.create_publisher(Float64,      '/peg_actual_omega_z',  10)
        
        # --- Subscribers ---
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, px4_qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom1_cb, px4_qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, px4_qos_profile)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)
        self.create_subscription(String, '/keyboard_input', self.keyboard_cb, 10)
        self.create_subscription(Float64MultiArray, '/actual_pov', self.actual_pov_cb, 10)
        self.actual_pov = None
        
        # --- Parametri di volo ---
        self.declare_parameter('takeoff_alt_1', 4.52+3.0)
        self.declare_parameter('guardrone_start_x', 0.0)
        self.declare_parameter('guardrone_start_y', 0.0)
        self.declare_parameter('guardrone_start_z', 4.52)
        self.declare_parameter('peg_start_x', 0.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 4.52)
        self.declare_parameter('peg_start_yaw', 0.0)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.0)
        self.declare_parameter('peg_z_offset', 0.22)
        # Parametri FoV camera per calcolo offset decentrato
        self.declare_parameter('fov_h_deg', 80.0)
        self.declare_parameter('fov_v_deg', 60.0)
        # Vettori delta cinematici relativi (X, Y, Z) per le fasi del peg
        self.declare_parameter('peg_delta_x_approach', 0.0)
        self.declare_parameter('peg_delta_y_approach', 1.0)
        self.declare_parameter('peg_delta_z_approach', 0.0)
        self.declare_parameter('peg_delta_x_detachment', 0.0)
        self.declare_parameter('peg_delta_y_detachment', -0.5)
        self.declare_parameter('peg_delta_z_detachment', 0.0)

        self.cam_offset_x = self.get_parameter('cam_offset_x').value
        self.cam_offset_y = self.get_parameter('cam_offset_y').value
        self.cam_offset_z = self.get_parameter('cam_offset_z').value
        self.peg_z_offset = self.get_parameter('peg_z_offset').value
        self.fov_h_deg = self.get_parameter('fov_h_deg').value
        self.fov_v_deg = self.get_parameter('fov_v_deg').value
        self.takeoff_alt_1 = self.get_parameter('takeoff_alt_1').value
        self.guardrone_start_x = self.get_parameter('guardrone_start_x').value
        self.guardrone_start_y = self.get_parameter('guardrone_start_y').value
        self.guardrone_start_z = self.get_parameter('guardrone_start_z').value
        self.peg_start_x = self.get_parameter('peg_start_x').value
        self.peg_start_y = self.get_parameter('peg_start_y').value
        self.peg_start_z = self.get_parameter('peg_start_z').value
        self.peg_start_yaw = float(self.get_parameter('peg_start_yaw').value)
        self.peg_delta_x_approach = float(self.get_parameter('peg_delta_x_approach').value)
        self.peg_delta_y_approach = float(self.get_parameter('peg_delta_y_approach').value)
        self.peg_delta_z_approach = float(self.get_parameter('peg_delta_z_approach').value)
        self.peg_delta_x_detachment = float(self.get_parameter('peg_delta_x_detachment').value)
        self.peg_delta_y_detachment = float(self.get_parameter('peg_delta_y_detachment').value)
        self.peg_delta_z_detachment = float(self.get_parameter('peg_delta_z_detachment').value)

        self.peg_delta_approach = np.array([self.peg_delta_x_approach, self.peg_delta_y_approach, self.peg_delta_z_approach], dtype=float)
        self.peg_delta_detachment = np.array([self.peg_delta_x_detachment, self.peg_delta_y_detachment, self.peg_delta_z_detachment], dtype=float)

        
        # Posizione ENU globale del drone, aggiornata continuamente da odom1_cb.
        # Placeholder con i parametri di spawn finché non arriva la prima odometria.
        self.drone_pos_enu = np.array([self.guardrone_start_x, self.guardrone_start_y, self.guardrone_start_z])
        
        # Riferimenti di hovering iniziale (saranno ricalcolati esattamente all'avvio con lo yaw reale)
        dx = float(self.guardrone_start_x + self.cam_offset_x - self.peg_start_x)
        dy = float(self.guardrone_start_y + self.cam_offset_y - self.peg_start_y)
        self.r_hover = math.sqrt(dx**2 + dy**2)
        self.beta_hover = math.atan2(dy, dx)

        # Matrici fisse di conversione frame (da MPC_planner_node.py)
        self.M_ned2enu = np.array([[0.0, 1.0, 0.0], 
                                   [1.0, 0.0, 0.0], 
                                   [0.0, 0.0, -1.0]])
                                   
        self.M_frd2flu = np.array([[1.0, 0.0, 0.0], 
                                   [0.0, -1.0, 0.0], 
                                   [0.0, 0.0, -1.0]])

        # Calcolo quaternioni per lo yaw del peg (in coordinate ENU e PX4 NED)
        rot_peg_flu2enu = Rotation.from_euler('z', self.peg_start_yaw)
        self.peg_enu_q = rot_peg_flu2enu.as_quat()  # [x, y, z, w]
        # Orientamento in PX4 NED (body FRD -> world NED): R_frd2ned = M_ned2enu @ R_flu2enu @ M_frd2flu
        R_peg_frd2ned = self.M_ned2enu @ rot_peg_flu2enu.as_matrix() @ self.M_frd2flu
        q_peg_ned = Rotation.from_matrix(R_peg_frd2ned).as_quat()  # [x, y, z, w]
        # In PX4 VehicleOdometry il formato è [w, x, y, z]
        self.peg_odom_q = [float(q_peg_ned[3]), float(q_peg_ned[0]), float(q_peg_ned[1]), float(q_peg_ned[2])]

        # Variabili di stato interne
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone1_odom = VehicleOdometry()
        self.drone1_mode = VehicleControlMode()
        self.mpc_ready = False
        self.user_ok = False
        self.wait_msg_printed = False
        self.switch_msg_printed = False

        # Posizione e velocita dinamica del fake peg (quota takeoff_alt_1 - peg_z_offset cosicché sommando peg_offset la punta sia a takeoff_alt_1)
        self.fake_peg_pos = np.array([self.peg_start_x, self.peg_start_y, self.takeoff_alt_1 - self.peg_z_offset], dtype=float)
        self.fake_peg_vel = np.zeros(3, dtype=float)
        # Traiettoria trapezoidale continua (utils_pkg.planner)
        self.peg_traj_p = None
        self.peg_traj_v = None
        self.peg_traj_idx = 0
        self.detachment_started = False
        self.approach_started = False
        self.interaction_started = False
        self.return_home_started = False
        self.landing_started = False
        self.msg_cnt = 0
        
        self.state = 'WAIT_EKF'
        self._prev_published_state = None
        self.wait_ticks = 0
        
        # Loop principale a 50Hz (necessario per l'odometria del peg)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Fake Supervisor & Peg Publisher avviato. Stato: WAIT_EKF")

    # --- Callbacks ---
    def pos1_cb(self, msg): self.drone1_local_pos = msg

    def odom1_cb(self, msg):
        self.drone1_odom = msg
        # Mantiene aggiornata la posizione ENU globale del drone (NED locale → ENU + offset spawn)
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.drone_pos_enu = self.M_ned2enu @ pos_ned + np.array([
            self.guardrone_start_x,
            self.guardrone_start_y,
            self.guardrone_start_z,
        ])

    def mode1_cb(self, msg): self.drone1_mode = msg

    def mpc_ready_cb(self, msg):
        if msg.data and not self.mpc_ready:
            self.mpc_ready = True
            self.get_logger().info("Segnale MPC Pronto ricevuto!")

    def actual_pov_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.actual_pov = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2]), float(msg.data[3])]
        elif len(msg.data) == 3:
            self.actual_pov = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2]), 0.0]

    def keyboard_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('ok', 'takeoff', 'start'):
            self.user_ok = True
            self.get_logger().info(f"Comando '{cmd}' ricevuto dal terminale GCS!")
        elif cmd in ('land', 'landing'):
            self.get_logger().warn('Comando LAND ricevuto dal terminale GCS! Avvio sequenza di atterraggio.')
            self.state = 'LANDING'
            self.landing_started = False
            self.user_ok = False
        elif cmd in ('hovering', 'home',
                     'approach', 'mission_start', 'mission_preparation',
                     'interaction', 'inspection_start', 'mission',
                     'detachment', 'inspection_end',
                     'return_home'):
            alias_map = {
                'hovering': 'HOVERING',
                'home': 'HOVERING',
                'approach': 'APPROACH',
                'interaction': 'INTERACTION',
                'detachment': 'DETACHMENT',
                'return_home': 'RETURN_HOME',
            }
            target_st = alias_map[cmd]
            self.get_logger().warn(f'COMANDO DIRETTO FASE RICEVUTO: "{cmd}" -> Salto immediato a {target_st}')
            self.state = target_st
            self.user_ok = False
            self.approach_started = False
            self.interaction_started = False
            self.detachment_started = False
            self.return_home_started = False
            self.msg_cnt = 0
        elif cmd == 'stop':
            self.get_logger().error("Comando STOP ricevuto! Atterraggio d'emergenza!")
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'

    def compute_actual_pov_target(self):
        """Restituisce il target PoV (r, beta, z) letto dal topic /actual_pov dell'MPC,
        oppure i valori stimati di hovering se /actual_pov non è ancora disponibile."""
        if self.actual_pov is not None:
            return self.actual_pov[0], self.actual_pov[1], self.actual_pov[2]
        return self.r_hover, self.beta_hover, 0.0

    def compute_decentered_pov(self, r, h_fraction=0.0, v_fraction=0.0):
        """Calcola yaw_offset e z_rel per decentrare l'oggetto nel FoV.

        h_fraction in [-1, 1]: +1 = bordo destro, -1 = bordo sinistro
        v_fraction in [-1, 1]: +1 = bordo superiore, -1 = bordo inferiore
        Restituisce (yaw_offset, z_rel) da passare a publish_pov_target.
        """
        fov_h_half = math.radians(self.get_parameter('fov_h_deg').value / 2.0)
        fov_v_half = math.radians(self.get_parameter('fov_v_deg').value / 2.0)

        yaw_offset = h_fraction * fov_h_half
        z_max = r * math.cos(yaw_offset) * math.tan(fov_v_half)
        z_rel = v_fraction * z_max

        return yaw_offset, z_rel

    def publish_pov_target(self, r, beta, z=0.0, yaw=0.0):
        """Pubblica riferimento PoV cilindrico [r, beta, z_rel, yaw_offset] per l'MPC."""
        msg = Float64MultiArray()
        msg.data = [float(r), float(beta), float(z), float(yaw)]
        self.pov_pub.publish(msg)
        self.get_logger().info(f"Target PoV inviato all'MPC: r={r:.3f} m, beta={math.degrees(beta):.1f}°, z={z:.3f} m, yaw={math.degrees(yaw):.1f}°")

    def _publish_state_if_changed(self):
        """Pubblica lo stato della FSM su /supervisor/state per il logger."""
        if self.state != self._prev_published_state:
            self._prev_published_state = self.state
            msg = String()
            msg.data = self.state
            self.state_pub.publish(msg)
            self.get_logger().info(f"[FSM] Transizione di stato pubblicata: {self.state}")

    def publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub_1.publish(msg)

    def plan_peg_trajectory(self, target_pos, v_max=0.2, a_max=0.2):
        """Genera profilo trapezoidale continuo per il fake peg a 50Hz (dt=0.02s)."""
        x0 = [float(self.fake_peg_pos[0]), float(self.fake_peg_pos[1]), float(self.fake_peg_pos[2]), 0.0, 0.0, 0.0]
        x_ref = [float(target_pos[0]), float(target_pos[1]), float(target_pos[2]), 0.0, 0.0, 0.0]
        t_vec, p_vals, _ = generate_trapezoidal_trajectory(x0, x_ref, dt=0.02, v_max=v_max, a_max=a_max)
        self.peg_traj_p = p_vals
        if len(p_vals) > 1:
            self.peg_traj_v = np.gradient(p_vals, 0.02, axis=0)
        else:
            self.peg_traj_v = np.zeros_like(p_vals)
        self.peg_traj_idx = 0
        return float(t_vec[-1])

    def is_peg_trajectory_done(self):
        """Verifica se la traiettoria trapezoidale in corso ha raggiunto la destinazione."""
        return self.peg_traj_p is not None and self.peg_traj_idx >= len(self.peg_traj_p)

    def timer_callback(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        
        # Aggiornamento cinematica trapezoidale del peg a 50Hz
        if self.peg_traj_p is not None:
            if self.peg_traj_idx < len(self.peg_traj_p):
                self.fake_peg_pos = self.peg_traj_p[self.peg_traj_idx].copy()
                self.fake_peg_vel = self.peg_traj_v[self.peg_traj_idx].copy()
                self.peg_traj_idx += 1
            else:
                self.fake_peg_pos = self.peg_traj_p[-1].copy()
                self.fake_peg_vel[:] = 0.0
        
        # =========================================================================
        # PUBBLICAZIONE COSTANTE ODOMETRIA PEG (50Hz)
        # =========================================================================
        peg_odom_msg = VehicleOdometry()
        peg_odom_msg.timestamp = int(now.nanoseconds / 1000)
        
        # Posizione del peg (coordinate NED relative al punto di spawn del peg)
        delta_enu = self.fake_peg_pos - np.array([self.peg_start_x, self.peg_start_y, self.peg_start_z])
        delta_ned = self.M_ned2enu @ delta_enu
        peg_odom_msg.position = [float(delta_ned[0]), float(delta_ned[1]), float(delta_ned[2])]
        peg_odom_msg.q = self.peg_odom_q
        vel_ned = self.M_ned2enu @ self.fake_peg_vel
        peg_odom_msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        peg_odom_msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(peg_odom_msg)

        # =========================================================================
        # PUBBLICAZIONE STATO ATTUALE PEG IN ENU (50Hz)
        # =========================================================================
        peg_pose_msg = PoseStamped()
        peg_pose_msg.header.stamp = stamp
        peg_pose_msg.header.frame_id = 'world'
        peg_pose_msg.pose.position.x = float(self.fake_peg_pos[0])
        peg_pose_msg.pose.position.y = float(self.fake_peg_pos[1])
        peg_pose_msg.pose.position.z = float(self.fake_peg_pos[2])
        peg_pose_msg.pose.orientation.x = float(self.peg_enu_q[0])
        peg_pose_msg.pose.orientation.y = float(self.peg_enu_q[1])
        peg_pose_msg.pose.orientation.z = float(self.peg_enu_q[2])
        peg_pose_msg.pose.orientation.w = float(self.peg_enu_q[3])
        self.peg_actual_pose_pub.publish(peg_pose_msg)

        peg_vel_msg = TwistStamped()
        peg_vel_msg.header.stamp = stamp
        peg_vel_msg.header.frame_id = 'world'
        peg_vel_msg.twist.linear.x = float(self.fake_peg_vel[0])
        peg_vel_msg.twist.linear.y = float(self.fake_peg_vel[1])
        peg_vel_msg.twist.linear.z = float(self.fake_peg_vel[2])
        self.peg_actual_vel_pub.publish(peg_vel_msg)

        yaw_msg = Float64()
        yaw_msg.data = float(self.peg_start_yaw)
        self.peg_actual_yaw_pub.publish(yaw_msg)

        omega_z_msg = Float64()
        omega_z_msg.data = 0.0
        self.peg_actual_omega_z_pub.publish(omega_z_msg)

        # =========================================================================
        # 2) MACCHINA A STATI DEL SUPERVISOR (~10Hz)
        # =========================================================================
        self.wait_ticks += 1
        if self.wait_ticks % 5 != 0:
            return

        self._publish_state_if_changed()
            
        if self.state == 'WAIT_EKF':
            # Controlla la convergenza di PX4 per il drone reale e la ricezione dell'odometria
            d1_ekf_ok = (self.drone1_local_pos.timestamp > 0 and 
                         self.drone1_local_pos.xy_valid and 
                         self.drone1_local_pos.z_valid and 
                         self.drone1_odom.timestamp > 0)
            
            if d1_ekf_ok:
                # Quaternione PX4: rappresenta R_frd2ned (body FRD → world NED)
                q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2], self.drone1_odom.q[3], self.drone1_odom.q[0]] 
                R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

                # Convertiamo in R_flu2enu (body → world in ENU)
                R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
                
                # Ruotiamo l'offset 3D della telecamera
                cam_offset = np.array([self.cam_offset_x, self.cam_offset_y, self.cam_offset_z])
                rotated_offset = R_flu2enu @ cam_offset
                
                # Posizione iniziale esatta della telecamera nel mondo (ENU).
                # drone_pos_enu è già aggiornata da odom1_cb con la posizione corrente.
                cam_spawn_x = self.drone_pos_enu[0] + rotated_offset[0]
                cam_spawn_y = self.drone_pos_enu[1] + rotated_offset[1]

                # Calcoliamo la distanza e l'azimut statici del target
                dx = float(cam_spawn_x - self.peg_start_x)
                dy = float(cam_spawn_y - self.peg_start_y)
                self.r_hover = math.sqrt(dx**2 + dy**2)
                self.beta_hover = math.atan2(dy, dx)
                
                # Estraiamo l'angolo di yaw ENU per la stampa di log
                rot_flu2enu = Rotation.from_matrix(R_flu2enu)
                psi_enu = rot_flu2enu.as_euler('xyz')[2]
                
                self.get_logger().info(
                    f"EKF Convergente. Rilevato yaw iniziale ENU: {math.degrees(psi_enu):.1f}° | "
                    f"Target impostato: r={self.r_hover:.3f}m, beta={math.degrees(self.beta_hover):.1f}°"
                )
                self.state = 'WAIT_TAKEOFF'
                
        elif self.state == 'WAIT_TAKEOFF':
            if self.mpc_ready:
                if not self.wait_msg_printed:
                    self.get_logger().info("Planner di takeoff pronto. Digita 'ok' (e premi invio) sul terminale GCS per autorizzare il decollo.")
                    self.wait_msg_printed = True
                
                if self.user_ok:
                    self.user_ok = False # Consuma il comando
                    self.get_logger().info("Inizio missione automatica. Invio target di takeoff e passo ad ARM_OFFBOARD.")

                    # drone_pos_enu è aggiornata in tempo reale da odom1_cb:
                    # cattura la posizione del drone nell'istante più vicino al decollo.
                    self.get_logger().info(
                        f"Posizione di decollo (ENU): x={self.drone_pos_enu[0]:.3f}, "
                        f"y={self.drone_pos_enu[1]:.3f}"
                    )

                    # Invia target di decollo.
                    # Il drone BODY deve salire a (takeoff_alt_1 - cam_offset_z) in modo
                    # che la CAMERA (offset in z rispetto al body) si trovi esattamente
                    # a takeoff_alt_1, allineata con il peg.
                    cam_body_takeoff_z = float(self.takeoff_alt_1 - self.cam_offset_z)

                    # Yaw corrente del drone in ENU (fotografato all'istante del decollo).
                    # Lo esplicitiamo nel quaternione invece di lasciare q=[0,0,0,0]
                    # (che il trajectory planner interpreta come "mantieni yaw corrente"):
                    # così l'intento è chiaro e robusto a modifiche future del planner.
                    q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2],
                               self.drone1_odom.q[3], self.drone1_odom.q[0]]
                    R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
                    R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
                    current_yaw_enu = float(Rotation.from_matrix(R_flu2enu).as_euler('xyz')[2])
                    yaw_quat = Rotation.from_euler('z', current_yaw_enu).as_quat()  # [x,y,z,w]

                    cam_takeoff_target = PoseStamped()
                    cam_takeoff_target.header.frame_id = 'world'
                    cam_takeoff_target.pose.position.x = float(self.drone_pos_enu[0])
                    cam_takeoff_target.pose.position.y = float(self.drone_pos_enu[1])
                    cam_takeoff_target.pose.position.z = cam_body_takeoff_z
                    cam_takeoff_target.pose.orientation.x = float(yaw_quat[0])
                    cam_takeoff_target.pose.orientation.y = float(yaw_quat[1])
                    cam_takeoff_target.pose.orientation.z = float(yaw_quat[2])
                    cam_takeoff_target.pose.orientation.w = float(yaw_quat[3])
                    self.get_logger().info(f"Target decollo: z={cam_body_takeoff_z:.3f}m, yaw={math.degrees(current_yaw_enu):.1f}°")
                    self.cam_target_pub.publish(cam_takeoff_target)
                    
                    # Accende il trajectory planner
                    msg_traj = Bool()
                    msg_traj.data = True
                    self.cam_traj_enabled_pub.publish(msg_traj)
                    
                    # Segnale di logging start
                    log_start_msg = Bool()
                    log_start_msg.data = True
                    self.logging_start_pub.publish(log_start_msg)
                    
                    self.state = 'ARM_OFFBOARD'
                
        elif self.state == 'ARM_OFFBOARD':
            d1_ready = (self.drone1_mode.flag_control_offboard_enabled and self.drone1_mode.flag_armed)
            
            if not d1_ready:
                # Ripete l'invio dei comandi finché non vengono accettati
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                self.get_logger().info("Drone Armato e in Offboard. Attesa raggiungimento quota di hovering...")
                self.state = 'TAKEOFF_MONITOR'
                
        elif self.state == 'TAKEOFF_MONITOR':
            # drone1_local_pos.z è NED (negativo verso l'alto). Essendo un valore locale, è relativo a guardrone_start_z.
            # La quota target per il body è (takeoff_alt_1 - cam_offset_z).
            peg_local_pos_z = -self.drone1_local_pos.z 
            dist = abs(peg_local_pos_z - (self.takeoff_alt_1 - self.cam_offset_z - self.guardrone_start_z))
            d1_up = dist < 0.10
            self.get_logger().info(f"Distanza dal reference di takeoff: d ={dist:.3f}")

            if d1_up:
                # --- Aggiorna il riferimento in hovering finché si attende lo switch ---
                q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2], self.drone1_odom.q[3], self.drone1_odom.q[0]] 
                R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
                R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
                rotated_offset = R_flu2enu @ np.array([self.cam_offset_x, self.cam_offset_y, self.cam_offset_z])
                cam_x = self.drone_pos_enu[0] + rotated_offset[0]
                cam_y = self.drone_pos_enu[1] + rotated_offset[1]
                dx = float(cam_x - self.peg_start_x)
                dy = float(cam_y - self.peg_start_y)
                self.r_hover = math.sqrt(dx**2 + dy**2)
                self.beta_hover = math.atan2(dy, dx)
                # -----------------------------------------------------------------------

                if not self.switch_msg_printed:
                    self.get_logger().info(f"Drone in quota! Riferimento stimato: r={self.r_hover:.3f}, beta={math.degrees(self.beta_hover):.1f}°. Switch a MPC pronto. Dare ok da tastiera")
                    self.switch_msg_printed = True
                    
                if self.user_ok:
                    self.user_ok = False # Consuma il comando
                    # 1. Spegne offboard trajectory planner
                    msg_traj = Bool()
                    msg_traj.data = False
                    self.cam_traj_enabled_pub.publish(msg_traj)
            
                    # 2. Invia target PoV iniziale per l'MPC (centrato: yaw_offset=0):
                    r_init, beta_init, z_init = self.compute_actual_pov_target()
                    self.publish_pov_target(r_init, beta_init, z_init, 0.0)

                    # 3. Avvia MPC
                    msg_start = Bool()
                    msg_start.data = True
                    self.task_start_pub.publish(msg_start)
                    
                    self.state = 'HOVERING'
                    self.get_logger().info("HOVERING AVVIATO. Hovering mantenuto tramite MPC (PoV centrato).")
                
        elif self.state == 'HOVERING':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in HOVERING: inizio fase di APPROACH...")
                self.state = 'APPROACH'
                self.approach_started = False
                self.msg_cnt = 0
            elif self.msg_cnt == 0:
                self.get_logger().info('HOVERING in corso con MPC (PoV centrato). Digita "ok" per procedere ad APPROACH o "land" per atterrare.')
                self.msg_cnt += 1

        elif self.state == 'APPROACH':
            if not self.approach_started:
                self.approach_started = True
                # Mantiene lo stesso PoV centrato
                r_act, beta_act, z_act = self.compute_actual_pov_target()
                self.publish_pov_target(r_act, beta_act, z_act, 0.0)

                approach_target = np.array([
                    self.peg_start_x,
                    self.peg_start_y,
                    self.takeoff_alt_1 - self.peg_z_offset
                ]) + self.peg_delta_approach
                dur = self.plan_peg_trajectory(approach_target, v_max=0.2, a_max=0.2)
                self.get_logger().info(f"Fase APPROACH avviata (trapezoidale, {dur:.1f}s): peg verso {approach_target}. GuarDrone segue con PoV centrato...")

            if self.is_peg_trajectory_done():
                if self.user_ok:
                    self.user_ok = False
                    self.get_logger().info("Avvicinamento completato + 'ok' ricevuto! Inizio INTERACTION...")
                    self.state = 'INTERACTION'
                    self.interaction_started = False
                    self.msg_cnt = 0
                elif self.msg_cnt == 0:
                    self.get_logger().info('Peg in posizione di avvicinamento. Digita "ok" per procedere ad INTERACTION o "land" per atterrare.')
                    self.msg_cnt += 1

        elif self.state == 'INTERACTION':
            if not self.interaction_started:
                self.interaction_started = True
                # Inquadra in basso a destra (60%) e raggio r = 1.5 m
                _, beta_act, _ = self.compute_actual_pov_target()
                r_target = 1.5
                h_frac = 0.6
                v_frac = 0.6
                yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
                self.publish_pov_target(r_target, beta_act, z_off, yaw_off)
                self.get_logger().info(f"Fase INTERACTION avviata: GuarDrone in PoV decentrato (r={r_target:.2f}m). Peg in teleoperazione (nessun target da nodo).")

            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in INTERACTION: inizio fase di DETACHMENT...")
                self.state = 'DETACHMENT'
                self.detachment_started = False
                self.msg_cnt = 0
            elif self.msg_cnt == 0:
                self.get_logger().info('INTERACTION in corso (teleoperazione). Digita "ok" per procedere al distacco (DETACHMENT) o "land" per atterrare.')
                self.msg_cnt += 1

        elif self.state == 'DETACHMENT':
            if not self.detachment_started:
                self.detachment_started = True
                # Torna a centrare l'inquadratura (z=0, yaw=0) mantenendo r = 1.5 m
                _, beta_act, _ = self.compute_actual_pov_target()
                r_target = 1.5
                self.publish_pov_target(r_target, beta_act, 0.0, 0.0)

                detach_target = self.fake_peg_pos + self.peg_delta_detachment
                dur = self.plan_peg_trajectory(detach_target, v_max=0.2, a_max=0.2)
                self.get_logger().info(f"Distacco avviato (trapezoidale, {dur:.1f}s): peg verso {detach_target}. GuarDrone torna a PoV centrato (r=1.5m)...")

            if self.is_peg_trajectory_done():
                if self.user_ok:
                    self.user_ok = False
                    self.get_logger().info("Distacco completato + 'ok' ricevuto! Inizio RETURN_HOME...")
                    self.state = 'RETURN_HOME'
                    self.return_home_started = False
                    self.msg_cnt = 0
                elif self.msg_cnt == 0:
                    self.get_logger().info('Peg staccato dalla parete in sicurezza. Digita "ok" per procedere a RETURN_HOME (o "land" per atterrare)...')
                    self.msg_cnt += 1

        elif self.state == 'RETURN_HOME':
            if not self.return_home_started:
                self.return_home_started = True
                # Torna a centrare l'inquadratura (z=0, yaw=0) con raggio allargato a r = 2.0 m
                _, beta_act, _ = self.compute_actual_pov_target()
                r_target = 2.0
                self.publish_pov_target(r_target, beta_act, 0.0, 0.0)

                home_target = np.array([
                    self.peg_start_x,
                    self.peg_start_y,
                    self.takeoff_alt_1 - self.peg_z_offset
                ])
                dur = self.plan_peg_trajectory(home_target, v_max=0.3, a_max=0.2)
                self.get_logger().info(f"Ritorno alla base avviato (trapezoidale, {dur:.1f}s): peg verso {home_target}. GuarDrone allarga raggio a 2.0m...")

            if self.is_peg_trajectory_done():
                if self.user_ok:
                    self.user_ok = False
                    self.get_logger().info("Ritorno completato + 'ok' ricevuto! Inizio LANDING...")
                    self.state = 'LANDING'
                    self.landing_started = False
                    self.msg_cnt = 0
                elif self.msg_cnt == 0:
                    self.get_logger().info('Peg tornato alla base sopra lo spawn. Digita "ok" per procedere a LANDING...')
                    self.msg_cnt += 1

        elif self.state == 'LANDING':
            # Assicura che la transizione a LANDING sia pubblicata prima di DISARM_WAIT
            self._publish_state_if_changed()

            # Stop MPC Task e Trajectory Planner su GuarDrone
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)

            # Comando di Atterraggio nativo PX4
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Comando di Atterraggio nativo PX4 (VEHICLE_CMD_NAV_LAND) inviato a GuarDrone. Stop MPC e attesa disarmo...")
            self.state = 'DISARM_WAIT'

        elif self.state == 'DISARM_WAIT':
            if not self.drone1_mode.flag_armed:
                self.get_logger().info("GuarDrone atterrato e DISARMATO. Missione completata con successo!")
                self.state = 'MISSION_COMPLETE'

        elif self.state == 'MISSION_COMPLETE':
            pass

        elif self.state == 'EMERGENCY':
            # Stop MPC e Trajectory Planner per evitare conflitti con autoland PX4
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)
            # Continua a inviare land
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

def main(args=None):
    rclpy.init(args=args)
    node = FakePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
