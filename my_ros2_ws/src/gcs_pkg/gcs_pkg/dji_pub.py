#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dji_pub.py - DJI Target Bridge & Mission Supervisor for GuarDrone.

Sostituisce il target fittizio (fake peg) con un drone reale (es. DJI)
tracciato dal sistema OptiTrack Motion Capture tramite il pacchetto optitrack_listener2.

Funzionalità principali:
1. Sottoscrizione a /optitrack/body_<dji_id>/odometry (nav_msgs/Odometry).
2. Filtraggio e stima delle velocità lineari e angolari del DJI (mocap pubblica solo pose).
3. Conversione di frame e ri-pubblicazione dell'odometria su:
   - /px4_1/fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry in NED per MPC_planner_node)
   - /peg_actual_pose, /peg_actual_velocity, /peg_actual_yaw, /peg_actual_omega_z (per RViz e logger)
4. Compensazione opzionale dell'offset end-effector 'peg_offset' aggiunto da MPC_planner_node.
5. Macchina a stati FSM (WAIT_EKF, WAIT_TAKEOFF, ARM_OFFBOARD, TAKEOFF_MONITOR,
   HOVERING, APPROACH, INTERACTION, DETACHMENT, RETURN_HOME, LANDING) con comandi da tastiera.
6. Prefiltraggio e transizioni fluide (smoothstep) per i cambi di PoV tra le fasi.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleControlMode, VehicleCommand
from std_msgs.msg import Bool, Float64MultiArray, String, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped

from utils_pkg.utils_np import min_angle, wrap_pi


class DJIPublisherNode(Node):
    def __init__(self):
        super().__init__('dji_publisher_node')

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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # =====================================================================
        # PARAMETRI
        # =====================================================================
        self.declare_parameter('dji_id', 4)
        self.declare_parameter('dji_topic', '')
        self.declare_parameter('takeoff_alt_1', 1.50)
        self.declare_parameter('guardrone_start_x', 0.0)
        self.declare_parameter('guardrone_start_y', 0.0)
        self.declare_parameter('guardrone_start_z', 0.0)
        self.declare_parameter('peg_start_x', 0.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 0.0)

        self.declare_parameter('cam_offset_x', 0.105)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', -0.15)
        self.declare_parameter('fov_h_deg', 85.2)
        self.declare_parameter('fov_v_deg', 58.0)

        # Lettura parametri
        self.dji_id = int(self.get_parameter('dji_id').value)
        explicit_topic = str(self.get_parameter('dji_topic').value).strip()
        self.dji_topic = explicit_topic if explicit_topic else f'/optitrack/body_{self.dji_id}/odometry'

        self.takeoff_alt_1 = float(self.get_parameter('takeoff_alt_1').value)
        self.guardrone_start_x = float(self.get_parameter('guardrone_start_x').value)
        self.guardrone_start_y = float(self.get_parameter('guardrone_start_y').value)
        self.guardrone_start_z = float(self.get_parameter('guardrone_start_z').value)
        self.peg_start_x = float(self.get_parameter('peg_start_x').value)
        self.peg_start_y = float(self.get_parameter('peg_start_y').value)
        self.peg_start_z = float(self.get_parameter('peg_start_z').value)

        self.cam_offset_x = float(self.get_parameter('cam_offset_x').value)
        self.cam_offset_y = float(self.get_parameter('cam_offset_y').value)
        self.cam_offset_z = float(self.get_parameter('cam_offset_z').value)
        self.fov_h_deg = float(self.get_parameter('fov_h_deg').value)
        self.fov_v_deg = float(self.get_parameter('fov_v_deg').value)

        # =====================================================================
        # MATRICI TRASFORMAZIONE FRAME
        # =====================================================================
        self.M_ned2enu = np.array([
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0],
            [0.0, 0.0, -1.0]
        ], dtype=float)

        self.M_frd2flu = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0]
        ], dtype=float)

        self.M_enu2ned = self.M_ned2enu.T
        self.M_flu2frd = self.M_frd2flu.T

        # =====================================================================
        # PUBLISHERS
        # =====================================================================
        # Odometria target per MPC_planner_node
        self.odom_pub = self.create_publisher(VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', px4_qos_profile)

        # Abilitazione MPC e Logging
        self.task_start_pub = self.create_publisher(Bool, '/mpc_enabled', qos_latched)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)

        # Offboard Trajectory Planner (fase di decollo GuarDrone)
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)

        # Setpoint PoV cilindrico per MPC [r, beta, z_rel, yaw_offset]
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)

        # Stato FSM per il logger e plot_script
        self.state_pub = self.create_publisher(String, '/supervisor/state', qos_latched)

        # Comandi PX4 per GuarDrone
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos_profile)

        # Stato del target in ENU per RViz e logger.py
        self.peg_actual_pose_pub     = self.create_publisher(PoseStamped,  '/peg_actual_pose',     10)
        self.peg_actual_vel_pub      = self.create_publisher(TwistStamped, '/peg_actual_velocity', 10)
        self.peg_actual_yaw_pub      = self.create_publisher(Float64,      '/peg_actual_yaw',      10)
        self.peg_actual_omega_z_pub  = self.create_publisher(Float64,      '/peg_actual_omega_z',  10)

        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        # Sottoscrizione a OptiTrack
        self.create_subscription(Odometry, self.dji_topic, self.optitrack_cb, sensor_qos)

        # Sottoscrizioni stato GuarDrone (PX4)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, px4_qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom1_cb, px4_qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, px4_qos_profile)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)
        self.create_subscription(String, '/keyboard_input', self.keyboard_cb, 10)
        self.create_subscription(Float64MultiArray, '/actual_pov', self.actual_pov_cb, 10)

        # =====================================================================
        # STATO INTERNO
        # =====================================================================
        # GuarDrone
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone1_odom = VehicleOdometry()
        self.drone1_mode = VehicleControlMode()
        self.drone_pos_enu = np.array([self.guardrone_start_x, self.guardrone_start_y, self.guardrone_start_z], dtype=float)
        self.mpc_ready = False
        self.user_ok = False
        self.wait_msg_printed = False
        self.switch_msg_printed = False
        self.actual_pov = None

        # DJI OptiTrack
        self.optitrack_received = False
        self.last_optitrack_time = 0.0
        self.dji_pos_enu = np.array([self.peg_start_x, self.peg_start_y, self.peg_start_z], dtype=float)
        self.dji_vel_enu = np.zeros(3, dtype=float)
        self.dji_enu_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.dji_rot_flu2enu = Rotation.identity()
        self.dji_omega_flu = np.zeros(3, dtype=float)
        self.dji_yaw = 0.0
        self.dji_omega_z = 0.0

        self.last_p = None
        self.last_rot = None
        self.last_yaw = 0.0

        # Hovering references
        self.r_hover = 2.0
        self.beta_hover = 0.0

        # FSM
        self.state = 'WAIT_EKF'
        self._prev_published_state = None
        self.wait_ticks = 0
        self.msg_cnt = 0
        self.landing_started = False

        # Timer principale a 50Hz (20ms)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            f"DJI Publisher & Supervisor avviato.\n"
            f" -> Sottoscritto a topic OptiTrack: {self.dji_topic}\n"
            f"Stato iniziale FSM: WAIT_EKF"
        )

    # =========================================================================
    # OPTITRACK CALLBACK & VELOCITY ESTIMATION
    # =========================================================================
    def optitrack_cb(self, msg: Odometry):
        # Calcolo timestamp corrente
        t_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if t_sec <= 0.0:
            t_sec = float(self.get_clock().now().nanoseconds) * 1e-9

        # Lettura posizione e orientamento ENU
        p = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=float)

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        norm_q = np.linalg.norm(q)
        if norm_q > 1e-6:
            q = [c / norm_q for c in q]
        else:
            q = [0.0, 0.0, 0.0, 1.0]

        rot = Rotation.from_quat(q)
        yaw = float(rot.as_euler('xyz')[2])

        if not self.optitrack_received:
            self.dji_pos_enu = p
            self.dji_vel_enu = np.zeros(3, dtype=float)
            self.dji_enu_q = np.array(q, dtype=float)
            self.dji_rot_flu2enu = rot
            self.dji_omega_flu = np.zeros(3, dtype=float)
            self.dji_yaw = yaw
            self.dji_omega_z = 0.0

            self.last_p = p.copy()
            self.last_rot = rot
            self.last_yaw = yaw
            self.last_optitrack_time = t_sec
            self.optitrack_received = True
            self.get_logger().info(
                f"[OptiTrack] Primo pacchetto ricevuto da {self.dji_topic}! "
                f"Pos ENU: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}], Yaw: {math.degrees(yaw):.1f}°"
            )
            return

        dt = t_sec - self.last_optitrack_time
        if 1e-4 < dt < 0.2:
            # 1. Derivata numerica velocità lineare con filtro passa-basso esponenziale
            v_raw = (p - self.last_p) / dt
            alpha_v = 0.35
            self.dji_vel_enu = alpha_v * v_raw + (1.0 - alpha_v) * self.dji_vel_enu

            # 2. Derivata rotazione: R_rel = rot * last_rot^T
            R_rel = rot.as_matrix() @ self.last_rot.as_matrix().T
            rot_vec = Rotation.from_matrix(R_rel).as_rotvec()
            omega_world_raw = rot_vec / dt
            omega_flu_raw = rot.as_matrix().T @ omega_world_raw
            alpha_w = 0.35
            self.dji_omega_flu = alpha_w * omega_flu_raw + (1.0 - alpha_w) * self.dji_omega_flu

            # 3. Derivata yaw e yaw rate
            dyaw = min_angle(yaw - self.last_yaw)
            omega_z_raw = dyaw / dt
            self.dji_omega_z = alpha_w * omega_z_raw + (1.0 - alpha_w) * self.dji_omega_z

            self.last_p = p.copy()
            self.last_rot = rot
            self.last_yaw = yaw
            self.last_optitrack_time = t_sec

        self.dji_pos_enu = p
        self.dji_enu_q = np.array(q, dtype=float)
        self.dji_rot_flu2enu = rot
        self.dji_yaw = yaw

    # =========================================================================
    # SUBSCRIBER CALLBACKS
    # =========================================================================
    def pos1_cb(self, msg):
        self.drone1_local_pos = msg

    def odom1_cb(self, msg):
        self.drone1_odom = msg
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=float)
        self.drone_pos_enu = self.M_ned2enu @ pos_ned + np.array([
            self.guardrone_start_x,
            self.guardrone_start_y,
            self.guardrone_start_z,
        ], dtype=float)

    def mode1_cb(self, msg):
        self.drone1_mode = msg

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
            self.get_logger().warn("Comando LAND ricevuto dal terminale GCS! Avvio sequenza di atterraggio.")
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
            self.get_logger().warn(f"COMANDO DIRETTO FASE RICEVUTO: '{cmd}' -> Transizione immediata a {target_st}")
            self.state = target_st
            self.user_ok = False
            self.msg_cnt = 0
            self.apply_phase_pov(target_st)
        elif cmd == 'stop':
            self.get_logger().error("Comando STOP ricevuto! Atterraggio d'emergenza!")
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'

    # =========================================================================
    # POV UTILITIES (STEP REFERENCES FOR MPC)
    # =========================================================================
    def compute_actual_pov_target(self):
        """Restituisce il target PoV (r, beta, z) corrente letto da /actual_pov o da self.r_hover/beta_hover."""
        if self.actual_pov is not None:
            return self.actual_pov[0], self.actual_pov[1], self.actual_pov[2]
        return self.r_hover, self.beta_hover, 0.0

    def compute_decentered_pov(self, r, h_fraction=0.0, v_fraction=0.0):
        """Calcola yaw_offset e z_rel per decentrare l'oggetto nel FoV della telecamera."""
        fov_h_half = math.radians(self.fov_h_deg / 2.0)
        fov_v_half = math.radians(self.fov_v_deg / 2.0)

        yaw_offset = h_fraction * fov_h_half
        z_max = r * math.cos(yaw_offset) * math.tan(fov_v_half)
        z_rel = v_fraction * z_max

        return yaw_offset, z_rel

    def publish_pov_target(self, r, beta, z=0.0, yaw=0.0):
        """Pubblica riferimento PoV cilindrico [r, beta, z_rel, yaw_offset] per l'MPC."""
        msg = Float64MultiArray()
        msg.data = [float(r), float(beta), float(z), float(yaw)]
        self.pov_pub.publish(msg)
        self.get_logger().info(
            f"Target PoV inviato all'MPC (step): r={r:.3f} m, beta={math.degrees(beta):.1f}°, z={z:.3f} m, yaw={math.degrees(yaw):.1f}°"
        )

    def apply_phase_pov(self, phase_name):
        """Pubblica direttamente lo step di riferimento PoV per la fase richiesta."""
        r_act, beta_act, z_act_ = self.compute_actual_pov_target()

        if phase_name == 'HOVERING':
            self.publish_pov_target(r_act, beta_act, 0.0, 0.0)
        elif phase_name == 'APPROACH':
            self.publish_pov_target(r_act + 0.5, beta_act, 0.0, 0.0)
        elif phase_name == 'INTERACTION':
            r_target = r_act - 0.5
            h_frac = 0.70
            v_frac = 0.70
            yaw_off, z_off = self.compute_decentered_pov(r_target, h_frac, v_frac)
            self.publish_pov_target(r_target, beta_act, z_off, yaw_off)
        elif phase_name == 'DETACHMENT':
            self.publish_pov_target(r_act+0.5, beta_act, 0.0, 0.0)
        elif phase_name == 'RETURN_HOME':
            self.publish_pov_target(+0,5, beta_act, 0.0, 0.0)

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

    # =========================================================================
    # MAIN LOOP (50Hz)
    # =========================================================================
    def timer_callback(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # =====================================================================
        # 2. PUBBLICAZIONE STATO DJI & ODOMETRIA TARGET PER MPC (50Hz)
        # =====================================================================
        if self.optitrack_received:
            # Calcolo orientamento PX4 FRD->NED
            R_flu2enu = self.dji_rot_flu2enu.as_matrix()
            R_frd2ned = self.M_enu2ned @ R_flu2enu @ self.M_frd2flu
            q_frd2ned = Rotation.from_matrix(R_frd2ned).as_quat()  # [x, y, z, w]
            q_px4 = [float(q_frd2ned[3]), float(q_frd2ned[0]), float(q_frd2ned[1]), float(q_frd2ned[2])]

            # Posizione del target (coordinate NED relative allo spawn del peg)
            delta_enu = self.dji_pos_enu - np.array([self.peg_start_x, self.peg_start_y, self.peg_start_z], dtype=float)
            p_ned = self.M_enu2ned @ delta_enu
            v_ned = self.M_enu2ned @ self.dji_vel_enu
            omega_frd = self.M_flu2frd @ self.dji_omega_flu

            # Pubblicazione su /px4_1/fmu/out/vehicle_odometry per l'MPC
            peg_odom_msg = VehicleOdometry()
            peg_odom_msg.timestamp = int(now.nanoseconds / 1000)
            peg_odom_msg.position = [float(p_ned[0]), float(p_ned[1]), float(p_ned[2])]
            peg_odom_msg.q = q_px4
            peg_odom_msg.velocity = [float(v_ned[0]), float(v_ned[1]), float(v_ned[2])]
            peg_odom_msg.angular_velocity = [float(omega_frd[0]), float(omega_frd[1]), float(omega_frd[2])]
            self.odom_pub.publish(peg_odom_msg)

            # Pubblicazione stato ENU per RViz e logger
            peg_pose_msg = PoseStamped()
            peg_pose_msg.header.stamp = stamp
            peg_pose_msg.header.frame_id = 'world'
            peg_pose_msg.pose.position.x = float(self.dji_pos_enu[0])
            peg_pose_msg.pose.position.y = float(self.dji_pos_enu[1])
            peg_pose_msg.pose.position.z = float(self.dji_pos_enu[2])
            peg_pose_msg.pose.orientation.x = float(self.dji_enu_q[0])
            peg_pose_msg.pose.orientation.y = float(self.dji_enu_q[1])
            peg_pose_msg.pose.orientation.z = float(self.dji_enu_q[2])
            peg_pose_msg.pose.orientation.w = float(self.dji_enu_q[3])
            self.peg_actual_pose_pub.publish(peg_pose_msg)

            peg_vel_msg = TwistStamped()
            peg_vel_msg.header.stamp = stamp
            peg_vel_msg.header.frame_id = 'world'
            peg_vel_msg.twist.linear.x = float(self.dji_vel_enu[0])
            peg_vel_msg.twist.linear.y = float(self.dji_vel_enu[1])
            peg_vel_msg.twist.linear.z = float(self.dji_vel_enu[2])
            peg_vel_msg.twist.angular.x = float(self.dji_omega_flu[0])
            peg_vel_msg.twist.angular.y = float(self.dji_omega_flu[1])
            peg_vel_msg.twist.angular.z = float(self.dji_omega_flu[2])
            self.peg_actual_vel_pub.publish(peg_vel_msg)

            yaw_msg = Float64()
            yaw_msg.data = float(self.dji_yaw)
            self.peg_actual_yaw_pub.publish(yaw_msg)

            omega_z_msg = Float64()
            omega_z_msg.data = float(self.dji_omega_z)
            self.peg_actual_omega_z_pub.publish(omega_z_msg)

        # =====================================================================
        # 3. MACCHINA A STATI DEL SUPERVISOR (~10Hz)
        # =====================================================================
        self.wait_ticks += 1
        if self.wait_ticks % 5 != 0:
            return

        self._publish_state_if_changed()

        if self.state == 'WAIT_EKF':
            d1_ekf_ok = (self.drone1_local_pos.timestamp > 0 and
                         self.drone1_local_pos.xy_valid and
                         self.drone1_local_pos.z_valid and
                         self.drone1_odom.timestamp > 0)

            now_sec = float(now.nanoseconds) * 1e-9
            dji_mocap_ok = self.optitrack_received and ((now_sec - self.last_optitrack_time) < 1.0)

            if d1_ekf_ok and dji_mocap_ok:
                # Calcolo posizione camera spawn GuarDrone in ENU
                q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2], self.drone1_odom.q[3], self.drone1_odom.q[0]]
                R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
                R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
                cam_offset = np.array([self.cam_offset_x, self.cam_offset_y, self.cam_offset_z])
                rotated_offset = R_flu2enu @ cam_offset

                cam_spawn_x = self.drone_pos_enu[0] + rotated_offset[0]
                cam_spawn_y = self.drone_pos_enu[1] + rotated_offset[1]

                # Riferimento di hovering iniziale rispetto alla posizione reale del DJI
                dx = float(cam_spawn_x - self.dji_pos_enu[0])
                dy = float(cam_spawn_y - self.dji_pos_enu[1])
                self.r_hover = math.sqrt(dx**2 + dy**2)
                self.beta_hover = math.atan2(dy, dx)
                self.current_cmd_pov = np.array([self.r_hover, self.beta_hover, 0.0, 0.0], dtype=float)

                psi_enu = Rotation.from_matrix(R_flu2enu).as_euler('xyz')[2]

                self.get_logger().info(
                    f"EKF e OptiTrack OK!\n"
                    f" - GuarDrone spawn ENU: [{self.drone_pos_enu[0]:.2f}, {self.drone_pos_enu[1]:.2f}, {self.drone_pos_enu[2]:.2f}], Yaw: {math.degrees(psi_enu):.1f}°\n"
                    f" - DJI OptiTrack ENU:  [{self.dji_pos_enu[0]:.2f}, {self.dji_pos_enu[1]:.2f}, {self.dji_pos_enu[2]:.2f}]\n"
                    f" - Target iniziale stimato: r={self.r_hover:.3f}m, beta={math.degrees(self.beta_hover):.1f}°"
                )
                self.state = 'WAIT_TAKEOFF'

            elif self.wait_ticks % 25 == 0:
                if not d1_ekf_ok:
                    self.get_logger().warn("Attesa convergenza EKF GuarDrone...")
                if not dji_mocap_ok:
                    self.get_logger().warn(f"Attesa segnale OptiTrack da {self.dji_topic}...")

        elif self.state == 'WAIT_TAKEOFF':
            if self.mpc_ready:
                if not self.wait_msg_printed:
                    self.get_logger().info(
                        "Planner di takeoff pronto. DJI rilevato.\n"
                        "-> Digita 'ok' (e premi invio) sul terminale GCS per autorizzare il decollo di GuarDrone."
                    )
                    self.wait_msg_printed = True

                if self.user_ok:
                    self.user_ok = False
                    self.get_logger().info("Inizio decollo. Invio target e passaggio ad ARM_OFFBOARD.")

                    cam_body_takeoff_z = float(self.takeoff_alt_1 - self.cam_offset_z)

                    q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2],
                               self.drone1_odom.q[3], self.drone1_odom.q[0]]
                    R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
                    R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
                    current_yaw_enu = float(Rotation.from_matrix(R_flu2enu).as_euler('xyz')[2])
                    yaw_quat = Rotation.from_euler('z', current_yaw_enu).as_quat()

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

                    msg_traj = Bool()
                    msg_traj.data = True
                    self.cam_traj_enabled_pub.publish(msg_traj)

                    log_start_msg = Bool()
                    log_start_msg.data = True
                    self.logging_start_pub.publish(log_start_msg)

                    self.state = 'ARM_OFFBOARD'

        elif self.state == 'ARM_OFFBOARD':
            d1_ready = (self.drone1_mode.flag_control_offboard_enabled and self.drone1_mode.flag_armed)
            if not d1_ready:
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                self.get_logger().info("GuarDrone Armato e in Offboard. Salita in corso...")
                self.state = 'TAKEOFF_MONITOR'

        elif self.state == 'TAKEOFF_MONITOR':
            peg_local_pos_z = -self.drone1_local_pos.z
            dist = abs(peg_local_pos_z - (self.takeoff_alt_1 - self.cam_offset_z - self.guardrone_start_z))
            d1_up = dist < 0.10

            # Aggiornamento continuo di r_hover e beta_hover in base alla posizione reale del DJI
            q_scipy = [self.drone1_odom.q[1], self.drone1_odom.q[2], self.drone1_odom.q[3], self.drone1_odom.q[0]]
            R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
            R_flu2enu = self.M_ned2enu @ R_frd2ned @ self.M_frd2flu
            cam_offset = np.array([self.cam_offset_x, self.cam_offset_y, self.cam_offset_z])
            rotated_offset = R_flu2enu @ cam_offset
            cam_x = self.drone_pos_enu[0] + rotated_offset[0]
            cam_y = self.drone_pos_enu[1] + rotated_offset[1]

            dx = float(cam_x - self.dji_pos_enu[0])
            dy = float(cam_y - self.dji_pos_enu[1])
            self.r_hover = math.sqrt(dx**2 + dy**2)
            self.beta_hover = math.atan2(dy, dx)

            if d1_up:
                if not self.switch_msg_printed:
                    self.get_logger().info(
                        f"GuarDrone in quota! Riferimento stimato rispetto al DJI: "
                        f"r={self.r_hover:.3f}m, beta={math.degrees(self.beta_hover):.1f}°.\n"
                        f"-> Digita 'ok' sul terminale GCS per attivare l'MPC."
                    )
                    self.switch_msg_printed = True

                if self.user_ok:
                    self.user_ok = False
                    # Spegne offboard trajectory planner
                    msg_traj = Bool()
                    msg_traj.data = False
                    self.cam_traj_enabled_pub.publish(msg_traj)

                    # Invia target PoV iniziale
                    self.current_cmd_pov = np.array([self.r_hover, self.beta_hover, 0.0, 0.0], dtype=float)
                    self.publish_pov_target(self.r_hover, self.beta_hover, 0.0, 0.0)

                    # Avvia MPC
                    msg_start = Bool()
                    msg_start.data = True
                    self.task_start_pub.publish(msg_start)

                    self.state = 'HOVERING'
                    self.get_logger().info("HOVERING AVVIATO. GuarDrone controllato tramite MPC su target DJI.")

        elif self.state == 'HOVERING':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in HOVERING: inizio fase di APPROACH...")
                self.state = 'APPROACH'
                self.msg_cnt = 0
                self.apply_phase_pov('APPROACH')
            elif self.msg_cnt == 0:
                self.get_logger().info('HOVERING in corso (MPC attivo su DJI). Digita "ok" per procedere ad APPROACH o "land" per atterrare.')
                self.msg_cnt += 1

        elif self.state == 'APPROACH':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in APPROACH: inizio fase di INTERACTION (PoV decentrato)...")
                self.state = 'INTERACTION'
                self.msg_cnt = 0
                self.apply_phase_pov('INTERACTION')
            elif self.msg_cnt == 0:
                self.get_logger().info('APPROACH in corso: pilota il DJI nella posizione desiderata. Digita "ok" per passare ad INTERACTION (o "land").')
                self.msg_cnt += 1

        elif self.state == 'INTERACTION':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in INTERACTION: inizio fase di DETACHMENT...")
                self.state = 'DETACHMENT'
                self.msg_cnt = 0
                self.apply_phase_pov('DETACHMENT')
            elif self.msg_cnt == 0:
                self.get_logger().info('INTERACTION in corso (PoV decentrato r=1.5m). L\'operatore pilota il DJI liberamente. Digita "ok" per DETACHMENT (o "land").')
                self.msg_cnt += 1

        elif self.state == 'DETACHMENT':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in DETACHMENT: inizio fase di RETURN_HOME...")
                self.state = 'RETURN_HOME'
                self.msg_cnt = 0
                self.apply_phase_pov('RETURN_HOME')
            elif self.msg_cnt == 0:
                self.get_logger().info('DETACHMENT completato (PoV centrato r=1.5m). Digita "ok" per RETURN_HOME (o "land").')
                self.msg_cnt += 1

        elif self.state == 'RETURN_HOME':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in RETURN_HOME: inizio fase di LANDING...")
                self.state = 'LANDING'
                self.landing_started = False
                self.msg_cnt = 0
            elif self.msg_cnt == 0:
                self.get_logger().info('RETURN_HOME in corso (PoV allargato r=2.0m). Digita "ok" per avviare il LANDING...')
                self.msg_cnt += 1

        elif self.state == 'LANDING':
            self._publish_state_if_changed()

            # Stop MPC Task e Trajectory Planner
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)

            # Comando di Atterraggio nativo PX4
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Comando di Atterraggio nativo PX4 inviato a GuarDrone. Stop MPC e attesa disarmo...")
            self.state = 'DISARM_WAIT'

        elif self.state == 'DISARM_WAIT':
            if not self.drone1_mode.flag_armed:
                self.get_logger().info("GuarDrone atterrato e DISARMATO. Missione completata con successo!")
                self.state = 'MISSION_COMPLETE'

        elif self.state == 'MISSION_COMPLETE':
            pass

        elif self.state == 'EMERGENCY':
            msg_stop = Bool()
            msg_stop.data = False
            self.task_start_pub.publish(msg_stop)
            self.cam_traj_enabled_pub.publish(msg_stop)
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main(args=None):
    rclpy.init(args=args)
    node = DJIPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
