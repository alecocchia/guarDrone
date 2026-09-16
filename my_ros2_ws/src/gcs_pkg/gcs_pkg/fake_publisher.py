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
        self.task_start_pub = self.create_publisher(Bool, '/mpc_task/start', qos_latched)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)
        #  Offboard Trajectory Planner e POV
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)
        # PX4 Commands (Arm, Offboard)
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos_profile)
        # Stato attuale drone peg (ENU) — in sim: posizione fissa di hovering
        # In real: pubblicato da offboard_admittance_planner
        self.peg_actual_pose_pub     = self.create_publisher(PoseStamped,  '/peg_actual_pose',     10)
        self.peg_actual_vel_pub      = self.create_publisher(TwistStamped, '/peg_actual_velocity', 10)
        self.peg_actual_yaw_pub      = self.create_publisher(Float64,      '/peg_actual_yaw',      10)
        self.peg_actual_yaw_rate_pub = self.create_publisher(Float64,      '/peg_actual_yaw_rate', 10)
        
        # --- Subscribers ---
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, px4_qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom1_cb, px4_qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, px4_qos_profile)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)
        self.create_subscription(String, '/keyboard_input', self.keyboard_cb, 10)
        
        # --- Parametri di volo ---
        self.declare_parameter('takeoff_alt_1', 4.52+3.0)
        self.declare_parameter('guardrone_start_x', 0.0)
        self.declare_parameter('guardrone_start_y', 0.0)
        self.declare_parameter('guardrone_start_z', 4.52)
        self.declare_parameter('peg_start_x', 0.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 4.52)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.0)

        self.cam_offset_x = self.get_parameter('cam_offset_x').value
        self.cam_offset_y = self.get_parameter('cam_offset_y').value
        self.cam_offset_z = self.get_parameter('cam_offset_z').value
        self.takeoff_alt_1 = self.get_parameter('takeoff_alt_1').value
        self.guardrone_start_x = self.get_parameter('guardrone_start_x').value
        self.guardrone_start_y = self.get_parameter('guardrone_start_y').value
        self.guardrone_start_z = self.get_parameter('guardrone_start_z').value
        self.peg_start_x = self.get_parameter('peg_start_x').value
        self.peg_start_y = self.get_parameter('peg_start_y').value
        self.peg_start_z = self.get_parameter('peg_start_z').value

        
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

        # Variabili di stato interne
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone1_odom = VehicleOdometry()
        self.drone1_mode = VehicleControlMode()
        self.mpc_ready = False
        self.user_ok = False
        self.wait_msg_printed = False
        self.switch_msg_printed = False

        # Posizione e velocita dinamica del fake peg (inizialmente sopra il suo spawn point a quota takeoff_alt_1)
        self.fake_peg_pos = np.array([self.peg_start_x, self.peg_start_y, self.takeoff_alt_1], dtype=float)
        self.fake_peg_vel = np.zeros(3, dtype=float)
        # Traiettoria trapezoidale continua (utils_pkg.planner)
        self.peg_traj_p = None
        self.peg_traj_v = None
        self.peg_traj_idx = 0
        self.detachment_started = False
        self.return_home_started = False
        self.landing_started = False
        self.msg_cnt = 0
        
        self.state = 'WAIT_EKF'
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

    def keyboard_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'ok':
            self.user_ok = True
            self.get_logger().info('Comando OK ricevuto dal terminale GCS!')
        elif cmd == 'land':
            self.get_logger().warn('Comando LAND ricevuto dal terminale GCS! Avvio sequenza di atterraggio.')
            self.state = 'LANDING'
            self.landing_started = False
            self.user_ok = False
        elif cmd == 'stop':
            self.get_logger().error("Comando STOP ricevuto! Atterraggio d'emergenza!")
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'EMERGENCY'

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
        peg_odom_msg.q = [1.0, 0.0, 0.0, 0.0]
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
        peg_pose_msg.pose.orientation.w = 1.0  # identità (yaw=0)
        self.peg_actual_pose_pub.publish(peg_pose_msg)

        peg_vel_msg = TwistStamped()
        peg_vel_msg.header.stamp = stamp
        peg_vel_msg.header.frame_id = 'world'
        peg_vel_msg.twist.linear.x = float(self.fake_peg_vel[0])
        peg_vel_msg.twist.linear.y = float(self.fake_peg_vel[1])
        peg_vel_msg.twist.linear.z = float(self.fake_peg_vel[2])
        self.peg_actual_vel_pub.publish(peg_vel_msg)

        yaw_msg = Float64()
        yaw_msg.data = 0.0
        self.peg_actual_yaw_pub.publish(yaw_msg)

        yaw_rate_msg = Float64()
        yaw_rate_msg.data = 0.0
        self.peg_actual_yaw_rate_pub.publish(yaw_rate_msg)

        # =========================================================================
        # 2) MACCHINA A STATI DEL SUPERVISOR (~10Hz)
        # =========================================================================
        self.wait_ticks += 1
        if self.wait_ticks % 5 != 0:
            return
            
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
                self.state = 'WAIT_START'
                
        elif self.state == 'WAIT_START':
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
            d1_up = dist < 0.03
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
                    self.get_logger().info(f"Drone in quota! Riferimento attuale: r={self.r_hover:.3f}, beta={math.degrees(self.beta_hover):.1f}°. Switch a MPC pronto. Dare ok da tastiera")
                    self.switch_msg_printed = True
                    
                if self.user_ok:
                    self.user_ok = False # Consuma il comando
                    # 1. Spegne offboard trajectory planner
                    msg_traj = Bool()
                    msg_traj.data = False
                    self.cam_traj_enabled_pub.publish(msg_traj)
            
                    # 2. Invia target PoV iniziale per l'MPC (inviato una sola volta allo switch)
                    pov_msg = Float64MultiArray()
                    pov_msg.data = [self.r_hover, self.beta_hover, 0.0, 0.0]
                    self.pov_pub.publish(pov_msg)

                    # 3. Avvia MPC
                    msg_start = Bool()
                    msg_start.data = True
                    self.task_start_pub.publish(msg_start)
                    
                    self.state = 'MISSION'
                    self.get_logger().info("MISSIONE AVVIATA. Hovering mantenuto tramite MPC.")
                
        elif self.state == 'MISSION':
            if self.user_ok:
                self.user_ok = False
                self.get_logger().info("Comando 'ok' ricevuto in MISSION: inizio fase di DETACHMENT...")
                self.state = 'DETACHMENT'
                self.detachment_started = False
                self.msg_cnt = 0
            elif self.msg_cnt == 0:
                self.get_logger().info('MISSIONE in corso con MPC. Digita "ok" per procedere al distacco (DETACHMENT) o "land" per atterrare.')
                self.msg_cnt += 1

        elif self.state == 'DETACHMENT':
            if not self.detachment_started:
                self.detachment_started = True
                detach_target = np.array([self.peg_start_x, self.peg_start_y + 1.5, self.takeoff_alt_1])
                dur = self.plan_peg_trajectory(detach_target, v_max=0.2, a_max=0.2)
                self.get_logger().info(f"Distacco avviato (trapezoidale, {dur:.1f}s): peg verso {detach_target}. GuarDrone segue in MPC...")

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
                home_target = np.array([self.peg_start_x, self.peg_start_y, self.takeoff_alt_1])
                dur = self.plan_peg_trajectory(home_target, v_max=0.3, a_max=0.2)
                self.get_logger().info(f"Ritorno alla base avviato (trapezoidale, {dur:.1f}s): peg verso {home_target}. GuarDrone segue in MPC...")

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
            if not self.landing_started:
                self.landing_started = True
                target_ground = np.array([self.peg_start_x, self.peg_start_y, self.peg_start_z])
                dur = self.plan_peg_trajectory(target_ground, v_max=0.2, a_max=0.2)
                self.get_logger().info(f"LANDING avviato (trapezoidale, {dur:.1f}s): il peg scende verso il suolo, GuarDrone segue in MPC.")

            # Controlla la quota reale di GuarDrone
            # drone1_local_pos.z e NED (negativo verso l'alto). La quota relativa allo spawn e -z
            guardrone_alt_rel = -self.drone1_local_pos.z
            if guardrone_alt_rel < 0.7:
                # GuarDrone e vicino al suolo: spegni l'MPC e dai comando di LAND PX4 per il touchdown finale
                self.get_logger().info(f"GuarDrone vicino al suolo (quota rel = {guardrone_alt_rel:.2f}m). Stop MPC e invio VEHICLE_CMD_NAV_LAND!")
                msg_stop = Bool()
                msg_stop.data = False
                self.task_start_pub.publish(msg_stop)

                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'DISARM_WAIT'

        elif self.state == 'DISARM_WAIT':
            if not self.drone1_mode.flag_armed:
                self.get_logger().info("GuarDrone atterrato e DISARMATO. Missione completata con successo!")
                self.state = 'MISSION_COMPLETE'

        elif self.state == 'MISSION_COMPLETE':
            pass

        elif self.state == 'EMERGENCY':
            # Keep sending land
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
