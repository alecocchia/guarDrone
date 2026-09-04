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

        self.guardrone_start_yaw = 0.0
        
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
        if msg.data.strip().lower() == 'ok':
            self.user_ok = True
            self.get_logger().info("Comando OK ricevuto dal terminale GCS!")

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

    def timer_callback(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        
        # =========================================================================
        # PUBBLICAZIONE COSTANTE ODOMETRIA PEG (50Hz)
        # =========================================================================
        peg_odom_msg = VehicleOdometry()
        peg_odom_msg.timestamp = int(now.nanoseconds / 1000)
        
        # Posizione del peg (coordinate NED).
        # L'odometria PX4 è locale rispetto al punto di spawn.
        # Il peg deve essere in hovering esattamente sopra il suo punto di spawn,
        # quindi N=0, E=0. La quota locale (D) è negativo (takeoff_alt_1 - peg_start_z).
        local_z = float(self.takeoff_alt_1 - self.peg_start_z)
        peg_odom_msg.position = [0.0, 0.0, -local_z]
        peg_odom_msg.q = [1.0, 0.0, 0.0, 0.0]
        peg_odom_msg.velocity = [0.0, 0.0, 0.0]
        peg_odom_msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(peg_odom_msg)

        # =========================================================================
        # PUBBLICAZIONE STATO ATTUALE PEG IN ENU (50Hz)
        # Nella sim il peg è fermo in hovering: posizione fissa, vel/yaw/yaw_rate = 0
        # Nella configurazione reale questi topic sono pubblicati da offboard_admittance_planner
        # =========================================================================
        # Posizione ENU del peg: M_ned2enu @ [0,0,-local_z] + [peg_start_x, peg_start_y, peg_start_z]
        #   = [0, 0, local_z] + [peg_start_x, peg_start_y, peg_start_z]
        #   = [peg_start_x, peg_start_y, takeoff_alt_1]
        peg_pose_msg = PoseStamped()
        peg_pose_msg.header.stamp = stamp
        peg_pose_msg.header.frame_id = 'world'
        peg_pose_msg.pose.position.x = float(self.peg_start_x)
        peg_pose_msg.pose.position.y = float(self.peg_start_y)
        peg_pose_msg.pose.position.z = float(self.takeoff_alt_1)
        peg_pose_msg.pose.orientation.w = 1.0  # identità (yaw=0)
        self.peg_actual_pose_pub.publish(peg_pose_msg)

        peg_vel_msg = TwistStamped()
        peg_vel_msg.header.stamp = stamp
        peg_vel_msg.header.frame_id = 'world'
        # velocità = 0 (hovering)
        self.peg_actual_vel_pub.publish(peg_vel_msg)

        yaw_msg = Float64()
        yaw_msg.data = 0.0
        self.peg_actual_yaw_pub.publish(yaw_msg)

        yaw_rate_msg = Float64()
        yaw_rate_msg.data = 0.0
        self.peg_actual_yaw_rate_pub.publish(yaw_rate_msg)

        # =========================================================================
        # 2) PUBBLICAZIONE POV TARGET (50Hz)
        # =========================================================================
        pov_msg = Float64MultiArray()
        pov_msg.data = [self.r_hover, self.beta_hover, 0.0, 0.0] 
        self.pov_pub.publish(pov_msg)
        
        # =========================================================================
        # 3) MACCHINA A STATI DEL SUPERVISOR (~10Hz)
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

                    cam_pose = PoseStamped()
                    cam_pose.header.frame_id = 'world'
                    cam_pose.pose.position.x = float(self.drone_pos_enu[0])
                    cam_pose.pose.position.y = float(self.drone_pos_enu[1])
                    cam_pose.pose.position.z = cam_body_takeoff_z
                    cam_pose.pose.orientation.x = float(yaw_quat[0])
                    cam_pose.pose.orientation.y = float(yaw_quat[1])
                    cam_pose.pose.orientation.z = float(yaw_quat[2])
                    cam_pose.pose.orientation.w = float(yaw_quat[3])
                    self.get_logger().info(f"Target decollo: z={cam_body_takeoff_z:.3f}m, yaw={math.degrees(current_yaw_enu):.1f}°")
                    self.cam_target_pub.publish(cam_pose)
                    
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
            d1_up = dist < 2
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
            
                    # 2. Avvia MPC
                    msg_start = Bool()
                    msg_start.data = True
                    self.task_start_pub.publish(msg_start)
                    
                    self.state = 'MISSION'
                    self.get_logger().info("MISSIONE AVVIATA. Hovering mantenuto tramite MPC.")
                
        elif self.state == 'MISSION':
            # Il loop principale a 50Hz continua a mandare pov_target e odometria peg
            pass

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
