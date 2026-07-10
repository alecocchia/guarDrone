#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleControlMode, VehicleCommand
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np

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
        # 1. Odometria fittizia per il peg
        self.odom_pub = self.create_publisher(VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', px4_qos_profile)
        # 2. Controllo task e logging
        self.task_start_pub = self.create_publisher(Bool, '/mpc_task/start', qos_latched)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)
        # 3. Offboard Trajectory Planner e POV
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)
        # 4. PX4 Commands (Arm, Offboard)
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos_profile)
        
        # --- Subscribers ---
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, px4_qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, px4_qos_profile)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)
        
        # --- Parametri di volo ---
        self.takeoff_alt_1 = 2.0  # Quota di decollo (ENU)
        
        # Variabili di stato interne
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone1_mode = VehicleControlMode()
        self.mpc_ready = False
        
        self.state = 'WAIT_EKF'
        self.wait_ticks = 0
        
        # Loop principale a 50Hz (necessario per l'odometria del peg)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Fake Supervisor & Peg Publisher avviato. Stato: WAIT_EKF")

    # --- Callbacks ---
    def pos1_cb(self, msg): self.drone1_local_pos = msg

    def mode1_cb(self, msg): self.drone1_mode = msg

    def mpc_ready_cb(self, msg):
        if msg.data and not self.mpc_ready:
            self.mpc_ready = True
            self.get_logger().info("Segnale MPC Pronto ricevuto!")

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
        
        # =========================================================================
        # 1) PUBBLICAZIONE COSTANTE ODOMETRIA PEG (50Hz)
        # =========================================================================
        odom_msg = VehicleOdometry()
        odom_msg.timestamp = int(now.nanoseconds / 1000)
        
        # Posizione del peg (coordinate NED).
        # Impostiamo il peg 2m "davanti" (North) e a 2m di quota (Up = -2.0 NED)
        # Cosi', se il drone sta all'origine in XY, il peg_drone è a [2, 0, 2] in ENU.
        odom_msg.position = [2.0, 0.0, -float(self.takeoff_alt_1)]
        odom_msg.q = [1.0, 0.0, 0.0, 0.0] 
        odom_msg.velocity = [0.0, 0.0, 0.0]
        odom_msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(odom_msg)
        
        # =========================================================================
        # 2) PUBBLICAZIONE POV TARGET (50Hz)
        # =========================================================================
        # Se il peg è a X=2 e il drone è a X=0, il drone è "dietro" al peg di 2m.
        # r_cyl = 2.0, beta = Pi rad. Z rel = 0.
        # Con questo target, il drone l'MPC cercherà di mantenere la
        # posizione di hovering a [0, 0, 2] (cioè dove ha fatto il takeoff), 
        # senza avere problemi di singolarità atan2 per r=0.
        pov_msg = Float64MultiArray()
        pov_msg.data = [2.0, math.pi, 0.0, 0.0] 
        self.pov_pub.publish(pov_msg)
        
        # =========================================================================
        # 3) MACCHINA A STATI DEL SUPERVISOR (~10Hz)
        # =========================================================================
        self.wait_ticks += 1
        if self.wait_ticks % 5 != 0:
            return
            
        if self.state == 'WAIT_EKF':
            # Controlla la convergenza di PX4 per il drone reale
            d1_ekf_ok = (self.drone1_local_pos.timestamp > 0 and 
                         self.drone1_local_pos.xy_valid and 
                         self.drone1_local_pos.z_valid)
            
            if d1_ekf_ok:
                self.get_logger().info("EKF Convergente. Passo a WAIT_START.")
                self.state = 'WAIT_START'
                
        elif self.state == 'WAIT_START':
            if self.mpc_ready:
                self.get_logger().info("Inizio missione automatica. Invio target di takeoff e passo ad ARM_OFFBOARD.")
                
                # Invia target di decollo
                cam_pose = PoseStamped()
                cam_pose.header.frame_id = 'world'
                cam_pose.pose.position.x = 0.0
                cam_pose.pose.position.y = 0.0
                cam_pose.pose.position.z = float(self.takeoff_alt_1)
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
            # drone1_local_pos.z è NED (negativo verso l'alto)
            d1_up = abs(-self.drone1_local_pos.z - self.takeoff_alt_1) < 0.2
            
            if d1_up:
                self.get_logger().info("Drone in quota (2m) raggiunto! Switch da Trajectory Planner a MPC.")
                
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
