#!/usr/bin/env python3
# Il supervisore deve: 
# 1. Attendere che i droni siano pronti per il takeoff, armarli e farli decollare ad un'altezza prestabilita (es. 2 m)
# 2. Quando i droni sono in quota, passare in offboard
# 3. Iniziare il task di inseguimento
# 4. Fare landing quando hanno finito
# 5. Gestire emergenze
# Inoltre sarà lui a gestire tutte le richieste di offboard ed i check di sicurezza

import rclpy
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleCommand, VehicleLocalPosition
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_latched = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )

        # Publishers for VehicleCommand (Drone 1 and Drone 2)
        self.cmd_pub_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.cmd_pub_2 = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)

        # Publisher for Task Start
        self.task_start_pub = self.create_publisher(Bool, '/mpc_task/start', qos_latched)
        # Publisher per segnale di avvio logging (al momento dell'arming+offboard)
        self.logging_start_pub = self.create_publisher(Bool, '/logging/start', qos_latched)

        # Publisher per i nodi trajectory_planner
        self.cam_target_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)
        self.peg_target_pub = self.create_publisher(PoseStamped, '/peg_target_pose', 10)
        self.cam_traj_enabled_pub = self.create_publisher(Bool, '/camera_traj_enabled', 10)
        self.peg_traj_enabled_pub = self.create_publisher(Bool, '/peg_traj_enabled', 10)

        # State Variables
        self.drone1_local_pos = VehicleLocalPosition()
        self.drone2_local_pos = VehicleLocalPosition()
        self.drone1_mode = VehicleControlMode()
        self.drone2_mode = VehicleControlMode()

        # Subscribers Drone 1
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos1_cb, qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode1_cb, qos_profile)

        # Subscribers Drone 2
        self.create_subscription(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position', self.pos2_cb, qos_profile)
        self.create_subscription(VehicleControlMode, '/px4_1/fmu/out/vehicle_control_mode', self.mode2_cb, qos_profile)

        # Parameters
        self.declare_parameter('takeoff_alt_1', 4.52+3.0) # Camera takeoff in ENU
        self.declare_parameter('takeoff_alt_2', 4.52+3.0) # Peg takeoff in ENU
        self.declare_parameter('cam_start_x', 0.0)
        self.declare_parameter('cam_start_y', 0.0)
        self.declare_parameter('cam_start_z', 4.52)
        self.declare_parameter('peg_start_x', 3.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 4.52)

        self.takeoff_alt_1 = self.get_parameter('takeoff_alt_1').value
        self.takeoff_alt_2 = self.get_parameter('takeoff_alt_2').value
        self.cam_start_x = self.get_parameter('cam_start_x').value
        self.cam_start_y = self.get_parameter('cam_start_y').value
        self.cam_start_z = self.get_parameter('cam_start_z').value
        self.peg_start_x = self.get_parameter('peg_start_x').value
        self.peg_start_y = self.get_parameter('peg_start_y').value
        self.peg_start_z = self.get_parameter('peg_start_z').value

        self.state = 'WAIT_EKF'
        self.task_started = False
        self.mission_start_received = False
        self.task_goal_pose_received = False
        self.mpc_ready = False
        
        # Subscriber for manual mission start
        self.create_subscription(Bool, '/mission/start', self.mission_start_cb, qos_latched)
        self.create_subscription(Bool, '/drone_planner_ready', self.mpc_ready_cb, qos_latched)

        self.get_logger().info("Supervisor Node Avviato. Stato: WAIT_EKF")

        # Main loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def mission_start_cb(self, msg):
        if msg.data and not self.mission_start_received:
            self.mission_start_received = True
            self.get_logger().info("Ricevuto segnale MANUALE su /mission/start!")

    def mpc_ready_cb(self, msg):
        if msg.data and not self.mpc_ready:
            self.mpc_ready = True
            self.get_logger().info("Ricevuto segnale MPC Pronto!")

    def pos1_cb(self, msg): self.drone1_local_pos = msg
    def pos2_cb(self, msg): self.drone2_local_pos = msg
    def mode1_cb(self, msg): self.drone1_mode = msg
    def mode2_cb(self, msg): self.drone2_mode = msg

    def publish_command(self, publisher, target_sys, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        publisher.publish(msg)

    def loop(self):
        if self.state == 'WAIT_EKF':
            # Check if both drones have full EKF convergence
            d1_ekf_ok = (self.drone1_local_pos.timestamp > 0 and 
                         self.drone1_local_pos.xy_valid and 
                         self.drone1_local_pos.z_valid)
            d2_ekf_ok = (self.drone2_local_pos.timestamp > 0 and 
                         self.drone2_local_pos.xy_valid and 
                         self.drone2_local_pos.z_valid)

            if d1_ekf_ok and d2_ekf_ok:
                self.get_logger().info("EKF Convergenti per entrambi. Passo a WAIT_START")
                self.state = 'WAIT_START'
                self.mission_start_received = True
            else:
                # Logghiamo cosa manca ogni 2 secondi
                if not hasattr(self, 'last_log_time'):
                    self.last_log_time = self.get_clock().now()
                elif (self.get_clock().now() - self.last_log_time).nanoseconds > 2e9:
                    self.get_logger().info(f"In attesa EKF... D1_EKF: {d1_ekf_ok}, D2_EKF: {d2_ekf_ok}")
                    self.last_log_time = self.get_clock().now()

        elif self.state == 'WAIT_START':
            if self.mission_start_received and self.mpc_ready:
                self.get_logger().info("Segnale di avvio e MPC Pronto ricevuti. Mando comandi di decollo e Passo a ARM_OFFBOARD")
                
                # Invia comando di decollo al camera drone
                cam_pose = PoseStamped()
                cam_pose.header.frame_id = 'world'
                cam_pose.pose.position.x = float(self.cam_start_x)
                cam_pose.pose.position.y = float(self.cam_start_y)
                cam_pose.pose.position.z = float(self.takeoff_alt_1) # ENU
                self.get_logger().info(f"Posizione di decollo al camera drone: {cam_pose.pose.position.x}, {cam_pose.pose.position.y}, {cam_pose.pose.position.z}")
                self.cam_target_pub.publish(cam_pose)

                # Invia comando di decollo al peg drone
                peg_pose = PoseStamped()
                peg_pose.header.frame_id = 'world'
                peg_pose.pose.position.x = float(self.peg_start_x)
                peg_pose.pose.position.y = float(self.peg_start_y)
                peg_pose.pose.position.z = float(self.takeoff_alt_2) # ENU
                self.get_logger().info(f"Posizione di decollo al peg drone: {peg_pose.pose.position.x}, {peg_pose.pose.position.y}, {peg_pose.pose.position.z}")
                self.peg_target_pub.publish(peg_pose)
                
                # Assicurati che i planner siano accesi
                msg = Bool()
                msg.data = True
                self.cam_traj_enabled_pub.publish(msg)
                self.peg_traj_enabled_pub.publish(msg)

                self.state = 'ARM_OFFBOARD'
                # Segnala al logger di iniziare a registrare
                log_start_msg = Bool()
                log_start_msg.data = True
                self.logging_start_pub.publish(log_start_msg)
                self.get_logger().info("Segnale /logging/start pubblicato.")

        elif self.state == 'ARM_OFFBOARD':
            self.get_logger().info(f"Stato droni -> D1(offboard:{self.drone1_mode.flag_control_offboard_enabled}, arm:{self.drone1_mode.flag_armed}) | D2(offboard:{self.drone2_mode.flag_control_offboard_enabled}, arm:{self.drone2_mode.flag_armed})")
            d1_ready = (self.drone1_mode.flag_control_offboard_enabled and self.drone1_mode.flag_armed)
            d2_ready = (self.drone2_mode.flag_control_offboard_enabled and self.drone2_mode.flag_armed)

            if not d1_ready:
                self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            if not d2_ready:
                self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

            if d1_ready and d2_ready:
                self.get_logger().info("Entrambi i droni sono Armati e in Offboard. Monitoraggio Decollo...")
                self.state = 'TAKEOFF_MONITOR'

        elif self.state == 'TAKEOFF_MONITOR':
            # Check altitudes (z is negative upwards)
            d1_up = abs(-self.drone1_local_pos.z - (self.takeoff_alt_1 - self.cam_start_z)) < 0.1
            d2_up = abs(-self.drone2_local_pos.z - (self.takeoff_alt_2 - self.peg_start_z)) < 0.1
            self.get_logger().info(f"d1_up: {self.drone1_local_pos.z}")
            self.get_logger().info(f"d2_up: {self.drone2_local_pos.z}")

            if d1_up and d2_up:
                self.get_logger().info("Droni in quota! Disabilito traj planner camera e invio segnale di START all'MPC.")
                
                # Spegne offboard_trajectory_planner del camera drone
                msg_traj = Bool()
                msg_traj.data = False
                self.cam_traj_enabled_pub.publish(msg_traj)

                # Accende MPC
                msg_start = Bool()
                msg_start.data = True
                self.task_start_pub.publish(msg_start)
                
                self.task_started = True
                self.state = 'MISSION'

        elif self.state == 'MISSION':
            # Emergency monitoring
            d1_fail = not self.drone1_mode.flag_armed or not self.drone1_mode.flag_control_offboard_enabled
            d2_fail = not self.drone2_mode.flag_armed or not self.drone2_mode.flag_control_offboard_enabled

            if d1_fail and self.drone2_mode.flag_armed:
                self.get_logger().error("EMERGENZA: Drone 1 ha fallito. Atterraggio Drone 2!")
                self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'EMERGENCY'
            elif d2_fail and self.drone1_mode.flag_armed:
                self.get_logger().error("EMERGENZA: Drone 2 ha fallito. Atterraggio Drone 1!")
                self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'EMERGENCY'
            elif not self.task_goal_pose_received and self.task_started:
                msg_peg_target = PoseStamped()
                msg_peg_target.pose.position.x = float(-1.0)
                msg_peg_target.pose.position.y = float(-56.2)   #-55.91 for contact with wall;-14 for testing teleoperation with cube, -1.91
                msg_peg_target.pose.position.z = float(10.0)
                
                # Impostazione del target di yaw desiderato (in radianti)
                # Esempio: rotazione di 90 gradi rispetto all'asse Z
                target_yaw = -math.pi/2 +0.2 
                
                # Utilizziamo scipy.spatial.transform.Rotation per convertire Roll-Pitch-Yaw in Quaternioni
                rot = R.from_euler('xyz', [0.0, 0.0, target_yaw])
                quat = rot.as_quat() # Restituisce un array [x, y, z, w]
                
                msg_peg_target.pose.orientation.x = float(quat[0])
                msg_peg_target.pose.orientation.y = float(quat[1])
                msg_peg_target.pose.orientation.z = float(quat[2])
                msg_peg_target.pose.orientation.w = float(quat[3])
                
                self.peg_target_pub.publish(msg_peg_target)
                self.task_goal_pose_received = True

        elif self.state == 'EMERGENCY':
            # Keep sending land just in case
            self.get_logger().error("EMERGENZA: Atterraggio forzato per entrambi i droni")
            self.publish_command(self.cmd_pub_1, 1, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_command(self.cmd_pub_2, 2, VehicleCommand.VEHICLE_CMD_NAV_LAND)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()