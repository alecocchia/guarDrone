#!/usr/bin/env python3
"""
peg_planner_node.py — Peg-Drone Planner con controllo PX4

Genera una traiettoria per il drone che trasporta il peg e invia
TrajectorySetpoint a PX4 in modalità Offboard (position mode).
Riceve l'odometria dal flight controller e la ripubblica come /peg_odom
per il nodo MPC del camera-drone.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
import numpy as np
from drone_mpc_pkg.common import RPY_to_quat, quat_to_RPY
from drone_mpc_pkg.planner import generate_trapezoidal_trajectory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation

# --- PX4 Messages ---
from px4_msgs.msg import (
    VehicleOdometry, TrajectorySetpoint,
    OffboardControlMode, VehicleCommand, VehicleControlMode
)


class PegPlannerNode(Node):
    def __init__(self):
        super().__init__('peg_planner_node')

        self.t0 = 0
        self.Tf = 20.0
        self.ts = 0.005

        # --- Dichiarazione parametri ---
        self.declare_parameter('peg_start_x', 2.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 0.5)
        self.declare_parameter('peg_start_roll',0.0)
        self.declare_parameter('peg_start_pitch',0.0)
        self.declare_parameter('peg_start_yaw',0.0)
        self.declare_parameter('px4_ns', 'px4_peg')

        # --- Lettura dei valori da launchfile ---
        peg_x = self.get_parameter('peg_start_x').value
        peg_y = self.get_parameter('peg_start_y').value
        peg_z = self.get_parameter('peg_start_z').value
        peg_roll = self.get_parameter('peg_start_roll').value
        peg_pitch = self.get_parameter('peg_start_pitch').value
        peg_yaw = self.get_parameter('peg_start_yaw').value
        self.px4_ns = self.get_parameter('px4_ns').value

        self.p_obj_in = np.array([peg_x, peg_y, peg_z])
        self.rot_obj_in = np.array([peg_roll, peg_pitch, peg_yaw])
        
        self.get_logger().info(f"Peg posizionato in: {self.p_obj_in}. Inizio generazione traiettoria 2-fasi.")

        # --- Traiettoria in due fasi (in frame ENU) ---
        Tf1 = 10.0  # Fine fase decollo (10s)
        Tf2 = 15.0  # Fine fase inserimento (totale 25s)

        # Fase 1: Takeoff verticale a 10 metri
        p_takeoff = np.array([peg_x, peg_y, 10.0])
        ref_takeoff = np.concatenate([p_takeoff, self.rot_obj_in])
        
        ref_start = np.concatenate([self.p_obj_in, self.rot_obj_in])
        
        time1, p1, rpy1 = generate_trapezoidal_trajectory(
            ref_start, ref_takeoff, 0.0, Tf1, self.ts,
            v_max=1.5, a_max=0.5
        )
        
        # Fase 2: Inserimento nel buco (5, 0, 5)
        p_hole = np.array([5.0, 0.0, 5.0])
        ref_hole = np.concatenate([p_hole, self.rot_obj_in])
        
        time2, p2, rpy2 = generate_trapezoidal_trajectory(
            ref_takeoff, ref_hole, Tf1, Tf2, self.ts,
            v_max=1.0, a_max=0.3
        )
        
        # Concatenazione delle due fasi
        self.traj_time = np.concatenate([time1, time2[1:]])
        self.p_obj = np.concatenate([p1, p2[1:]])
        self.rpy_obj = np.concatenate([rpy1, rpy2[1:]])
        
        self.get_logger().info(f"Traiettoria pronta: Decollo -> 10m, Inserimento -> (5,0,5).")

        # --- Matrici di conversione ENU <-> NED ---
        # NED_x = ENU_y (North = North)
        # NED_y = ENU_x (East = East)  
        # NED_z = -ENU_z (Down = -Up)
        self.M_ned2enu = np.array([[0.0, 1.0, 0.0],
                                   [1.0, 0.0, 0.0],
                                   [0.0, 0.0, -1.0]])
        # M_enu2ned è la stessa matrice (è la propria inversa)
        self.M_enu2ned = self.M_ned2enu.copy()

        # --- Stato PX4 ---
        self.is_armed = False
        self.is_offboard = False
        self.first_odom_received = False
        self.startup_counter = 0
        
        # --- QoS Profiles ---
        qos_latched = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- Publishers PX4 (namespaced) ---
        ns = self.px4_ns
        self.traj_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', 1)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'/{ns}/fmu/in/offboard_control_mode', 1)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, f'/{ns}/fmu/in/vehicle_command', 1)

        # --- Subscribers PX4 ---
        self.odom_sub = self.create_subscription(
            VehicleOdometry, f'/{ns}/fmu/out/vehicle_odometry',
            self.px4_odom_callback, px4_qos)
        self.control_mode_sub = self.create_subscription(
            VehicleControlMode, f'/{ns}/fmu/out/vehicle_control_mode',
            self.control_mode_callback, px4_qos)

        # --- Publishers ROS (per il resto del sistema) ---
        self.odom_pub = self.create_publisher(Odometry, '/peg_odom', qos_latched)
        self.path_finished_pub = self.create_publisher(Bool, '/peg_path_finished', qos_latched)
        self.pose_pub = self.create_publisher(PoseStamped, '/peg_pose', 1)

        # --- Subscriber segnale ready dal camera drone planner ---
        self.ready_subscription = self.create_subscription(
            Bool, '/drone_planner_ready',
            self.controller_ready_callback,
            qos_latched)
        
        self.current_index = 0
        self.is_ready = False

        # Stato odometria corrente (ENU)
        self.current_position_enu = np.zeros(3)
        self.current_vel_enu = np.zeros(3)
        self.current_ang_vel = np.zeros(3)
        self.current_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Timer di pubblicazione
        self.timer = self.create_timer(self.ts, self.publish_next_pose)
        
        self.get_logger().info(f"Peg planner avviato con controllo PX4 (namespace: /{ns}). "
                               f"In attesa dell'odometria PX4...")

    # ==================== Callbacks ====================

    def control_mode_callback(self, msg: VehicleControlMode):
        self.is_armed = msg.flag_armed
        self.is_offboard = msg.flag_control_offboard_enabled

    def px4_odom_callback(self, msg: VehicleOdometry):
        """Riceve odometria PX4 (NED), converte in ENU e ripubblica come /peg_odom."""
        # Quaternione PX4: R_frd2ned → SciPy [x,y,z,w]
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()

        M_frd2flu = np.array([[1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0]])

        # R_flu2enu: body→world
        R_flu2enu = self.M_ned2enu @ R_frd2ned @ M_frd2flu
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        q_flu2enu = rot_flu2enu.as_quat()  # [x,y,z,w] scipy

        # Posizione NED → ENU
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        # Sommiamo lo spawn offset per riportare la posizione nel frame world globale
        self.current_position_enu = (self.M_ned2enu @ pos_ned) + self.p_obj_in

        # Quaternione [w,x,y,z] per il resto del sistema
        self.current_quat_wxyz = np.array([q_flu2enu[3], q_flu2enu[0], q_flu2enu[1], q_flu2enu[2]])

        # Velocità NED → ENU
        vel_ned = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        self.current_vel_enu = self.M_ned2enu @ vel_ned

        # Velocità angolare FRD → FLU
        ang_vel_frd = np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])
        self.current_ang_vel = M_frd2flu @ ang_vel_frd

        if not self.first_odom_received:
            self.first_odom_received = True
            self.get_logger().info(f"Prima odometria PX4 peg-drone ricevuta: {self.current_position_enu}")

        # Pubblica posa come PoseStamped (per RViz/bridge)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(self.current_position_enu[0])
        pose_msg.pose.position.y = float(self.current_position_enu[1])
        pose_msg.pose.position.z = float(self.current_position_enu[2])
        pose_msg.pose.orientation.w = float(self.current_quat_wxyz[0])
        pose_msg.pose.orientation.x = float(self.current_quat_wxyz[1])
        pose_msg.pose.orientation.y = float(self.current_quat_wxyz[2])
        pose_msg.pose.orientation.z = float(self.current_quat_wxyz[3])
        self.pose_pub.publish(pose_msg)

        # Pubblica Odometry per il MPC planner
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = 'peg_base_link'
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = float(self.current_vel_enu[0])
        odom_msg.twist.twist.linear.y = float(self.current_vel_enu[1])
        odom_msg.twist.twist.linear.z = float(self.current_vel_enu[2])
        odom_msg.twist.twist.angular.x = float(self.current_ang_vel[0])
        odom_msg.twist.twist.angular.y = float(self.current_ang_vel[1])
        odom_msg.twist.twist.angular.z = float(self.current_ang_vel[2])
        self.odom_pub.publish(odom_msg)

    def controller_ready_callback(self, msg: Bool):
        """Callback chiamato quando il controllore invia il segnale di pronto."""
        if msg.data and not self.is_ready:
            self.get_logger().info("Segnale 'ready' ricevuto dal planner del drone. Inizio movimento lungo la traiettoria.")
            self.is_ready = True
            self.destroy_subscription(self.ready_subscription)

    # ==================== Controllo PX4 ====================

    def publish_offboard_control_mode(self):
        """Pubblica OffboardControlMode in position mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.timestamp = 0  # PX4 auto-compila con hrt_absolute_time()
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, p_enu, yaw_enu=float('nan')):
        """Converte posizione ENU in NED (riferita allo spawn offset) e pubblica TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = 0

        # Sottraiamo lo spawn offset per inviare a PX4 le coordinate nel suo frame locale EKF2
        p_local_enu = p_enu - self.p_obj_in

        # Conversione ENU → NED
        p_ned = self.M_enu2ned @ p_local_enu
        msg.position[0] = float(p_ned[0])
        msg.position[1] = float(p_ned[1])
        msg.position[2] = float(p_ned[2])

        # Yaw: ENU (CCW da East) → NED (CW da North)
        # yaw_NED = pi/2 - yaw_ENU
        if not np.isnan(yaw_enu):
            msg.yaw = float(np.pi / 2.0 - yaw_enu)
        else:
            msg.yaw = float('nan')

        # Velocità e accelerazione NaN = non specificati
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')

        self.traj_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 2       # Instance 1 → target_system 2
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = 0
        self.vehicle_cmd_pub.publish(msg)

    def manage_offboard_state(self):
        """Gestione arming e offboard mode per il peg-drone."""
        self.startup_counter += 1
        
        # Aspettiamo 200 cicli (~1s a 200Hz) per stabilizzare i setpoint
        if self.startup_counter < 200:
            return
            
        if self.startup_counter % 10 == 0:
            if not self.is_offboard:
                self.get_logger().info("Peg-drone: richiesta Offboard mode...", throttle_duration_sec=2.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif not self.is_armed:
                self.get_logger().info("Peg-drone: armamento motori...", throttle_duration_sec=2.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                if not hasattr(self, '_armed_logged'):
                    self.get_logger().info("Peg-drone armato e in Offboard! Pronto per la traiettoria.")
                    self._armed_logged = True

    # ==================== Loop principale ====================

    def publish_next_pose(self):
        """Pubblica il prossimo setpoint per il peg-drone PX4."""
        
        # Sempre pubblicare offboard control mode (PX4 lo richiede a >2Hz)
        self.publish_offboard_control_mode()

        # Determina l'indice corrente nella traiettoria
        if self.current_index >= len(self.traj_time):
            idx = len(self.traj_time) - 1
            if not hasattr(self, 'finished_logged'):
                is_finished = Bool()
                is_finished.data = True
                self.path_finished_pub.publish(is_finished)
                self.get_logger().info("Fine traiettoria. Il peg-drone rimarrà nella posizione finale.")
                self.finished_logged = True
        else:
            idx = self.current_index

        # Posizione target in ENU
        p = self.p_obj[idx]
        rpy = self.rpy_obj[idx]

        # Pubblica TrajectorySetpoint a PX4
        self.publish_trajectory_setpoint(p, yaw_enu=rpy[2])

        # Gestione arming/offboard
        self.manage_offboard_state()

        # Avanza lungo la traiettoria solo se il camera-drone MPC ha dato il via libera
        # e il peg-drone è armato e in volo
        if self.is_ready and self.is_armed and self.current_index < len(self.traj_time):
            self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    peg_planner_node = PegPlannerNode()
    rclpy.spin(peg_planner_node)
    peg_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
