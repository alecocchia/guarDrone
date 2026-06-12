#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
import numpy as np
import math

from drone_mpc_pkg.planner import generate_trapezoidal_trajectory

def quaternion_to_euler(w, x, y, z):
    """Converte quaternione in angoli di eulero (roll, pitch, yaw)"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class OffboardTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('offboard_trajectory_planner')

        #self.declare_parameter('use_sim_time', True)
        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        
        # Parametri della cinematica
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('a_max', 2.0)
        self.declare_parameter('dt', 0.5) # 20Hz

        ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.a_max = self.get_parameter('a_max').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        prefix = f'/{ns}' if ns else ''

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', 1)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', 1)

        # Subscribers
        self.odom_sub = self.create_subscription(VehicleOdometry, f'{prefix}/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.target_sub = self.create_subscription(PoseStamped, 'target_pose', self.target_cb, 10)
        self.enabled_sub = self.create_subscription(Bool, 'offboard_traj_enabled', self.enabled_cb, 10)

        # Stato interno
        self.current_pos = np.zeros(3)
        self.current_rpy = np.zeros(3)
        self.has_odom = False
        self.offboard_traj_enabled = True

        # Traiettoria
        self.traj_p = None
        self.traj_rpy = None
        self.current_index = 0

        # Timer a 50Hz
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info("Offboard Trajectory Planner avviato. In attesa di odometria e target...")

    def odom_cb(self, msg):
        # Odometria PX4 è in NED. Trasformiamo in ENU per comodità
        x_enu = msg.position[1]
        y_enu = msg.position[0]
        z_enu = -msg.position[2]
        
        # Applica offset di spawn (i parametri sono in ENU)
        self.current_pos[0] = x_enu + self.get_parameter('start_x').value
        self.current_pos[1] = y_enu + self.get_parameter('start_y').value
        self.current_pos[2] = z_enu + self.get_parameter('start_z').value
        
        # Quaternione NED -> rpy NED -> rpy ENU (yaw_enu = -yaw_ned + pi/2)
        r, p, yaw_ned = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self.current_rpy[2] = -yaw_ned + np.pi/2.0
        
        self.has_odom = True

    def enabled_cb(self, msg):
        self.offboard_traj_enabled = msg.data
        if not self.offboard_traj_enabled:
            self.get_logger().info("Trajectory Planner DISABILITATO (Passaggio a MPC).")

    def target_cb(self, msg):
        if not self.has_odom:
            self.get_logger().warn("Ricevuto target, ma odometria non ancora valida. Ignoro.")
            return

        t_x = msg.pose.position.x
        t_y = msg.pose.position.y
        t_z = msg.pose.position.z
        
        if msg.pose.orientation.w == 0.0 and msg.pose.orientation.x == 0.0 and msg.pose.orientation.y == 0.0 and msg.pose.orientation.z == 0.0:
            t_yaw = self.current_rpy[2]
            self.get_logger().info("Target senza orientamento esplicito: mantengo lo yaw corrente.")
        else:
            _, _, t_yaw = quaternion_to_euler(
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            )

        x0 = [self.current_pos[0], self.current_pos[1], self.current_pos[2], 0.0, 0.0, self.current_rpy[2]]
        x_ref = [t_x, t_y, t_z, 0.0, 0.0, t_yaw]

        self.get_logger().info(f"Calcolo traiettoria da {x0[:3]} a {x_ref[:3]}...")

        # Genera traiettoria
        t_vec, p_vals, rpy_vals = generate_trapezoidal_trajectory(
            x0, x_ref, dt=self.dt, v_max=self.v_max, a_max=self.a_max
        )

        self.traj_p = p_vals
        self.traj_rpy = rpy_vals
        self.current_index = 0

        self.get_logger().info(f"Traiettoria calcolata! Punti: {len(self.traj_p)}, Tempo: {t_vec[-1]:.2f}s")

    def timer_cb(self):
        if not self.offboard_traj_enabled:
            return

        # Pubblica sempre OffboardControlMode
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

        if self.traj_p is None:
            # Se non abbiamo un target, cerchiamo di stare fermi se abbiamo l'odometria
            if self.has_odom:
                self.publish_setpoint(self.current_pos, self.current_rpy[2])
            return

        # Avanza lungo la traiettoria
        idx = min(self.current_index, len(self.traj_p) - 1)
        p = self.traj_p[idx]
        y = self.traj_rpy[idx][2]
        
        self.publish_setpoint(p, y)

        if self.current_index < len(self.traj_p):
            self.current_index += 1

    def publish_setpoint(self, pos_enu, yaw_enu):
        # Converte da ENU a NED per PX4
        # Rimuove l'offset per tornare in coordinate locali (quelle usate da PX4)
        local_x = pos_enu[0] - self.get_parameter('start_x').value
        local_y = pos_enu[1] - self.get_parameter('start_y').value
        local_z = pos_enu[2] - self.get_parameter('start_z').value
        
        msg = TrajectorySetpoint()
        msg.position = [local_y, local_x, -local_z]
        msg.yaw = -yaw_enu + np.pi/2.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
