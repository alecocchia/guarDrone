#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Bool, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class FakePublisherNode(Node):
    def __init__(self):
        super().__init__('fake_publisher_node')
        
        # Profilo QoS compatibile con i messaggi PX4 (Best Effort)
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', px4_qos_profile)
        self.start_pub = self.create_publisher(Bool, '/mpc_task/start', 10)
        self.pov_pub = self.create_publisher(Float64MultiArray, '/pov_target', 10)
        
        # Frequenza di pubblicazione (50 Hz, come un odometria tipica)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Fake Publisher (Supervisor) avviato. Pubblicazione su /px4_1/fmu/out/vehicle_odometry, /mpc_task/start e /pov_target in corso.")
        
    def timer_callback(self):
        now = self.get_clock().now()
        
        # 1) Pubblica l'odometria fittizia del secondo drone (Peg)
        odom_msg = VehicleOdometry()
        odom_msg.timestamp = int(now.nanoseconds / 1000)
        
        # Posizione del peg (in coordinate NED di PX4). 
        # Modifica questi valori se vuoi che il peg sia da un'altra parte.
        # [0.0, 0.0, 0.0] significa che il peg è fermo all'origine (che per il MOCAP è lo zero).
        odom_msg.position = [0.0, 0.0, 0.0]
        
        # Quaternione (PX4 usa [w, x, y, z]). [1, 0, 0, 0] è orientamento nullo.
        odom_msg.q = [1.0, 0.0, 0.0, 0.0] 
        
        odom_msg.velocity = [0.0, 0.0, 0.0]
        odom_msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(odom_msg)
        
        # 2) Avvia la task dell'MPC in loop
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        
        # 3) Target dell'MPC: [r_cyl, beta, z]
        # Invece di usare il default [2.0, np.pi, 0.0] dell'MPC_planner_node, forziamo
        # il target. Es. [2.0, 0.0, 0.0] dice al drone di stare a 2m di distanza, azimut 0, quota = quota peg.
        # N.B. Evitiamo r_cyl = 0.0 se in passato ti ha dato problemi di singolarità sull'atan2.
        pov_msg = Float64MultiArray()
        pov_msg.data = [2.0, 0.0, 0.0] 
        self.pov_pub.publish(pov_msg)

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
