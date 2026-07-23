#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PX4VisualOdomPublisher(Node):
    def __init__(self):
        super().__init__('PX4_visual_odom_publisher')

        # Matrice di trasformazione dal mondo ENU al mondo NED
        self.M_ned2enu = np.array([
            [0.0, 1.0, 0.0], 
            [1.0, 0.0, 0.0], 
            [0.0, 0.0, -1.0]
        ])

        self.M_enu2ned = self.M_ned2enu.T
                              
        # Matrice di trasformazione dal corpo FLU al corpo FRD
        self.M_frd2flu = np.array([
            [1.0,  0.0,  0.0], 
            [0.0, -1.0,  0.0], 
            [0.0,  0.0, -1.0]
        ])

        self.M_flu2frd = self.M_frd2flu.T
        self.optitrack2enu = np.array([[1,0,0],[0,0,1],[0,-1,0]]).T

        # Profilo QoS del tipo "sensor_data"
        self.qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self.optitrack_odom_sub = self.create_subscription(
            Odometry, 
            '/optitrack/body_2/odometry', 
            self.optitrack_odom_cb, 
            self.qos
        )
        
        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            self.qos
        )

    def optitrack_odom_cb(self, msg: Odometry):
        # conversione del Timestamp
        timestamp_us = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)

        # trasformazione della Posizione (ENU -> NED)
        p_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        p_enu = self.optitrack2enu @ p_enu.T
        
        p_ned = self.M_enu2ned @ p_enu.T

        # trasformazione dell'Orientamento (FLU -> FRD e ENU -> NED)
        # SciPy utilizza il formato dei quaternioni [x, y, z, w]
        q_enu = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # converte il quaternione OptiTrack in matrice di rotazione
        R_flu2optitrack = Rotation.from_quat(q_enu).as_matrix()
        R_flu2enu = self.optitrack2enu @ R_flu2optitrack
        
        # applica le matrici fisse: R_ned2frd = M_world * R_flu2enu * M_body
        R_frd2ned = self.M_frd2flu @ R_flu2enu @ self.M_enu2ned
        self.M_ned2enu @ R_flu2enu @ self.M_frd2flu
        
        # riconverte la matrice corretta in quaternione
        q_frd2ned_scipy = Rotation.from_matrix(R_frd2ned).as_quat()
        
        # PX4 si aspetta il quaternione nel formato [w, x, y, z]
        q_px4 = [
            q_frd2ned_scipy[3], 
            q_frd2ned_scipy[0], 
            q_frd2ned_scipy[1], 
            q_frd2ned_scipy[2]
        ]

        # costruzione del messaggio VehicleOdometry
        out_msg = VehicleOdometry()
        out_msg.timestamp = timestamp_us
        out_msg.timestamp_sample = timestamp_us
        
        # specificazione dei frame di riferimento per l'EKF2
        out_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # assegnazione posizione e orientamento
        out_msg.position = [float(p_ned[0]), float(p_ned[1]), float(p_ned[2])]
        out_msg.q = [float(q_px4[0]), float(q_px4[1]), float(q_px4[2]), float(q_px4[3])]

        # il nodo OptiTrack in C++ non pubblica le velocità (vengono lasciate a 0).
        # per PX4 è fondamentale passare "NaN" sui campi che non si stanno misurando,
        # in modo che l'EKF2 non consideri uno zero statico come un dato reale.
        out_msg.velocity = [float('nan'), float('nan'), float('nan')]
        out_msg.angular_velocity = [float('nan'), float('nan'), float('nan')]
        out_msg.position_variance = [float('nan'), float('nan'), float('nan')]
        out_msg.orientation_variance = [float('nan'), float('nan'), float('nan')]
        out_msg.velocity_variance = [float('nan'), float('nan'), float('nan')]

        # pubblicazione messaggio VehicleOdometry 
        self.visual_odom_pub.publish(out_msg)


def main():
    rclpy.init()
    node = PX4VisualOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()