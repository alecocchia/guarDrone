#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
import numpy as np
from drone_mpc_pkg.common import RPY_to_quat
from drone_mpc_pkg.planner import generate_trapezoidal_trajectory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

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

        # --- Lettura dei valori da launchfile ---
        peg_x = self.get_parameter('peg_start_x').value
        peg_y = self.get_parameter('peg_start_y').value
        peg_z = self.get_parameter('peg_start_z').value
        peg_roll = self.get_parameter('peg_start_roll').value
        peg_pitch = self.get_parameter('peg_start_pitch').value
        peg_yaw = self.get_parameter('peg_start_yaw').value

        self.p_obj_in = np.array([peg_x, peg_y, peg_z])
        self.rot_obj_in = np.array([peg_roll, peg_pitch, peg_yaw])
        
        self.get_logger().info(f"Peg posizionato correttamente in: {self.p_obj_in}")

        self.p_obj_in = np.array([peg_x, peg_y, peg_z])
        self.rot_obj_in = np.array([peg_roll, peg_pitch, peg_yaw])
        
        self.get_logger().info(f"Peg posizionato in: {self.p_obj_in}. Inizio generazione traiettoria 2-fasi.")

        # --- Traiettoria in due fasi ---
        Tf1 = 10.0  # Fine fase decollo (10s)
        Tf2 = 25.0  # Fine fase inserimento (totale 25s)

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

        
        # Profilo QoS per garantire che il messaggio venga ricevuto
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # Publisher per l'odometria continua (usato dal controllore)
        self.odom_pub = self.create_publisher(Odometry, '/peg_odom', qos_profile)

        self.path_finished_pub = self.create_publisher(Bool, '/peg_path_finished', qos_profile)

        # Publisher per la singola Pose (usato per la visualizzazione in tempo reale)
        # Sostituito da PoseStamped per includere il frame di riferimento
        self.pose_pub = self.create_publisher(PoseStamped, '/peg_pose', 1)

        qos_ready = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        # Subscriber che attende il segnale di pronto dal controllore
        self.ready_subscription = self.create_subscription(
            Bool,
            '/drone_planner_ready',
            self.controller_ready_callback,
            qos_ready)
        
        self.current_index = 0
        self.is_ready = False
        
        # Iniziamo subito a pubblicare la posa iniziale in modo continuo
        # sicché l'MPC possa leggerla e inizializzarsi.
        self.timer = self.create_timer(self.ts, self.publish_next_pose)
        
        self.get_logger().info("Peg planner Node avviato. Pubblicazione posa inziale in corso. In attesa del segnale 'ready' dal planner.")


    def controller_ready_callback(self, msg: Bool):
        """Callback chiamato quando il controllore invia il segnale di pronto."""
        if msg.data and not self.is_ready:
            self.get_logger().info("Segnale 'ready' ricevuto dal planner del drone. Inizio movimento lungo la traiettoria.")
            self.is_ready = True
            self.destroy_subscription(self.ready_subscription)


    def publish_next_pose(self):
        """Pubblica la prossima posa dell'oggetto e gestisce la fine della traiettoria."""
        if self.current_index >= len(self.traj_time):
            is_finished = Bool()
            is_finished.data = True
            self.path_finished_pub.publish(is_finished)
            self.get_logger().info("Fine traiettoria. L'oggetto rimarrà nella posizione finale.")
            self.timer.cancel()
            return

        # Crea un messaggio PoseStamped per includere l'header
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'world'

        p = self.p_obj[self.current_index]
        rpy = self.rpy_obj[self.current_index]
        q = RPY_to_quat(rpy[0], rpy[1], rpy[2])

        pose_stamped_msg.pose.position.x = float(p[0])
        pose_stamped_msg.pose.position.y = float(p[1])
        pose_stamped_msg.pose.position.z = float(p[2])
        pose_stamped_msg.pose.orientation.w = float(q[0])
        pose_stamped_msg.pose.orientation.x = float(q[1])
        pose_stamped_msg.pose.orientation.y = float(q[2])
        pose_stamped_msg.pose.orientation.z = float(q[3])
        
        self.pose_pub.publish(pose_stamped_msg)

        # Calcolo velocità numerica
        if self.current_index > 0:
            p_prev = self.p_obj[self.current_index - 1]
            v = (p - p_prev) / self.ts
        else:
            v = np.zeros(3)

        # Creazione e pubblicazione Odometry
        odom_msg = Odometry()
        odom_msg.header = pose_stamped_msg.header
        odom_msg.child_frame_id = 'peg_base_link'
        odom_msg.pose.pose = pose_stamped_msg.pose
        odom_msg.twist.twist.linear.x = float(v[0])
        odom_msg.twist.twist.linear.y = float(v[1])
        odom_msg.twist.twist.linear.z = float(v[2])
        self.odom_pub.publish(odom_msg)

        # Avanza lungo la traiettoria solo se l'MPC ha dato il via libera
        if self.is_ready:
            self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    peg_planner_node = PegPlannerNode()
    rclpy.spin(peg_planner_node)
    peg_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
