#!/usr/bin/env python3
import sys
import math
import numpy as np

def main():
    target_dist = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
    except ImportError:
        sys.stderr.write("Errore: rclpy o geometry_msgs non disponibili. Assicurati che ROS2 sia sorgato.\n")
        sys.exit(1)

    rclpy.init(args=None)
    node = Node('get_front_coord_temp')

    cam_pos = None
    cam_quat = None
    peg_pos = None

    def cam_cb(msg):
        nonlocal cam_pos, cam_quat
        cam_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        cam_quat = [q.x, q.y, q.z, q.w]

    def peg_cb(msg):
        nonlocal peg_pos
        peg_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def drone_pose_cb(msg):
        nonlocal cam_pos, cam_quat
        if cam_pos is None:
            cam_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = msg.pose.orientation
            cam_quat = [q.x, q.y, q.z, q.w]

    node.create_subscription(PoseStamped, '/drone_cam_pose', cam_cb, 10)
    node.create_subscription(PoseStamped, '/drone_pose', drone_pose_cb, 10)
    node.create_subscription(PoseStamped, '/peg_pose', peg_cb, 10)
    node.create_subscription(PoseStamped, '/peg_actual_pose', peg_cb, 10)

    # Attendiamo fino a 1.2 secondi per ricevere i messaggi
    start_time = node.get_clock().now()
    timeout_sec = 1.2
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if cam_pos is not None and peg_pos is not None:
            break
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout_sec:
            break

    node.destroy_node()
    rclpy.shutdown()

    if cam_pos is None:
        sys.stderr.write("Errore: nessuna posa ricevuta da /drone_cam_pose o /drone_pose (il nodo GuaDrone è attivo?).\n")
        sys.exit(2)

    # Calcolo coordinate della sfera
    if peg_pos is not None:
        # Metodo Line-Of-Sight (LOS) ideale tra GuaDrone e Peg Drone
        vec = peg_pos - cam_pos
        dist_between = np.linalg.norm(vec)
        if dist_between > 1e-3:
            u_dir = vec / dist_between
            # Se la distanza richiesta è eccessiva rispetto alla distanza tra droni, piazzala a metà
            actual_d = min(target_dist, 0.7 * dist_between)
            sphere_pos = cam_pos + actual_d * u_dir
        else:
            sphere_pos = cam_pos + np.array([target_dist, 0.0, 0.0])
    elif cam_quat is not None:
        # Metodo basato sullo yaw della camera
        qx, qy, qz, qw = cam_quat
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        sphere_pos = cam_pos + np.array([target_dist * math.cos(yaw), target_dist * math.sin(yaw), 0.0])
    else:
        sphere_pos = cam_pos + np.array([target_dist, 0.0, 0.0])

    print(f"{sphere_pos[0]:.3f} {sphere_pos[1]:.3f} {sphere_pos[2]:.3f}")

if __name__ == '__main__':
    main()
