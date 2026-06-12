#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import atan2
from drone_mpc_pkg.common import quat_to_R
from scipy.spatial.transform import Rotation as Rot

# --- PX4 MESSAGES IMPORTS ---
from px4_msgs.msg import VehicleOdometry

def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0*(qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
    return atan2(siny_cosp, cosy_cosp)

def quat_to_rpy(qw, qx, qy, qz):
    sinr_cosp = 2*(qw*qx + qy*qz)
    cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2*(qw*qy - qz*qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    siny_cosp = 2*(qw*qz + qx*qy)
    cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)

class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        self.declare_parameter('save_path', '/tmp/pid_run.npz')
        self.declare_parameter('log_hz', 50.0) 
        self.declare_parameter('save_ref_flag', True)
        
        # Nuovi parametri per metadata
        self.declare_parameter('mass', 2.064)
        self.declare_parameter('cam_x', 0.0)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('ft_topic', '/world/interaction/model/x500_interaction_0/joint/end_eff_sens_joint/force_torque')

        self.save_path = self.get_parameter('save_path').value
        self.log_hz    = float(self.get_parameter('log_hz').value)
        self.log_dt    = 1.0 / max(self.log_hz, 1e-3)
        self.save_ref_flag = bool(self.get_parameter('save_ref_flag').value)
        
        self.mass = self.get_parameter('mass').value
        self.cam_offset = np.array([
            self.get_parameter('cam_x').value,
            self.get_parameter('cam_y').value,
            self.get_parameter('cam_z').value
        ])
        self.start_offset = np.array([
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_z').value
        ])
        self.ft_topic = self.get_parameter('ft_topic').value

        self.logging_enabled = False
        self.last_log_time = None

        # Liste per dati grezzi (per velocità in callback)
        self.t, self.raw_pos, self.raw_q, self.raw_v, self.raw_omega = [], [], [], [], []
        
        # Liste per riferimenti e dati esterni
        self.pref_pos, self.pref_rpy, self.pref_q, self.vref, self.omegaref = [], [], [], [], []
        self.wrench_cmd, self.wrench_ref, self.wrench_target, self.t_ref = [], [], [], []
        self.peg_pos, self.online_ref, self.online_visual_ref = [], [], []
        self.peg_ext_force = []
        self.delta_p = []

        self.last_peg_pos = [0.0, 0.0, 0.0]
        self.last_online_ref = [0.0] * 6
        self.last_online_visual_ref = [0.0, 0.0, 0.0]
        self.last_pref_pos, self.last_pref_rpy, self.last_pref_q = [0.0]*3, [0.0]*3, [1.0, 0.0, 0.0, 0.0]
        self.last_vref, self.last_omegaref = [0.0]*3, [0.0]*3
        
        self.last_w_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_w_ref = [0.0, 0.0, 0.0, 0.0]
        self.last_w_target = [0.0, 0.0, 0.0, 0.0]
        
        self.haptic_force = []
        self.last_haptic_force = [0.0, 0.0, 0.0]
        
        self.last_peg_ext_force = [0.0, 0.0, 0.0]
        self.last_delta_p = [0.0, 0.0, 0.0]

        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(PoseStamped, '/peg_pose', self.cb_peg_pose, 10)
        self.create_subscription(Float64MultiArray, '/online_spherical_ref', self.cb_online_ref, 10)
        self.create_subscription(Float64MultiArray, '/online_visual_ref', self.cb_online_visual_ref, 10)
        self.create_subscription(PoseStamped,  '/optimal_drone_pose',  self.cb_ref_pose,   10)
        self.create_subscription(TwistStamped, '/velocity_reference', self.cb_ref_twist,  10)
        self.create_subscription(Wrench, '/optimal_wrench', self.cb_wrench_ref, 10)
        self.create_subscription(Wrench, '/wrench_reference', self.cb_wrench_target, 10)
        self.create_subscription(Wrench, '/wrench_cmd', self.cb_wrench_cmd, 10)
        self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.cb_haptic_force, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb_px4_odom, px4_qos_profile)
        self.create_subscription(Wrench, self.ft_topic, self.cb_peg_ft, 10)
        self.create_subscription(Vector3Stamped, '/delta_p', self.cb_delta_p, 10)
        
        self.get_logger().info(f'Logger ottimizzato avviato | Salva in: {self.save_path}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_wrench_cmd(self, msg: Wrench):
        self.last_w_cmd = [msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_wrench_ref(self, msg: Wrench):
        self.last_w_ref = [msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_wrench_target(self, msg: Wrench):
        self.last_w_target = [msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_haptic_force(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.last_haptic_force = [msg.data[0], msg.data[1], msg.data[2]]

    def cb_peg_ft(self, msg: Wrench):
        # Logghiamo le 3 componenti lineari (puoi anche loggare i torque aggiungendo elementi all'array)
        self.last_peg_ext_force = [msg.force.x, msg.force.y, msg.force.z]

    def cb_delta_p(self, msg: Vector3Stamped):
        self.last_delta_p = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_ref_pose(self, msg: PoseStamped):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        qw, qx, qy, qz = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.last_pref_pos = p
        self.last_pref_rpy = quat_to_rpy(qw, qx, qy, qz)
        self.last_pref_q   = np.array([qw, qx, qy, qz], dtype=float)
        self.t_ref.append(self.now_sec())
        if not self.logging_enabled:
            self.logging_enabled = True

    def cb_ref_twist(self, msg: TwistStamped):
        self.last_vref = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float)
        self.last_omegaref = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z], dtype=float)

    def cb_px4_odom(self, msg: VehicleOdometry):
        if not self.logging_enabled:
            return
        t_now = self.now_sec()
        if self.last_log_time is not None and (t_now - self.last_log_time < self.log_dt):
            return

        # Salvataggio dati grezzi (molto veloce)
        self.t.append(t_now)
        self.raw_pos.append(list(msg.position))
        self.raw_q.append(list(msg.q))
        self.raw_v.append(list(msg.velocity))
        self.raw_omega.append(list(msg.angular_velocity))

        # Riferimenti
        self.pref_pos.append(self.last_pref_pos)
        self.pref_rpy.append(self.last_pref_rpy)
        self.pref_q.append(self.last_pref_q)
        self.vref.append(self.last_vref)
        self.omegaref.append(self.last_omegaref)
        self.wrench_cmd.append(self.last_w_cmd.copy())
        self.wrench_ref.append(self.last_w_ref.copy())
        self.wrench_target.append(self.last_w_target.copy())
        self.peg_pos.append(self.last_peg_pos)
        self.online_ref.append(self.last_online_ref)
        self.online_visual_ref.append(self.last_online_visual_ref) 
        self.haptic_force.append(self.last_haptic_force.copy())
        self.peg_ext_force.append(self.last_peg_ext_force.copy())
        self.delta_p.append(self.last_delta_p.copy())

        self.last_log_time = t_now

    def cb_peg_pose(self, msg: PoseStamped):
        self.last_peg_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def cb_online_ref(self, msg: Float64MultiArray):
        self.last_online_ref = list(msg.data)    

    def cb_online_visual_ref(self, msg: Float64MultiArray):
        self.last_online_visual_ref = list(msg.data)   
    
    def save(self):
        T = np.asarray(self.t)
        if not T.size:
            self.get_logger().warn("Nessun dato loggato, salvataggio annullato.")
            return
        T_rel = T - T[0]

        # --- TRASFORMAZIONI DI COORDINATE (NED -> ENU, FRD -> FLU) ---
        raw_pos = np.asarray(self.raw_pos)
        raw_q = np.asarray(self.raw_q) # PX4 [qw, qx, qy, qz]
        raw_v = np.asarray(self.raw_v)
        raw_omega = np.asarray(self.raw_omega)

        M_ned2enu = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        M_frd2flu = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

        # Posizione e Velocità: ENU = M_ned2enu @ NED
        pos = (M_ned2enu @ raw_pos.T).T + self.start_offset
        v = (M_ned2enu @ raw_v.T).T
        omega = (M_frd2flu @ raw_omega.T).T

        # Rotazione: R_flu2enu = M_ned2enu @ R_frd2ned @ M_frd2flu
        # Usiamo scipy per gestire il blocco di quaternioni in modo efficiente
        q_scipy_fmt = np.column_stack((raw_q[:, 1], raw_q[:, 2], raw_q[:, 3], raw_q[:, 0])) # [qx, qy, qz, qw]
        R_frd2ned = Rot.from_quat(q_scipy_fmt).as_matrix()
        R_flu2enu = M_ned2enu @ R_frd2ned @ M_frd2flu
        rot_flu2enu = Rot.from_matrix(R_flu2enu)
        q_enu = rot_flu2enu.as_quat() # [qx, qy, qz, qw]
        q = np.column_stack((q_enu[:, 3], q_enu[:, 0], q_enu[:, 1], q_enu[:, 2])) # [qw, qx, qy, qz]
        rpy = rot_flu2enu.as_euler('xyz')
        
        peg_pos = np.asarray(self.peg_pos)
        online_visual_ref = np.asarray(self.online_visual_ref)
        online_ref = np.asarray(self.online_ref)
        peg_ext_force = np.asarray(self.peg_ext_force)

        # 1. Calcolo derivate numeriche
        acc = np.zeros_like(v)
        ang_acc = np.zeros_like(omega)
        jerk = np.zeros_like(v)
        snap = np.zeros_like(v)
        
        if len(T_rel) > 1:
            for i in range(3):
                acc[:, i] = np.gradient(v[:, i], T_rel)
                ang_acc[:, i] = np.gradient(omega[:, i], T_rel)
                jerk[:, i] = np.gradient(acc[:, i], T_rel)
                snap[:, i] = np.gradient(jerk[:, i], T_rel)

        # 2. Calcolo Camera Position e Visual/Spherical Actuals
        p_cam = pos + rot_flu2enu.apply(self.cam_offset)
        p_rel_world_obj2cam = p_cam - peg_pos
        p_rel_cam = rot_flu2enu.inv().apply(peg_pos - p_cam) # [Xc, Yc, Zc]
        
        Xc, Yc, Zc = p_rel_cam[:, 0], p_rel_cam[:, 1], p_rel_cam[:, 2]
        radius = np.linalg.norm(p_rel_world_obj2cam, axis=1)
        pan = np.arctan2(p_rel_world_obj2cam[:, 1], p_rel_world_obj2cam[:, 0])
        tilt = np.arcsin(Zc / np.clip(radius, 1e-3, None))

        out = dict(
            t=T_rel, t_ref=np.asarray(self.t_ref),
            pos=pos, rpy=rpy, q=q,
            v=v, omega=omega,
            pref_pos=np.asarray(self.pref_pos),
            pref_rpy=np.asarray(self.pref_rpy),
            pref_q=np.asarray(self.pref_q),
            vref=np.asarray(self.vref),
            omegaref=np.asarray(self.omegaref),
            wrench_cmd=np.asarray(self.wrench_cmd),
            wrench_ref=np.asarray(self.wrench_ref),
            wrench_target=np.asarray(self.wrench_target),
            haptic_force=np.asarray(self.haptic_force),
            peg_pos=peg_pos, online_ref=online_ref,
            online_visual_ref=online_visual_ref,
            peg_ext_force=peg_ext_force,
            delta_p=np.asarray(self.delta_p),
            acc=acc, ang_acc=ang_acc, jerk=jerk, snap=snap,
            p_cam=p_cam, Xc=Xc, Yc=Yc, Zc=Zc, 
            radius_real=radius, pan_real=pan, tilt_real=tilt,
            mass=self.mass, cam_offset=self.cam_offset
        )

        np.savez(self.save_path, **out)
        self.get_logger().info(f"Salvataggio completato in {self.save_path}. Elaborati {len(T)} campioni.")

def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
