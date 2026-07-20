#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Vector3Stamped
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from math import atan2
from utils_pkg.utils_np import quat_to_R
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

        self.declare_parameter('save_path', '/tmp/sim_run.npz')
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
        self.declare_parameter('ft_topic', '/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque')
        self.declare_parameter('peg_px4_ns', 'px4_1')
        self.declare_parameter('peg_start_x', 0.0)
        self.declare_parameter('peg_start_y', 0.0)
        self.declare_parameter('peg_start_z', 0.0)

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
        peg_ns = self.get_parameter('peg_px4_ns').value
        self.peg_ns_prefix = f'/{peg_ns}' if peg_ns else ''
        self.peg_start_offset = np.array([
            self.get_parameter('peg_start_x').value,
            self.get_parameter('peg_start_y').value,
            self.get_parameter('peg_start_z').value
        ])

        self.logging_enabled = False
        self.last_log_time = None

        # Liste per dati grezzi (per velocità in callback)
        self.t, self.raw_pos, self.raw_q, self.raw_v, self.raw_omega = [], [], [], [], []
        
        # Liste per riferimenti e dati esterni
        self.pref_pos, self.pref_rpy, self.pref_q, self.vref, self.omegaref = [], [], [], [], []
        self.wrench_cmd, self.wrench_ref, self.wrench_target, self.t_ref = [], [], [], []
        self.peg_pos, self.online_ref, self.online_cyl_ref = [], [], []
        self.peg_ext_force = []
        self.estimated_wrench = []
        self.delta_p = []
        self.delta_p_sensor = []
        
        # Drone di interazione (peg): posizione attuale e riferimento in ENU
        self.peg_actual_pos = []  # posizione attuale drone interazione (ENU)
        self.peg_ref_pos = []     # riferimento nominale planner ammettenza (ENU)

        self.last_peg_pos = [0.0, 0.0, 0.0]
        self.last_online_ref    = [0.0, 0.0, 0.0]   # [r_cyl_ref, beta_ref, z_ref]
        self.last_online_cyl_ref = [0.0, 0.0, 0.0]  # alias (stesso topic, tenuto per compatibilità)
        self.last_pref_pos, self.last_pref_rpy, self.last_pref_q = [0.0]*3, [0.0]*3, [1.0, 0.0, 0.0, 0.0]
        self.last_vref, self.last_omegaref = [0.0]*3, [0.0]*3
        
        self.last_w_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_w_ref = [0.0, 0.0, 0.0, 0.0]
        self.last_w_target = [0.0, 0.0, 0.0, 0.0]
        
        self.haptic_force = []
        self.last_haptic_force = [0.0, 0.0, 0.0]
        
        self.last_peg_ext_force = [0.0, 0.0, 0.0]
        self.last_estimated_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_delta_p = [0.0, 0.0, 0.0]
        self.last_delta_p_sensor = [0.0, 0.0, 0.0]
        
        # Ultimi valori drone di interazione
        self.last_peg_actual_pos = [0.0, 0.0, 0.0]
        self.last_peg_ref_pos = [0.0, 0.0, 0.0]
        self.peg_actual_yaw = []       # yaw ENU drone di interazione
        self.last_peg_actual_yaw = 0.0
        self.peg_ref_yaw = []          # yaw ENU riferimento planner ammettenza
        self.last_peg_ref_yaw = 0.0
        self.peg_actual_vel = []       # velocità ENU drone interazione [vx, vy, vz]
        self.last_peg_actual_vel = [0.0, 0.0, 0.0]
        self.peg_actual_yaw_rate = []  # velocità angolare yaw FLU drone interazione [rad/s]
        self.last_peg_actual_yaw_rate = 0.0
        self.peg_ref_vel = []          # velocità ref ENU planner ammettenza
        self.last_peg_ref_vel = [0.0, 0.0, 0.0]
        self.peg_ref_yaw_rate = []     # yaw rate ref planner ammettenza
        self.last_peg_ref_yaw_rate = 0.0

        self.task_start_time = None    # timestamp di inizio missione (secondi)

        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(PoseStamped, '/peg_pose', self.cb_peg_pose, 10)
        self.create_subscription(Float64MultiArray, '/online_cylindrical_ref', self.cb_online_ref, 10)
        self.create_subscription(Float64MultiArray, '/online_visual_ref',       self.cb_online_cyl_ref, 10)
        self.create_subscription(PoseStamped,  '/optimal_drone_pose',  self.cb_ref_pose,   10)
        self.create_subscription(PoseStamped,  '/camera_ref_pose',     self.cb_ref_pose,   10)
        self.create_subscription(TwistStamped, '/velocity_reference', self.cb_ref_twist,  10)
        self.create_subscription(Wrench, '/optimal_wrench', self.cb_wrench_ref, 10)
        self.create_subscription(Wrench, '/wrench_reference', self.cb_wrench_target, 10)
        self.create_subscription(Wrench, '/wrench_cmd', self.cb_wrench_cmd, 10)
        self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.cb_haptic_force, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb_px4_odom, px4_qos_profile)
        self.create_subscription(Wrench, self.ft_topic, self.cb_peg_ft, 10)
        self.create_subscription(Wrench, '/estimated_wrench', self.cb_estimated_wrench, 10)
        self.create_subscription(Vector3Stamped, '/delta_p', self.cb_delta_p, 10)
        self.create_subscription(Vector3Stamped, '/delta_p_sensor', self.cb_delta_p_sensor, 10)
        # Drone di interazione: odometria attuale, riferimento nominale e velocità di riferimento
        peg_odom_topic = f'{self.peg_ns_prefix}/fmu/out/vehicle_odometry'
        self.create_subscription(VehicleOdometry, peg_odom_topic, self.cb_peg_odom, px4_qos_profile)
        self.create_subscription(PoseStamped, '/peg_ref_pose', self.cb_peg_ref_pose, 10)
        self.create_subscription(TwistStamped, '/peg_ref_twist', self.cb_peg_ref_twist, 10)
        # Trigger avvio logging: segnale dal supervisor al momento dell'arm+offboard
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Bool, '/logging/start', self.cb_logging_start, qos_latched)
        self.create_subscription(Bool, '/mpc_task/start', self.cb_task_start, qos_latched)
        
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

    def cb_estimated_wrench(self, msg: Wrench):
        self.last_estimated_wrench = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_delta_p(self, msg: Vector3Stamped):
        self.last_delta_p = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_delta_p_sensor(self, msg: Vector3Stamped):
        self.last_delta_p_sensor = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_ref_pose(self, msg: PoseStamped):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        qw, qx, qy, qz = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.last_pref_pos = p
        self.last_pref_rpy = quat_to_rpy(qw, qx, qy, qz)
        self.last_pref_q   = np.array([qw, qx, qy, qz], dtype=float)
        self.t_ref.append(self.now_sec())

    def cb_logging_start(self, msg: Bool):
        if msg.data and not self.logging_enabled:
            self.logging_enabled = True
            self.get_logger().info('Logging AVVIATO (segnale /logging/start ricevuto).')

    def cb_task_start(self, msg: Bool):
        if msg.data and self.task_start_time is None:
            self.task_start_time = self.now_sec()
            self.get_logger().info('Ricevuto start task, salvo timestamp.')

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
        self.online_cyl_ref.append(self.last_online_ref)  # stesso dato, alias
        self.haptic_force.append(self.last_haptic_force.copy())
        self.peg_ext_force.append(self.last_peg_ext_force.copy())
        self.estimated_wrench.append(self.last_estimated_wrench.copy())
        self.delta_p.append(self.last_delta_p.copy())
        self.delta_p_sensor.append(self.last_delta_p_sensor.copy())
        self.peg_actual_pos.append(list(self.last_peg_actual_pos))
        self.peg_ref_pos.append(list(self.last_peg_ref_pos))
        self.peg_actual_yaw.append(self.last_peg_actual_yaw)
        self.peg_ref_yaw.append(self.last_peg_ref_yaw)
        self.peg_actual_vel.append(list(self.last_peg_actual_vel))
        self.peg_actual_yaw_rate.append(self.last_peg_actual_yaw_rate)
        self.peg_ref_vel.append(list(self.last_peg_ref_vel))
        self.peg_ref_yaw_rate.append(self.last_peg_ref_yaw_rate)

        self.last_log_time = t_now

    def cb_peg_pose(self, msg: PoseStamped):
        self.last_peg_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def cb_online_ref(self, msg: Float64MultiArray):
        self.last_online_ref = list(msg.data)[:3]

    def cb_online_cyl_ref(self, msg: Float64MultiArray):
        """Topic /visual_ref pubblica [r_cyl_ref, beta_ref, z_ref] — stessa cosa di online_ref."""
        self.last_online_cyl_ref = list(msg.data)[:3]

    def cb_peg_odom(self, msg: VehicleOdometry):
        """Odometria del drone di interazione (px4_1) - converte NED→ENU con spawn offset."""
        M_ned2enu = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        M_frd2flu = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
        # Posizione: NED → ENU + spawn offset
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        pos_enu = M_ned2enu @ pos_ned + self.peg_start_offset
        self.last_peg_actual_pos = pos_enu.tolist()
        # Velocità: NED → ENU
        vel_ned = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        self.last_peg_actual_vel = (M_ned2enu @ vel_ned).tolist()
        # Yaw ENU: stessa pipeline del drone principale
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]  # [qx, qy, qz, qw]
        R_frd2ned = Rot.from_quat(q_scipy).as_matrix()
        R_flu2enu = M_ned2enu @ R_frd2ned @ M_frd2flu
        self.last_peg_actual_yaw = float(Rot.from_matrix(R_flu2enu).as_euler('xyz')[2])
        # Yaw rate: FRD → FLU (omega_z_flu = -omega_z_frd)
        self.last_peg_actual_yaw_rate = float(-msg.angular_velocity[2])

    def cb_peg_ref_pose(self, msg: PoseStamped):
        """Posizione + yaw nominale di riferimento del peg planner (già in ENU)."""
        self.last_peg_ref_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Estrai yaw dal quaternione ENU codificato dal planner
        ox, oy, oz, ow = (msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)
        if abs(ox) + abs(oy) + abs(oz) + abs(ow) > 1e-6:  # quaternione valido
            self.last_peg_ref_yaw = float(Rot.from_quat([ox, oy, oz, ow]).as_euler('xyz')[2])

    def cb_peg_ref_twist(self, msg: TwistStamped):
        """Velocità + yaw rate nominali di riferimento del peg planner (ENU)."""
        self.last_peg_ref_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.last_peg_ref_yaw_rate = float(msg.twist.angular.z)
    
    def save(self):
        T = np.asarray(self.t)
        if not T.size:
            self.get_logger().warn("Nessun dato loggato, salvataggio annullato.")
            return
        T_rel = T - T[0]

        t_start_rel = (self.task_start_time - T[0]) if self.task_start_time else -1.0

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
        online_cyl_ref = np.asarray(self.online_cyl_ref)
        online_ref = np.asarray(self.online_ref)
        peg_ext_force = np.asarray(self.peg_ext_force)
        estimated_wrench = np.asarray(self.estimated_wrench)

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

        # 2. Coordinate cilindriche nel mondo basate sulla TELECAMERA
        # Trasformiamo l'offset della telecamera (nel frame body FLU) nel frame ENU 
        # R_flu2enu ha shape (N, 3, 3) e self.cam_offset ha shape (3,)
        cam_offset_world = np.einsum('nij,j->ni', R_flu2enu, self.cam_offset)
        p_cam = pos + cam_offset_world
        p_rel_world = p_cam - peg_pos                           # camera - oggetto
        r_cyl   = np.linalg.norm(p_rel_world[:, :2], axis=1)        # distanza 2D [m]
        beta_cyl  = np.arctan2(p_rel_world[:, 1],
                               p_rel_world[:, 0])            # azimut [rad]
        z_cyl = p_rel_world[:, 2]                            # elevazione Z [m]

        # Yaw attuale e yaw desiderato (puntare verso oggetto)
        yaw_actual   = rpy[:, 2]
        yaw_desired  = np.arctan2(-p_rel_world[:, 1], -p_rel_world[:, 0])
        yaw_err_cyl  = np.arctan2(np.sin(yaw_actual - yaw_desired),
                                  np.cos(yaw_actual - yaw_desired))

        online_cyl_ref = np.asarray(self.online_cyl_ref)  # [r_cyl_ref, beta_ref, z_ref]

        # 3. Target Cartesiano della Telecamera
        p_cam_target = np.zeros_like(p_cam)
        p_cam_target[:, 0] = peg_pos[:, 0] + online_cyl_ref[:, 0] * np.cos(online_cyl_ref[:, 1])
        p_cam_target[:, 1] = peg_pos[:, 1] + online_cyl_ref[:, 0] * np.sin(online_cyl_ref[:, 1])
        p_cam_target[:, 2] = peg_pos[:, 2] + online_cyl_ref[:, 2]

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
            peg_pos=peg_pos,
            online_ref=np.asarray(self.online_ref),
            online_cyl_ref=online_cyl_ref,
            p_cam=p_cam, p_cam_target=p_cam_target,
            # Grandezze cilindriche attuali
            r_cyl=r_cyl, beta_cyl=beta_cyl, z_cyl=z_cyl,
            yaw_err_cyl=yaw_err_cyl,
            peg_ext_force=peg_ext_force,
            estimated_wrench=estimated_wrench,
            delta_p=np.asarray(self.delta_p),
            delta_p_sensor=np.asarray(self.delta_p_sensor),
            peg_actual_pos=np.asarray(self.peg_actual_pos),
            peg_ref_pos=np.asarray(self.peg_ref_pos),
            peg_actual_yaw=np.asarray(self.peg_actual_yaw),
            peg_ref_yaw=np.asarray(self.peg_ref_yaw),
            peg_actual_vel=np.asarray(self.peg_actual_vel),
            peg_actual_yaw_rate=np.asarray(self.peg_actual_yaw_rate),
            peg_ref_vel=np.asarray(self.peg_ref_vel),
            peg_ref_yaw_rate=np.asarray(self.peg_ref_yaw_rate),
            acc=acc, ang_acc=ang_acc, jerk=jerk, snap=snap,
            mass=self.mass, cam_offset=self.cam_offset,
            task_start_time=np.array([t_start_rel])
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
