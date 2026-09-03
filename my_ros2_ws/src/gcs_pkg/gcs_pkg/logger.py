#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Vector3Stamped, Vector3
from std_msgs.msg import Float64MultiArray, Bool, Float64
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from utils_pkg.utils_np import cylindrical_to_cartesian

# --- PX4 MESSAGES IMPORTS ---
from px4_msgs.msg import VehicleOdometry


class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        self.declare_parameter('save_path', '/tmp/sim_run.mat')
        self.declare_parameter('log_hz', 50.0)
        self.declare_parameter('save_ref_flag', True)
        self.declare_parameter('mass', 2.064)
        self.declare_parameter('ft_topic', '/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque')

        import os
        import datetime
        import pytz
        raw_save_path     = self.get_parameter('save_path').value
        base_name = os.path.basename(raw_save_path)
        
        if 'hw' in base_name.lower() or 'exp' in base_name.lower():
            run_type = 'exp'
            base_dir = '/root/my_ros2_ws/HardwareScripts/bag_files'
        else:
            run_type = 'sim'
            base_dir = '/root/my_ros2_ws/SimulationScripts/bag_files'

        tz = pytz.timezone('Europe/Rome')    
        timestamp = datetime.datetime.now(tz).strftime('%Y%m%d_%H%M')
        self.out_dir = os.path.join(base_dir, f"{run_type}_{timestamp}")
        self.final_save_path = os.path.join(self.out_dir, base_name)
        
        self.log_hz        = float(self.get_parameter('log_hz').value)
        self.log_dt        = 1.0 / max(self.log_hz, 1e-3)
        self.save_ref_flag = bool(self.get_parameter('save_ref_flag').value)
        self.mass          = self.get_parameter('mass').value
        self.ft_topic      = self.get_parameter('ft_topic').value

        # Declare spawn coordinates to broadcast local frame
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        start_z = self.get_parameter('start_z').value

        # Setup Static TF Broadcaster
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        # i TF statici non scadono
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'spawn_origin'
        t.transform.translation.x = float(start_x)
        t.transform.translation.y = float(start_y)
        t.transform.translation.z = float(start_z)
        t.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(t)

        self.logging_enabled = False
        self.last_log_time   = None
        self.task_start_time = None

        # ------------------------------------------------------------------ #
        #  Arrays di logging (tutti in ENU/FLU — nessuna trasformazione qui)  #
        # ------------------------------------------------------------------ #
        self.t = []

        # Stato drone — da /drone_pose, /drone_rpy, /drone_velocity (MPC)
        self.pos   = []   # [x, y, z]          ENU
        self.q     = []   # [qw, qx, qy, qz]   ENU
        self.rpy   = []   # [roll, pitch, yaw]  ENU
        self.v     = []   # [vx, vy, vz]        ENU
        self.omega = []   # [ox, oy, oz]        FLU

        # Posizione camera nel mondo — da /drone_cam_pose (MPC)
        self.p_cam = []   # [x, y, z] ENU

        # PoV cilindrico attuale — da /actual_pov (MPC)
        # formato: [r_cyl, beta_cyl, z_cyl, yaw_err_cyl]
        self.actual_pov = []
        
        self.integral_action = []

        # Riferimenti drone
        self.pref_pos    = []
        self.pref_rpy    = []
        self.pref_q      = []
        self.vref        = []
        self.omegaref    = []
        self.optimal_wrench = []
        self.wrench_target = []
        self.t_ref       = []
        self.peg_pos     = []   # posizione peg ENU — da /peg_pose (MPC)
        self.online_ref      = []
        self.online_cyl_ref  = []
        self.haptic_force    = []
        self.peg_ext_force   = []
        self.estimated_wrench = []
        self.delta_p         = []
        self.delta_p_sensor  = []

        # Stato drone peg (ENU) — da fake_publisher (sim) / admittance_planner (real)
        self.peg_actual_pos      = []
        self.peg_actual_vel      = []
        self.peg_actual_yaw      = []
        self.peg_actual_yaw_rate = []

        # Riferimento peg — da /peg_ref_pose, /peg_ref_twist (admittance_planner)
        self.peg_ref_pos      = []
        self.peg_ref_yaw      = []
        self.peg_ref_vel      = []
        self.peg_ref_yaw_rate = []

        # ------------------------------------------------------------------ #
        #  Ultimi valori snapshot (aggiornati dalle callback, loggati al tick) #
        # ------------------------------------------------------------------ #
        self.last_pos          = [0.0, 0.0, 0.0]
        self.last_q            = [1.0, 0.0, 0.0, 0.0]
        self.last_rpy          = [0.0, 0.0, 0.0]
        self.last_v            = [0.0, 0.0, 0.0]
        self.last_omega        = [0.0, 0.0, 0.0]
        self.last_p_cam        = [0.0, 0.0, 0.0]
        self.last_actual_pov   = [0.0, 0.0, 0.0, 0.0]
        self.last_integral_action = [0.0, 0.0, 0.0]
        self.last_pref_pos     = [0.0, 0.0, 0.0]
        self.last_pref_rpy     = [0.0, 0.0, 0.0]
        self.last_pref_q       = [1.0, 0.0, 0.0, 0.0]
        self.last_vref         = [0.0, 0.0, 0.0]
        self.last_omegaref     = [0.0, 0.0, 0.0]
        self.last_optimal_wrench = [0.0, 0.0, 0.0, 0.0]
        self.last_w_target     = [0.0, 0.0, 0.0, 0.0]
        self.last_peg_pos      = [0.0, 0.0, 0.0]
        self.last_online_ref   = [0.0, 0.0, 0.0]
        self.last_haptic_force      = [0.0, 0.0, 0.0]
        self.last_peg_ext_force     = [0.0, 0.0, 0.0]
        self.last_estimated_wrench  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_delta_p           = [0.0, 0.0, 0.0]
        self.last_delta_p_sensor    = [0.0, 0.0, 0.0]
        self.last_peg_actual_pos      = [0.0, 0.0, 0.0]
        self.last_peg_actual_vel      = [0.0, 0.0, 0.0]
        self.last_peg_actual_yaw      = 0.0
        self.last_peg_actual_yaw_rate = 0.0
        self.last_peg_ref_pos      = [0.0, 0.0, 0.0]
        self.last_peg_ref_yaw      = 0.0
        self.last_peg_ref_vel      = [0.0, 0.0, 0.0]
        self.last_peg_ref_yaw_rate = 0.0

        # ------------------------------------------------------------------ #
        #  QoS                                                                 #
        # ------------------------------------------------------------------ #
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ------------------------------------------------------------------ #
        #  Subscriptions                                                        #
        # ------------------------------------------------------------------ #

        # Clock (rate-limiting unico — nessun dato estratto dall'odometria raw)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.cb_px4_odom_clock, px4_qos)

        # Stato drone ENU (da MPC — già trasformato)
        self.create_subscription(PoseStamped,      '/drone_pose',     self.cb_drone_pose,     10)
        self.create_subscription(Vector3,          '/drone_rpy',      self.cb_drone_rpy,      10)
        self.create_subscription(TwistStamped,     '/drone_velocity', self.cb_drone_velocity, 10)
        self.create_subscription(Float64MultiArray,'/actual_pov',     self.cb_actual_pov,     10)
        self.create_subscription(PoseStamped,   '/drone_cam_pose', self.cb_drone_cam_pose, 10)
        self.create_subscription(Vector3,          '/integral_action',self.cb_integral_action,10)

        # Riferimenti drone
        self.create_subscription(PoseStamped,      '/optimal_drone_pose',      self.cb_ref_pose,       10)
        # self.create_subscription(PoseStamped,      '/camera_ref_pose',         self.cb_ref_pose,       10)  # RIMOSSO: sovrascriveva pref_pos (riferimento drone) con il riferimento camera
        self.create_subscription(TwistStamped,     '/velocity_reference',      self.cb_ref_twist,      10)
        self.create_subscription(Wrench,           '/optimal_wrench',          self.cb_optimal_wrench,     10)
        self.create_subscription(Wrench,           '/wrench_reference',        self.cb_wrench_target,  10)
        self.create_subscription(Float64MultiArray,'/online_cylindrical_ref',  self.cb_online_ref,     10)

        # Posizione peg nel mondo (da MPC, già ENU)
        self.create_subscription(PoseStamped, '/peg_pose', self.cb_peg_pose, 10)

        # Stato attuale drone peg ENU (fake_publisher in sim / admittance_planner in real)
        self.create_subscription(PoseStamped,  '/peg_actual_pose',     self.cb_peg_actual_pose,     10)
        self.create_subscription(TwistStamped, '/peg_actual_velocity', self.cb_peg_actual_velocity, 10)
        self.create_subscription(Float64,      '/peg_actual_yaw',      self.cb_peg_actual_yaw,      10)
        self.create_subscription(Float64,      '/peg_actual_yaw_rate', self.cb_peg_actual_yaw_rate, 10)

        # Riferimento peg (admittance_planner)
        self.create_subscription(PoseStamped,  '/peg_ref_pose',  self.cb_peg_ref_pose,  10)
        self.create_subscription(TwistStamped, '/peg_ref_twist', self.cb_peg_ref_twist, 10)

        # Altro
        self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands',
                                 self.cb_haptic_force, 10)
        self.create_subscription(Wrench,        self.ft_topic,       self.cb_peg_ft,           10)
        self.create_subscription(Wrench,        '/estimated_wrench', self.cb_estimated_wrench, 10)
        self.create_subscription(Vector3Stamped,'/delta_p',          self.cb_delta_p,          10)
        self.create_subscription(Vector3Stamped,'/delta_p_sensor',   self.cb_delta_p_sensor,   10)

        # Trigger
        self.create_subscription(Bool, '/logging/start', self.cb_logging_start, qos_latched)
        self.create_subscription(Bool, '/mpc_task/start', self.cb_task_start,   qos_latched)

        self.get_logger().info(f'Logger avviato | Salva in: {self.final_save_path}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ================================================================== #
    #  Callbacks — stato drone ENU (direttamente dall'MPC)                #
    # ================================================================== #

    def cb_drone_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.last_pos = [p.x, p.y, p.z]
        # PoseStamped usa [x,y,z,w] — salviamo come [qw, qx, qy, qz]
        self.last_q = [o.w, o.x, o.y, o.z]

    def cb_drone_rpy(self, msg: Vector3):
        self.last_rpy = [msg.x, msg.y, msg.z]

    def cb_drone_velocity(self, msg: TwistStamped):
        l = msg.twist.linear
        a = msg.twist.angular
        self.last_v     = [l.x, l.y, l.z]
        self.last_omega = [a.x, a.y, a.z]

    def cb_actual_pov(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.last_actual_pov = list(msg.data[:4])  # [r_cyl, beta, z, yaw_err]

    def cb_integral_action(self, msg: Vector3):
        self.last_integral_action = [msg.x, msg.y, msg.z]

    def cb_drone_cam_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_p_cam = [p.x, p.y, p.z]

    # ================================================================== #
    #  Callbacks — riferimenti drone                                       #
    # ================================================================== #

    def cb_optimal_wrench(self, msg: Wrench):
        self.last_optimal_wrench = [msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_wrench_target(self, msg: Wrench):
        self.last_w_target = [msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

    def cb_haptic_force(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.last_haptic_force = [msg.data[0], msg.data[1], msg.data[2]]

    def cb_peg_ft(self, msg: Wrench):
        self.last_peg_ext_force = [msg.force.x, msg.force.y, msg.force.z]

    def cb_estimated_wrench(self, msg: Wrench):
        self.last_estimated_wrench = [
            msg.force.x,  msg.force.y,  msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z
        ]

    def cb_delta_p(self, msg: Vector3Stamped):
        self.last_delta_p = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_delta_p_sensor(self, msg: Vector3Stamped):
        self.last_delta_p_sensor = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_ref_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.last_pref_pos = [p.x, p.y, p.z]
        # RPY dal quaternione (già in ENU — semplice decomposizione, non trasformazione)
        self.last_pref_rpy = list(Rot.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz'))
        self.last_pref_q   = [o.w, o.x, o.y, o.z]
        self.t_ref.append(self.now_sec())

    def cb_ref_twist(self, msg: TwistStamped):
        l = msg.twist.linear
        a = msg.twist.angular
        self.last_vref     = [l.x, l.y, l.z]
        self.last_omegaref = [a.x, a.y, a.z]

    def cb_peg_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_peg_pos = [p.x, p.y, p.z]

    def cb_online_ref(self, msg: Float64MultiArray):
        self.last_online_ref = list(msg.data)[:3]

    # ================================================================== #
    #  Callbacks — stato attuale drone peg (ENU)                          #
    # ================================================================== #

    def cb_peg_actual_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_peg_actual_pos = [p.x, p.y, p.z]

    def cb_peg_actual_velocity(self, msg: TwistStamped):
        l = msg.twist.linear
        self.last_peg_actual_vel = [l.x, l.y, l.z]

    def cb_peg_actual_yaw(self, msg: Float64):
        self.last_peg_actual_yaw = msg.data

    def cb_peg_actual_yaw_rate(self, msg: Float64):
        self.last_peg_actual_yaw_rate = msg.data

    # ================================================================== #
    #  Callbacks — riferimento peg (admittance_planner)                   #
    # ================================================================== #

    def cb_peg_ref_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.last_peg_ref_pos = [p.x, p.y, p.z]
        # Estrazione yaw dal quaternione ENU (già nel frame corretto)
        if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) > 1e-6:
            self.last_peg_ref_yaw = float(Rot.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz')[2])

    def cb_peg_ref_twist(self, msg: TwistStamped):
        l = msg.twist.linear
        self.last_peg_ref_vel      = [l.x, l.y, l.z]
        self.last_peg_ref_yaw_rate = float(msg.twist.angular.z)

    # ================================================================== #
    #  Trigger                                                             #
    # ================================================================== #

    def cb_logging_start(self, msg: Bool):
        if msg.data and not self.logging_enabled:
            self.logging_enabled = True
            self.get_logger().info('Logging AVVIATO (segnale /logging/start ricevuto).')

    def cb_task_start(self, msg: Bool):
        if msg.data and self.task_start_time is None:
            self.task_start_time = self.now_sec()
            self.get_logger().info('Ricevuto start task, salvo timestamp.')

    # ================================================================== #
    #  Clock — /fmu/out/vehicle_odometry usato solo per il rate-limiting  #
    # ================================================================== #

    def cb_px4_odom_clock(self, _msg: VehicleOdometry):
        if not self.logging_enabled:
            return
        t_now = self.now_sec()
        if self.last_log_time is not None and (t_now - self.last_log_time < self.log_dt):
            return

        # Snapshot di tutti gli ultimi valori
        self.t.append(t_now)
        self.pos.append(list(self.last_pos))
        self.q.append(list(self.last_q))
        self.rpy.append(list(self.last_rpy))
        self.v.append(list(self.last_v))
        self.omega.append(list(self.last_omega))
        self.p_cam.append(list(self.last_p_cam))
        self.actual_pov.append(list(self.last_actual_pov))
        self.integral_action.append(list(self.last_integral_action))
        self.pref_pos.append(list(self.last_pref_pos))
        self.pref_rpy.append(list(self.last_pref_rpy))
        self.pref_q.append(list(self.last_pref_q))
        self.vref.append(list(self.last_vref))
        self.omegaref.append(list(self.last_omegaref))
        self.optimal_wrench.append(list(self.last_optimal_wrench))
        self.wrench_target.append(list(self.last_w_target))
        self.peg_pos.append(list(self.last_peg_pos))
        self.online_ref.append(list(self.last_online_ref))
        self.online_cyl_ref.append(list(self.last_online_ref))   # alias
        self.haptic_force.append(list(self.last_haptic_force))
        self.peg_ext_force.append(list(self.last_peg_ext_force))
        self.estimated_wrench.append(list(self.last_estimated_wrench))
        self.delta_p.append(list(self.last_delta_p))
        self.delta_p_sensor.append(list(self.last_delta_p_sensor))
        self.peg_actual_pos.append(list(self.last_peg_actual_pos))
        self.peg_actual_vel.append(list(self.last_peg_actual_vel))
        self.peg_actual_yaw.append(self.last_peg_actual_yaw)
        self.peg_actual_yaw_rate.append(self.last_peg_actual_yaw_rate)
        self.peg_ref_pos.append(list(self.last_peg_ref_pos))
        self.peg_ref_yaw.append(self.last_peg_ref_yaw)
        self.peg_ref_vel.append(list(self.last_peg_ref_vel))
        self.peg_ref_yaw_rate.append(self.last_peg_ref_yaw_rate)

        self.last_log_time = t_now

    # ================================================================== #
    #  Save                                                                #
    # ================================================================== #

    def save(self):
        T = np.asarray(self.t)
        if not T.size:
            self.get_logger().warn("Nessun dato loggato, salvataggio annullato.")
            return
        T_rel      = T - T[0]
        t_start_rel = (self.task_start_time - T[0]) if self.task_start_time else -1.0

        # Tutti gli array sono già in ENU/FLU — nessuna trasformazione necessaria
        pos    = np.asarray(self.pos)
        q      = np.asarray(self.q)
        rpy    = np.asarray(self.rpy)
        v      = np.asarray(self.v)
        omega  = np.asarray(self.omega)
        p_cam  = np.asarray(self.p_cam)
        actual_pov = np.asarray(self.actual_pov)  # [r_cyl, beta_cyl, z_cyl, yaw_err_cyl]

        # ---- Derivate numeriche (unici calcoli qui) ----
        acc     = np.zeros_like(v)
        ang_acc = np.zeros_like(omega)
        jerk    = np.zeros_like(v)
        snap    = np.zeros_like(v)
        if len(T_rel) > 1:
            for i in range(3):
                acc[:, i]     = np.gradient(v[:, i],     T_rel)
                ang_acc[:, i] = np.gradient(omega[:, i], T_rel)
                jerk[:, i]    = np.gradient(acc[:, i],   T_rel)
                snap[:, i]    = np.gradient(jerk[:, i],  T_rel)

        # ---- Coordinate cilindriche (direttamente dall'MPC via /actual_pov) ----
        r_cyl       = actual_pov[:, 0]
        beta_cyl    = actual_pov[:, 1]
        z_cyl       = actual_pov[:, 2]
        yaw_err_cyl = actual_pov[:, 3]

        # ---- Target cartesiano telecamera (geometria semplice da dati già loggati) ----
        online_cyl_ref = np.asarray(self.online_cyl_ref)
        peg_pos_arr    = np.asarray(self.peg_pos)
        p_cam_target   = cylindrical_to_cartesian(online_cyl_ref, p_origin=peg_pos_arr)

        out = dict(
            t=T_rel, t_ref=np.asarray(self.t_ref),
            pos=pos, rpy=rpy, q=q,
            v=v, omega=omega,
            pref_pos=np.asarray(self.pref_pos),
            pref_rpy=np.asarray(self.pref_rpy),
            pref_q=np.asarray(self.pref_q),
            vref=np.asarray(self.vref),
            omegaref=np.asarray(self.omegaref),
            optimal_wrench=np.asarray(self.optimal_wrench),
            wrench_target=np.asarray(self.wrench_target),
            haptic_force=np.asarray(self.haptic_force),
            peg_pos=peg_pos_arr,
            online_ref=np.asarray(self.online_ref),
            online_cyl_ref=online_cyl_ref,
            p_cam=p_cam,
            p_cam_target=p_cam_target,
            # PoV cilindrico attuale (da MPC /actual_pov)
            r_cyl=r_cyl, beta_cyl=beta_cyl, z_cyl=z_cyl, yaw_err_cyl=yaw_err_cyl,
            integral_action=np.asarray(self.integral_action),
            peg_ext_force=np.asarray(self.peg_ext_force),
            estimated_wrench=np.asarray(self.estimated_wrench),
            delta_p=np.asarray(self.delta_p),
            delta_p_sensor=np.asarray(self.delta_p_sensor),
            # Stato drone peg ENU (fake_publisher in sim / admittance_planner in real)
            peg_actual_pos=np.asarray(self.peg_actual_pos),
            peg_actual_vel=np.asarray(self.peg_actual_vel),
            peg_actual_yaw=np.asarray(self.peg_actual_yaw),
            peg_actual_yaw_rate=np.asarray(self.peg_actual_yaw_rate),
            # Riferimento peg
            peg_ref_pos=np.asarray(self.peg_ref_pos),
            peg_ref_yaw=np.asarray(self.peg_ref_yaw),
            peg_ref_vel=np.asarray(self.peg_ref_vel),
            peg_ref_yaw_rate=np.asarray(self.peg_ref_yaw_rate),
            # Derivate numeriche
            acc=acc, ang_acc=ang_acc, jerk=jerk, snap=snap,
            mass=self.mass,
            task_start_time=np.array([t_start_rel])
        )

        import os
        import subprocess

        if not os.path.exists(self.out_dir):
            os.makedirs(self.out_dir, exist_ok=True)

        from scipy.io import savemat
        savemat(self.final_save_path, out)
        self.get_logger().info(
            f"Salvataggio completato in {self.final_save_path}. Elaborati {len(T)} campioni."
        )

        # Generazione automatica dei grafici
        plot_script_path = '/root/my_ros2_ws/src/gcs_pkg/gcs_pkg/plot_script.py'
        if os.path.exists(plot_script_path):
            self.get_logger().info(f"Avvio autogenerazione grafici in {self.out_dir}...")
            try:
                subprocess.Popen(['python3', plot_script_path, '--log', self.final_save_path, '--save', '--out-dir', self.out_dir, '--formats', 'png'])
            except Exception as e:
                self.get_logger().error(f"Errore durante l'avvio del plot_script: {e}")


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
