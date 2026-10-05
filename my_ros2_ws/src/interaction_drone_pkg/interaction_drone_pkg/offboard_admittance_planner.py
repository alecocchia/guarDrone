#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offboard_admittance_planner.py — Planner traiettoria con controllo di ammettenza per il drone peg.

Comportamento:
  - FREE-FLIGHT (|F_ext| < F_threshold): identico a OffboardTrajectoryPlanner
    → traiettoria trapezoidale verso il target, setpoint di posizione a PX4.
  - CONTACT (|F_ext| >= F_threshold): controllo di ammettenza
    → la forza esterna modifica il target di posizione nominale secondo
      la dinamica virtuale M·Δp̈ + D·Δṗ + K·Δp = R_enu * F_sensor_enu

Frame del sensore FT (da SDF):
  Il sensore è montato su 'end_eff_sens' con pose pitch=-1.57 rad rispetto al modello.
  Gz Sim ForceTorque (default): misura nel frame del child link (end_eff_sens),
  convenzione child→parent (forza che il contatto esercita sull'end-effector, espressa
  nel frame del sensore).

  Per l'ammettenza usiamo solo la forza lineare sull'asse Z del sensore (assiale):
    R_sensor_to_body = Rz(0) · Ry(+π/2) → frame sensore → body FLU
    F_enu = R_flu2enu · R_sensor_to_body · F_sensor

Parametri configurabili (launch / ros2 param):
  F_threshold   [N]   soglia di attivazione ammettenza (default 0.5)
  adm_mass      [kg]  massa virtuale (default 1.0)
  adm_damping   [-]   smorzamento virtuale (default 8.0)
  adm_stiffness [-]   rigidezza virtuale (default 0.0, puro ammortizzatore)
  adm_max_delta [m]   saturazione dello spostamento di ammettenza (default 0.3)
  ft_topic      str   topic del sensore FT (default stringa vuota → usa _0)
"""
##################################
# PROBLEMA ATTUALE: QUANDO LA FORZA SI ANNULLA (PERCHÉ NON VIENE RILEVATO CONTATTO (F_SENS < f_THRESHOLD))
# IL TARGET DEL DRONE TORNA AD ESSERE QUELLO PRE-CONTATTO; E MENTRE SULL'ASSE Z DEL SENSORE È GIUSTO CHE SIA COSÌ, 
# SUGLI ASSI X ED Y QUESTO PROVOCA OSCILLAZIONI CONTINUE PER IL DRONE SU E GIU O DESTRA E SINISTRA SE IL TARGET
# PRE-CONTATTO SI TROVA NON PERPENDICOLARE ALLA PARETE
# IDEA: QUANDO VIENE RILEVATO CONTATTO, IL TARGET DEL DRONE SU X ED Y DIVENTA LA POSIZIONE ATTUALE, MENTRE SU Z
# AGISCE DAVVERO L'IMPEDENZA
##################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PoseStamped, Wrench, Vector3Stamped, TwistStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
import numpy as np
import math
from scipy.spatial.transform import Rotation

from utils_pkg.planner import generate_trapezoidal_trajectory
from utils_pkg.utils_np import min_angle

from collections import deque


# -- Frame: end_eff_sens → body FLU -------------------------------------------
# end_eff_sens ha pitch = -π/2 rispetto al body frame (FLU) 
# Gz FT default: misure nel frame child, verso parent (convenzione: forza che il
# contatto esercita sull'end-effector, nel frame child = sensore).

_R_SENSOR_TO_BODY = Rotation.from_euler('y', -np.pi / 2.0).as_matrix()
_R_BODY_TO_SENSOR = _R_SENSOR_TO_BODY.T
# -- Conversione NED → ENU (stessa usata in offboard_trajectory_planner) ------
_M_NED2ENU = np.array([[0., 1., 0 ],
                        [1., 0., 0.],
                        [0., 0., -1.]])

# -- Conversione FRD → FLU ----------------------------------------------------
_M_FRD2FLU = np.array([[1., 0., 0.],
                        [0., -1., 0.],
                        [0., 0., -1.]])


class OffboardAdmittancePlanner(Node):
    """
    Planner traiettoria per il drone peg con controllo di ammettenza.

    In free-flight (|F_ext| < F_threshold) si comporta come OffboardTrajectoryPlanner.
    In contatto (|F_ext| >= F_threshold) integra una dinamica virtuale di ammettenza
    per modificare il target di posizione inviato a PX4.
    """

    def __init__(self):
        super().__init__('offboard_admittance_planner')

        # -- Parametri planner base --
        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('v_takeoff_max', 0.1)
        self.declare_parameter('a_takeoff_max', 0.2)
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('a_max', 0.3)
        self.declare_parameter('dt', 0.01)   # 100 Hz

        # -- Parametri ammettenza --
        self.declare_parameter('F_threshold', 0.2)        # [N] soglia attivazione forza
        self.declare_parameter('adm_max_delta', 10.0)      # [m] saturazione spostamento traslazionale
        self.declare_parameter('tau_threshold', 0.002)    # [Nm] soglia attivazione coppia yaw

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value

        # -- Topic FT sensor --
        # Viene passato come parametro dal launch file.
        self.declare_parameter(
            'ft_topic',
            '/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque'
        )

        # -- Lettura parametri --
        ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        self.v_takeoff_max = self.get_parameter('v_takeoff_max').get_parameter_value().double_value
        self.a_takeoff_max = self.get_parameter('a_takeoff_max').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.a_max = self.get_parameter('a_max').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Parametri sensore di forza e coppia
        self.F_threshold = self.get_parameter('F_threshold').get_parameter_value().double_value
        self.tau_threshold = self.get_parameter('tau_threshold').get_parameter_value().double_value

        # -- Dimensionamento ammettenza in frame SENSOR --
        #
        # Il filtro è M * delta_ddot + D * delta_dot + K * delta = F_ext  (un sistema 2° ordine per asse).
        #
        # Parametri LIBERI:
        #   F_typ    : forza di contatto tipica attesa [N]
        #   delta_typ: spostamento desiderato a quella forza [m]  → K = F_typ / delta_typ
        #   Ta       : tempo di assestamento al 5% [s]            → wn = 3 / Ta
        #   zeta     : rapporto di smorzamento (1=critico, <1=oscillante, >1=sovrasmorzato)
        #
        # Parametri DERIVATI:
        #   M = K / wn^2                         (massa virtuale)
        #   D = 2 * zeta * sqrt(K*M)             (smorzamento)
        #
        # --- ASSE Z TRASLAZIONALE (perpendicolare alla parete) 
        F_typ_z    = 3.0    # [N]  forza di contatto
        delta_typ_z= 0.08   # [m]  rimbalzo desiderato a F_typ_z (--> rigidezza K)
        Ta_z       = 0.5    # [s]  tempo assestamento al 5%
        zeta_z     = 1.0    # [-]  smorzamento

        K = F_typ_z / delta_typ_z
        Ta   = Ta_z
        zeta = zeta_z
        wn   = 3.0 / (zeta * Ta)        # wn tale che assestamento 5% = Ta
        M    = K / (wn**2)                # M = K/wn²
        D    = 2.0 * zeta * K / wn      # D = 2*zeta*K/wn

        self.adm_K = K
        self.adm_M = M
        self.adm_D = D
        self.adm_max_delta = self.get_parameter('adm_max_delta').get_parameter_value().double_value

        # --- ASSE ROTAZIONALE YAW (K_psi > 0 per ritorno a perpendicolare alla parete) ---
        tau_typ_yaw   = 0.05              # [Nm] coppia tipica attesa a contatto
        delta_typ_yaw = float(np.radians(3.0))  # [rad] deflessione tipica a tau_typ_yaw (~5 deg)
        Ta_yaw        = 0.5              # [s] tempo assestamento al 5%
        zeta_yaw      = 1              # [-] smorzamento critico

        K_psi  = tau_typ_yaw / delta_typ_yaw
        wn_psi = 3.0 / (zeta_yaw * Ta_yaw)
        J_psi  = K_psi / (wn_psi ** 2)
        D_psi  = 2.0 * zeta_yaw * K_psi / wn_psi

        self.adm_K_psi = K_psi
        self.adm_J_psi = J_psi
        self.adm_D_psi = D_psi
        self.adm_max_delta_yaw = 0.35  # [rad] saturazione rotazione (~20 deg)

        self.get_logger().info(
            f"[Admittance Trans] K={self.adm_K:.2f}, M={self.adm_M:.3f}, D={self.adm_D:.2f}, "
            f"wn={wn:.2f} rad/s, Ta={Ta}s, zeta={zeta}"
        )
        self.get_logger().info(
            f"[Admittance Yaw] K_psi={self.adm_K_psi:.4f} Nm/rad, J_psi={self.adm_J_psi:.4f} kg*m^2, "
            f"D_psi={self.adm_D_psi:.4f} Nms/rad, wn={wn_psi:.2f} rad/s, Ta={Ta_yaw}s, zeta={zeta_yaw}"
        )

        ft_topic = self.get_parameter('ft_topic').get_parameter_value().string_value

        # -- QoS --
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_ft = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        prefix = f'/{ns}' if ns else ''

        # -- Publishers --
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', 1)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', 1)
        self.delta_p_pub         = self.create_publisher(Vector3Stamped, '/delta_p', 10)
        self.delta_p_sensor_pub   = self.create_publisher(Vector3Stamped, '/delta_p_sensor', 10)
        self.delta_yaw_pub        = self.create_publisher(Float64,        '/delta_yaw', 10)
        self.peg_ref_pub = self.create_publisher(PoseStamped, '/peg_ref_pose', 10)
        self.peg_ref_twist_pub = self.create_publisher(TwistStamped, '/peg_ref_twist', 10)

        # Stato attuale drone peg (ENU) — per logger
        self.peg_actual_pose_pub     = self.create_publisher(PoseStamped,  '/peg_actual_pose',     10)
        self.peg_actual_vel_pub      = self.create_publisher(TwistStamped, '/peg_actual_velocity', 10)
        self.peg_actual_yaw_pub      = self.create_publisher(Float64,      '/peg_actual_yaw',      10)
        self.peg_actual_omega_z_pub  = self.create_publisher(Float64,      '/peg_actual_omega_z',  10)

        # -- Subscribers --
        self.odom_sub = self.create_subscription(
            VehicleOdometry, f'{prefix}/fmu/out/vehicle_odometry',
            self.odom_cb, qos_px4)
        self.target_sub = self.create_subscription(
            PoseStamped, 'target_pose', self.target_cb, 10)
        self.enabled_sub = self.create_subscription(
            Bool, 'offboard_traj_enabled', self.enabled_cb, 10)
        self.ft_sub = self.create_subscription(
            Wrench, ft_topic, self.ft_cb, qos_ft)
        # Subscriber live haptic (bypassa il generatore di traiettoria)
        self.live_pose_sub = self.create_subscription(
            PoseStamped, '/peg_live_pose', self.live_target_cb, 10)

        # Subscriber stato FSM supervisore per rilevare fase di takeoff
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.supervisor_state_sub = self.create_subscription(
            String, '/supervisor/state', self.supervisor_state_cb, qos_latched)

        # -- Stato interno (traiettoria) --
        self.current_pos = np.zeros(3)   # ENU + spawn offset
        self.current_rpy = np.zeros(3)
        self.R_flu2enu = np.eye(3)       # rotazione corrente body FLU --> ENU
        self.has_odom = False
        self.offboard_traj_enabled = True
        self.current_phase = 'TAKEOFF_MONITOR'
        self._received_supervisor_state = False

        self.traj_p = None
        self.traj_rpy = None
        self.current_index = 0

        # -- Stato ammettenza traslazionale (terna SENSORE) --
        self.delta_p_s = 0.0   # [m]    in sensor frame
        self.delta_v_s = 0.0   # [m/s]  in sensor frame
        self.delta_p   = np.zeros(3)   # [m]    in ENU  (= R_s2e[:,2].T * delta_p_s)
        self.delta_v   = np.zeros(3)   # [m/s]  in ENU  (= R_s2e[:,2].T * delta_v_s)

        # Forza esterna nel frame SENSOR (aggiornata dalla callback FT)
        self.F_ext_sens = 0.0
        # Input nell'admittance control
        self.F_adm_input = 0.0
        self.admittance_active = False
        self.Fz_prev = 0.0

        # -- Stato ammettenza rotazionale yaw --
        self.delta_p_yaw = 0.0    # [rad] deviazione yaw dall'ammettenza
        self.delta_v_yaw = 0.0    # [rad/s] velocità angolare di ammettenza
        self.tau_ext_sens = 0.0   # [Nm] coppia filtrata (attorno a X sensore = Z drone)
        self.tau_adm_input = 0.0  # [Nm] ingresso ammettenza yaw
        self.tau_x_prev = 0.0     # [Nm] precedente lettura per filtro IIR

        self.p_contact   = None  # Posizione congelata al momento del contatto
        self.yaw_contact = None  # Yaw perpendicolare alla parete, congelato al primo contatto


        # Stato live haptic
        self.live_target_pos = None
        self.live_target_yaw = 0.0
        self.live_mode = False
        self.live_mode_stamp = None

        # -- Timer principale --
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f"[AdmittancePlanner] Avviato. ns={ns!r}, F_thr={self.F_threshold:.2f}N, "
            f"dt={self.dt:.3f}s, ft_topic={ft_topic!r}"
        )

    # -- Callbacks --

    def odom_cb(self, msg: VehicleOdometry):
        """Odometria PX4 (NED, FRD) --> stato interno (ENU, FLU)."""
        q_scipy = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]  # [x,y,z,w]
        R_frd2ned = Rotation.from_quat(q_scipy).as_matrix()
        R_flu2enu = _M_NED2ENU @ R_frd2ned @ _M_FRD2FLU

        # Posizione: NED --> ENU + spawn offset
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        pos_enu = _M_NED2ENU @ pos_ned
        self.current_pos[0] = pos_enu[0] + self.start_x
        self.current_pos[1] = pos_enu[1] + self.start_y
        self.current_pos[2] = pos_enu[2] + self.start_z

        # Orientamento
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        self.current_rpy[:] = rot_flu2enu.as_euler('xyz')
        self.R_flu2enu = R_flu2enu

        self.has_odom = True

        # -- Pubblicazione dello stato reale del peg in ENU (per logger) --
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(self.current_pos[0])
        pose_msg.pose.position.y = float(self.current_pos[1])
        pose_msg.pose.position.z = float(self.current_pos[2])
        q_quat = rot_flu2enu.as_quat()
        pose_msg.pose.orientation.x = float(q_quat[0])
        pose_msg.pose.orientation.y = float(q_quat[1])
        pose_msg.pose.orientation.z = float(q_quat[2])
        pose_msg.pose.orientation.w = float(q_quat[3])
        self.peg_actual_pose_pub.publish(pose_msg)

        vel_ned = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        vel_enu = _M_NED2ENU @ vel_ned
        omega_frd = np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]])
        omega_flu = _M_FRD2FLU @ omega_frd

        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'world'
        vel_msg.twist.linear.x = float(vel_enu[0])
        vel_msg.twist.linear.y = float(vel_enu[1])
        vel_msg.twist.linear.z = float(vel_enu[2])
        vel_msg.twist.angular.x = float(omega_flu[0])
        vel_msg.twist.angular.y = float(omega_flu[1])
        vel_msg.twist.angular.z = float(omega_flu[2])
        self.peg_actual_vel_pub.publish(vel_msg)

        yaw_msg = Float64()
        yaw_msg.data = float(self.current_rpy[2])
        self.peg_actual_yaw_pub.publish(yaw_msg)

        omega_z_msg = Float64()
        omega_z_msg.data = float(omega_flu[2])
        self.peg_actual_omega_z_pub.publish(omega_z_msg)

    def ft_cb(self, msg: Wrench):
        """
        Misura FT sensor -> forza esterna in ENU.

        Gz Sim ForceTorque:
          - Misure nel frame del child link (end_eff_sens)
          - end_eff_sens ha pitch = -π/2 rispetto al body FLU
          - Per passare al body FLU: R_SENSOR_TO_BODY = Ry(+π/2)
          - Per passare all'ENU: R_flu2enu (dall'odometria corrente)

        """
        F_sensor = msg.force.z
        alpha = 0.4
        self.F_ext_sens = alpha * F_sensor + (1 - alpha) * self.Fz_prev
        self.Fz_prev = self.F_ext_sens
        F_norm = np.abs(self.F_ext_sens)

        # Lettura e filtraggio coppia attorno all'asse X del sensore (= asse Z verticale / yaw del drone)
        tau_raw = msg.torque.x
        alpha_tau = 0.2
        self.tau_ext_sens = alpha_tau * tau_raw + (1.0 - alpha_tau) * self.tau_x_prev
        self.tau_x_prev = self.tau_ext_sens
        tau_norm = np.abs(self.tau_ext_sens)

        was_active = self.admittance_active
        # Attiviamo l'ammettenza traslazionale solo se superiamo la soglia E siamo sopra i 30 cm dalla quota di spawn
        alt_rel = self.current_pos[2] - self.start_z
        self.admittance_active = (F_norm >= self.F_threshold and alt_rel >= 0.3)

        # Se l'ammettenza traslazionale NON deve agire, azzeriamo l'input di forza
        if self.admittance_active:
            self.F_adm_input = self.F_ext_sens
        else:
            self.F_adm_input = 0.0

        # Input ammettenza rotazionale yaw: attivo durante contatto o se la coppia supera la soglia in volo
        if (self.admittance_active or tau_norm >= self.tau_threshold) and alt_rel >= 0.3:
            self.tau_adm_input = self.tau_ext_sens
        else:
            self.tau_adm_input = 0.0

        if self.admittance_active and not was_active:
            # Memorizza la posizione corrente al primo contatto dopo free-flight (fronte di salita).
            self.p_contact = self.current_pos.copy()

            # Yaw perpendicolare alla parete: n = R_sensor2enu[:,2] è il vettore che 
            # punta VERSO il drone
            # (F>0 = parete spinge il drone in direzione +n), quindi la parete
            # è nella direzione -n → il drone deve guardare verso -n.
            R_s2enu = self.R_flu2enu @ _R_SENSOR_TO_BODY
            n_contact = R_s2enu[:, 2]                        # normale parete in ENU
            n_h = np.array([n_contact[0], n_contact[1], 0.0])
            n_h_norm = np.linalg.norm(n_h)
            if n_h_norm > 1e-3:                            # parete non orizzontale
                self.yaw_contact = np.arctan2(-n_h[1], -n_h[0])
            else:
                self.yaw_contact = self.current_rpy[2]     # fallback: mantieni yaw attuale

            self.get_logger().info(
                f"[AdmittancePlanner] CONTATTO rilevato: |F|={F_norm:.3f}N >= {self.F_threshold:.2f}N"
                f" | |tau_yaw|={tau_norm:.4f}Nm | p_contact={self.p_contact} | yaw_contact={np.degrees(self.yaw_contact):.1f}°"
            )
        elif not self.admittance_active and was_active:
            self.get_logger().info(
                "[AdmittancePlanner] Contatto perso. Ritorno a free-flight lungo la normale al contatto."
            )

    def supervisor_state_cb(self, msg: String):
        self.current_phase = msg.data
        self._received_supervisor_state = True
        if msg.data in ('LANDING', 'DISARM_WAIT', 'EMERGENCY', 'MISSION_COMPLETE'):
            self.offboard_traj_enabled = False
            self.get_logger().info(f"[AdmittancePlanner] Ricevuto stato '{msg.data}': disabilito setpoint offboard.")

    def enabled_cb(self, msg: Bool):
        self.offboard_traj_enabled = msg.data
        if not self.offboard_traj_enabled:
            self.get_logger().info("[AdmittancePlanner] DISABILITATO.")

    def live_target_cb(self, msg: PoseStamped):
        """Haptic live teleop: aggiorna il target direttamente, bypass traiettoria trapezoidale."""
        self.live_target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.live_target_yaw = Rotation.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]).as_euler('xyz')[2]
        self.live_mode_stamp = self.get_clock().now()
        if not self.live_mode:
            self.live_mode = True
            self.get_logger().info("[AdmittancePlanner] HAPTIC live mode ATTIVO")

    def target_cb(self, msg: PoseStamped):
        """Ricezione nuovo target --> (ri)calcolo traiettoria nominale."""

        if not self.has_odom:
            self.get_logger().warn("[AdmittancePlanner] Target ricevuto, odometria non ancora valida. Ignoro.")
            return

        # Annullo la variabile di primo contatto
        self.p_contact   = None
        self.yaw_contact = None

        t_x = msg.pose.position.x
        t_y = msg.pose.position.y
        t_z = msg.pose.position.z

        if (msg.pose.orientation.w == 0.0 and msg.pose.orientation.x == 0.0
                and msg.pose.orientation.y == 0.0 and msg.pose.orientation.z == 0.0):
            t_yaw = self.current_rpy[2]
        else:
            t_yaw = Rotation.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]).as_euler('xyz')[2]

        x0 = [self.current_pos[0], self.current_pos[1], self.current_pos[2],
               0.0, 0.0, self.current_rpy[2]]
        x_ref = [t_x, t_y, t_z, 0.0, 0.0, t_yaw]

        is_takeoff = (self.current_phase in ['WAIT_START', 'ARM_OFFBOARD', 'TAKEOFF_MONITOR', 'WAIT_EKF', 'TAKEOFF']
                      or 'TAKEOFF' in self.current_phase.upper())
        v_limit = self.v_takeoff_max if is_takeoff else self.v_max
        a_limit = self.a_takeoff_max if is_takeoff else self.a_max

        self.get_logger().info(
            f"[AdmittancePlanner] Nuova traiettoria da {x0[:3]} a {x_ref[:3]} "
            f"(fase={self.current_phase}, v_limit={v_limit}, a_limit={a_limit})"
        )

        t_vec, p_vals, rpy_vals = generate_trapezoidal_trajectory(
            x0, x_ref, dt=self.dt, v_max=v_limit, a_max=a_limit
        )

        if not self._received_supervisor_state:
            self.current_phase = 'MISSION'

        self.traj_p = p_vals
        self.traj_rpy = rpy_vals
        self.current_index = 0

        self.get_logger().info(
            f"[AdmittancePlanner] Traiettoria calcolata: {len(self.traj_p)} punti, "
            f"durata={t_vec[-1]:.2f}s"
        )

    # -- Loop principale --

    def timer_cb(self):
        if not self.offboard_traj_enabled:
            return

        # -- Pubblica sempre OffboardControlMode --
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(ocm)

        # -- Ammettenza (integrazione in terna sensore, output in ENU) --
        R_sensor2enu = self.R_flu2enu @ _R_SENSOR_TO_BODY
        self._integrate_admittance(R_sensor2enu)
        # delta_p e delta_v sono in ENU (= R_s2e @ delta_p_s)
        delta_p_enu = self.delta_p
        delta_v_enu = self.delta_v
        #delta_v_enu = np.array([0,0,0])
        # delta_p_s è già in terna sensore, usato per il topic /delta_p_sensor
        self._delta_p_sensor = self.delta_p_s  # alias

        # -- MODALITÀ LIVE HAPTIC --
        HAPTIC_TIMEOUT = 0.5  # s: dopo questo tempo senza messaggi, mantieni ultima posizione
        if self.live_mode and self.live_target_pos is not None:
            elapsed = (self.get_clock().now() - self.live_mode_stamp).nanoseconds / 1e9
            if elapsed > HAPTIC_TIMEOUT:
                # Timeout: esci dal live mode, congela il target haptic come nuova traiettoria
                self.live_mode = False
                self.traj_p = [self.live_target_pos.copy()]
                self.traj_rpy = [np.array([0.0, 0.0, self.live_target_yaw])]
                self.current_index = 0
                self.get_logger().info("[AdmittancePlanner] Live mode TERMINATO - mantengo posizione haptic")
            else:
                # Setpoint diretto senza traiettoria (con ammettenza traslazionale + rotazionale)
                p_cmd = self.live_target_pos + delta_p_enu
                yaw_cmd = self.live_target_yaw + self.delta_p_yaw
                self.publish_setpoint(p_cmd, yaw_cmd, delta_v_enu)
                self._publish_deltas()
                return

        # -- Posizione e velocità nominali dalla traiettoria --
        if self.traj_p is None:
            if self.has_odom:
                self.publish_setpoint(self.current_pos, self.current_rpy[2])
            self._publish_deltas()
            return

        idx = min(self.current_index, len(self.traj_p) - 1)
        p_nom = self.traj_p[idx]
        yaw_nom = self.traj_rpy[idx][2]
        # Velocità nominale: differenza finita in ENU
        idx_next = min(idx + 1, len(self.traj_p) - 1)
        v_nom = (self.traj_p[idx_next] - p_nom) / self.dt          # [m/s] ENU
        dyaw = min_angle(self.traj_rpy[idx_next][2] - yaw_nom)
        omega_z_nom = dyaw / self.dt                                # [rad/s]


        # -- Composizione setpoint finale (delta_p/v già calcolati sopra) --
        if self.p_contact is not None:
            # Contatto avvenuto almeno una volta dal setpoint di posizione
            # - Normale:     segue p_nom_normal + compliance ammettenza (delta_p_enu)
            # - Tangenziale: congelata a p_contact (anche dopo il rilascio, delta_p_enu → 0)
            n = R_sensor2enu[:, 2]
            p_contact_normal     = np.dot(self.p_contact, n) * n
            p_contact_tangential = self.p_contact - p_contact_normal
            p_nom_normal         = np.dot(p_nom, n) * n
            v_nom_normal         = np.dot(v_nom, n) * n

            p_cmd = p_nom_normal + p_contact_tangential + delta_p_enu
            v_cmd = v_nom_normal + delta_v_enu   # tangenziale = 0, normale = feedforward
        else:
            p_cmd = p_nom + delta_p_enu
            v_cmd = v_nom + delta_v_enu

        # Yaw: nominale + deviazione di ammettenza rotazionale (K_psi > 0 riporta a 0 a coppia nulla)
        yaw_cmd = yaw_nom + self.delta_p_yaw
        self.publish_setpoint(p_cmd, yaw_cmd, v_cmd)

        # -- Pubblica delta topics (per logging e RViz) --
        self._publish_deltas()

        # -- Pubblica posizione + yaw di riferimento nominale peg in ENU (per logger) --
        ref_msg = PoseStamped()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'map'
        ref_msg.pose.position.x = float(p_nom[0])
        ref_msg.pose.position.y = float(p_nom[1])
        ref_msg.pose.position.z = float(p_nom[2])
        # Codifica yaw_nom come quaternione Rz(yaw_nom) [ENU]
        q_yaw = Rotation.from_euler('z', yaw_nom).as_quat()  # [qx, qy, qz, qw]
        ref_msg.pose.orientation.x = float(q_yaw[0])
        ref_msg.pose.orientation.y = float(q_yaw[1])
        ref_msg.pose.orientation.z = float(q_yaw[2])
        ref_msg.pose.orientation.w = float(q_yaw[3])
        self.peg_ref_pub.publish(ref_msg)

        # -- Pubblica velocità + omega_z nominali (per logger) --
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'map'
        twist_msg.twist.linear.x  = float(v_nom[0])
        twist_msg.twist.linear.y  = float(v_nom[1])
        twist_msg.twist.linear.z  = float(v_nom[2])
        twist_msg.twist.angular.z = float(omega_z_nom)
        self.peg_ref_twist_pub.publish(twist_msg)

        # Avanza l'indice della traiettoria nominale
        if self.current_index < len(self.traj_p):
            self.current_index += 1

    def _publish_deltas(self):
        """Pubblica i dislocamenti di ammettenza traslazionali e rotazionali per il logger."""
        stamp = self.get_clock().now().to_msg()

        # delta_p in ENU
        dp_msg = Vector3Stamped()
        dp_msg.header.stamp = stamp
        dp_msg.vector.x = float(self.delta_p[0])
        dp_msg.vector.y = float(self.delta_p[1])
        dp_msg.vector.z = float(self.delta_p[2])
        self.delta_p_pub.publish(dp_msg)

        # delta_p in terna SENSORE
        dp_s_msg = Vector3Stamped()
        dp_s_msg.header.stamp = stamp
        dp_s_msg.vector.x = 0.0
        dp_s_msg.vector.y = 0.0
        dp_s_msg.vector.z = float(self._delta_p_sensor)
        self.delta_p_sensor_pub.publish(dp_s_msg)

        # delta_yaw
        dy_msg = Float64()
        dy_msg.data = float(self.delta_p_yaw)
        self.delta_yaw_pub.publish(dy_msg)

    # -- Integrazione ammettenza --

    def _integrate_admittance(self, R_sensor2enu: np.ndarray):
        """
        Integra la dinamica virtuale di ammettenza in TERNA SENSORE.

        - Traslazionale (asse Z sensore): M * ddot(dp) + D * dot(dp) + K * dp = F_adm_input
        - Rotazionale (yaw):             J * ddot(dyaw) + D_psi * dot(dyaw) + K_psi * dyaw = tau_adm_input

        Output: self.delta_p e self.delta_v in ENU per il setpoint PX4,
                self.delta_p_yaw per lo yaw setpoint.
        """
        # -- 1. Ammettenza traslazionale (asse Z sensore) --
        F_s = float(self.F_adm_input)
        a_s = (F_s - self.adm_D * self.delta_v_s - self.adm_K * self.delta_p_s) / self.adm_M

        v_s_k = float(self.delta_v_s)    # salva v(k)
        self.delta_v_s += a_s * self.dt  # v(k+1)
        self.delta_p_s += v_s_k * self.dt  # p(k+1) con v(k)

        # Saturazione traslazionale
        if abs(self.delta_p_s) > self.adm_max_delta:
            self.delta_p_s = float(np.sign(self.delta_p_s) * self.adm_max_delta)

        # Output in ENU (lungo l'asse Z del sensore espresso in ENU)
        self.delta_p = R_sensor2enu[:, 2] * self.delta_p_s
        self.delta_v = R_sensor2enu[:, 2] * self.delta_v_s

        # -- 2. Ammettenza rotazionale yaw --
        tau_s = float(self.tau_adm_input)
        alpha_psi = (tau_s - self.adm_D_psi * self.delta_v_yaw - self.adm_K_psi * self.delta_p_yaw) / self.adm_J_psi

        v_psi_k = float(self.delta_v_yaw)
        self.delta_v_yaw += alpha_psi * self.dt
        self.delta_p_yaw += v_psi_k * self.dt

        # Saturazione rotazionale yaw
        if abs(self.delta_p_yaw) > self.adm_max_delta_yaw:
            self.delta_p_yaw = float(np.sign(self.delta_p_yaw) * self.adm_max_delta_yaw)


    # -- Pubblicazione setpoint ---------------------------

    def publish_setpoint(self, pos_enu: np.ndarray, yaw_enu: float,
                         vel_enu: np.ndarray = None):
        """
        Converte da ENU (con spawn offset) a NED locale (frame PX4) e pubblica.
        vel_enu [m/s]: se fornito, viene inviato come feedforward di velocità.
        """
        # Rimuovi spawn offset per tornare alle coordinate locali PX4
        lx = float(pos_enu[0] - self.start_x)
        ly = float(pos_enu[1] - self.start_y)
        lz = float(pos_enu[2] - self.start_z)

        msg = TrajectorySetpoint()
        # ENU --> NED: [E, N, U] → [N, E, -U]
        pose_ned = _M_NED2ENU.T @ np.array([lx, ly, lz])
        msg.position = [float(pose_ned[0]), float(pose_ned[1]), float(pose_ned[2])]

        # Feedforward velocità (ENU -> NED)
        if vel_enu is not None:
            vel_ned = _M_NED2ENU.T @ vel_enu
            msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]

        # Yaw: ENU -> NED convention
        msg.yaw = float(-yaw_enu + np.pi / 2)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardAdmittancePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
