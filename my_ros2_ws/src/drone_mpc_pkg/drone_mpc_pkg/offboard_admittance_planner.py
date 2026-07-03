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

  Per l'ammettenza usiamo solo le forze lineari (3 componenti), ruotate in ENU:
    R_sensor_to_body = Rz(0) · Ry(-π/2) → frame sensore → body FLU
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
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Wrench, Vector3Stamped, TwistStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
import numpy as np
import math
from scipy.spatial.transform import Rotation

from drone_mpc_pkg.planner import generate_trapezoidal_trajectory


# -- Frame: end_eff_sens → body FLU -------------------------------------------
# end_eff_sens ha pitch = -π/2 rispetto al body frame (FLU) 
# Gz FT default: misure nel frame child, verso parent (convenzione: forza che il
# contatto esercita sull'end-effector, nel frame child = sensore).

_R_SENSOR_TO_BODY = Rotation.from_euler('y', -np.pi / 2.0).as_matrix()
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
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('a_max', 2.0)
        self.declare_parameter('dt', 0.01)   # 100 Hz (più alto di prima per l'ammettenza)

        # -- Parametri ammettenza --
        self.declare_parameter('F_threshold', 0.06)    # [N] soglia attivazione
        #self.declare_parameter('adm_mass', 1.0)       # [kg] massa virtuale
        #self.declare_parameter('adm_damping', 8.0)    # smorzamento virtuale
        #self.declare_parameter('adm_stiffness', 0.0)  # rigidezza virtuale (0 = ammortizzatore puro)
        self.declare_parameter('adm_max_delta', 10.0)  # [m] saturazione spostamento

        # -- Topic FT sensor --
        # Viene passato come parametro dal launch file.
        self.declare_parameter(
            'ft_topic',
            '/world/interaction/model/x500_interaction/joint/end_eff_sens_joint/force_torque'
        )

        # -- Lettura parametri --
        ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.a_max = self.get_parameter('a_max').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        self.F_threshold = self.get_parameter('F_threshold').get_parameter_value().double_value
        #self.adm_M = self.get_parameter('adm_mass').get_parameter_value().double_value
        #self.adm_D = self.get_parameter('adm_damping').get_parameter_value().double_value
        #self.adm_K = self.get_parameter('adm_stiffness').get_parameter_value().double_value

        # -- Dimensionamento ammettenza (frame SENSOR) --
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
        # Risposta stazionaria a forza costante: delta_static = F / K
        #
        # --- ASSE Z (assiale, perp. alla parete) 
        # Forza tipica: 3 N (contatto leggero con la parete)
        # Cedevolezza desiderata: 3 N --> 8 cm di rimbalzo
        # Tempo di assestamento: 0.5 s (risposta reattiva ma stabile)
        # Smorzamento critico: niente rimbalzi sull'ostacolo
        F_typ_z    = 1.0     # [N]  forza di contatto tipica
        delta_typ_z= 0.05   # [m]  rimbalzo desiderato a F_typ_z (→ rigidezza K)
        Ta_z       = 1.0     # [s]  tempo assestamento al 5%
        zeta_z     = 1.1     # [-]  critico: risposta monotona senza rimbalzi

        # -- ASSE X e Y (laterali) -- (K alto = rigido) ------
        F_typ_x    = 400.0;  delta_typ_x = 0.0004  
        F_typ_y    = 100.0;  delta_typ_y = 0.0001   
        Ta_x       = 1;  zeta_x      = 1.0
        Ta_y       = 1;  zeta_y      = 1.0

        # -- M e D derivati wn e zeta
        K = np.array([F_typ_x / delta_typ_x,
                      F_typ_y / delta_typ_y,
                      F_typ_z / delta_typ_z])
        Ta   = np.array([Ta_x,   Ta_y,   Ta_z])
        zeta = np.array([zeta_x, zeta_y, zeta_z])
        wn   = 3.0 / (zeta * Ta)        # wn tale che assestamento 5% = Ta
        M    = K / wn**2                # M = K/wn²
        D    = 2.0 * zeta * K / wn      # D = 2*zeta*K/wn  (equivalente a 2*zeta*sqrt(K*M))

        self.adm_K = K
        self.adm_M = M
        self.adm_D = D
        self.adm_max_delta = self.get_parameter('adm_max_delta').get_parameter_value().double_value

        self.get_logger().info(
            f"[Admittance] K={self.adm_K}, M={self.adm_M.round(3)}, D={self.adm_D.round(3)}, "
            f"wn={wn.round(2)} rad/s, Ta={Ta}, zeta={zeta}"
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
        self.delta_p_pub        = self.create_publisher(Vector3Stamped, 'delta_p', 10)
        self.delta_p_sensor_pub  = self.create_publisher(Vector3Stamped, 'delta_p_sensor', 10)
        self.peg_ref_pub = self.create_publisher(PoseStamped, '/peg_ref_pose', 10)
        self.peg_ref_twist_pub = self.create_publisher(TwistStamped, '/peg_ref_twist', 10)

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

        # -- Stato interno (traiettoria) --
        self.current_pos = np.zeros(3)   # ENU + spawn offset
        self.current_rpy = np.zeros(3)
        self.R_flu2enu = np.eye(3)       # rotazione corrente body FLU --> ENU
        self.has_odom = False
        self.offboard_traj_enabled = True

        self.traj_p = None
        self.traj_rpy = None
        self.current_index = 0

        # -- Stato ammettenza --
        # Integrazione in terna SENSORE (assi disaccoppiati, nessun coupling da rotazione)
        # delta_p_s, delta_v_s: spostamento/velocità in terna sensore [m, m/s]
        # delta_p, delta_v:     idem in terna ENU (output per il setpoint PX4)
        self.delta_p_s = np.zeros(3)   # [m]    in sensor frame
        self.delta_v_s = np.zeros(3)   # [m/s]  in sensor frame
        self.delta_p   = np.zeros(3)   # [m]    in ENU  (= R_s2e @ delta_p_s)
        self.delta_v   = np.zeros(3)   # [m/s]  in ENU  (= R_s2e @ delta_v_s)

        # Forza esterna nel frame SENSOR (aggiornata dalla callback FT)
        self.F_ext_sens = np.zeros(3)
        self.admittance_active = False

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
        self.current_pos[0] = pos_enu[0] + self.get_parameter('start_x').value
        self.current_pos[1] = pos_enu[1] + self.get_parameter('start_y').value
        self.current_pos[2] = pos_enu[2] + self.get_parameter('start_z').value

        # Orientamento
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        self.current_rpy[:] = rot_flu2enu.as_euler('xyz')
        self.R_flu2enu = R_flu2enu

        self.has_odom = True

    def ft_cb(self, msg: Wrench):
        """
        Misura FT sensor -> forza esterna in ENU.

        Gz Sim ForceTorque:
          - Misure nel frame del child link (end_eff_sens)
          - end_eff_sens ha pitch = -π/2 rispetto al body FLU
          - Per passare al body FLU: R_SENSOR_TO_BODY = Ry(+π/2)
          - Per passare all'ENU: R_flu2enu (dall'odometria corrente)

        """
        F_sensor = np.array([msg.force.x, msg.force.y, msg.force.z])
        # correzione offset sensore (-0.05 su X)
        F_sensor[0] = F_sensor[0]+0.05
        F_norm = np.linalg.norm(F_sensor)

        was_active = self.admittance_active
        # Attiviamo l'ammettenza solo se superiamo la soglia E siamo sopra i 30 cm
        self.admittance_active = (F_norm >= self.F_threshold and self.current_pos[2] >= 0.3)

        # Se l'ammettenza NON deve agire (es. siamo a terra o forza debole), azzeriamo l'input.
        # In questo modo l'integrazione, se delta_p > 0, lo riporterà a zero (K>0)
        
        if self.admittance_active:
            self.F_ext_sens = F_sensor.copy()
        else:
            self.F_ext_sens = np.zeros(3)

        if self.admittance_active and not was_active:
            self.get_logger().info(
                f"[AdmittancePlanner] CONTATTO rilevato: |F|={F_norm:.3f}N >= {self.F_threshold:.2f}N"
            )
        elif not self.admittance_active and was_active:
            self.get_logger().info(
                "[AdmittancePlanner] Contatto perso. Ritorno a free-flight."
            )

    def enabled_cb(self, msg: Bool):
        self.offboard_traj_enabled = msg.data
        if not self.offboard_traj_enabled:
            self.get_logger().info("[AdmittancePlanner] DISABILITATO.")

    def live_target_cb(self, msg: PoseStamped):
        """Haptic live teleop: aggiorna il target direttamente, bypass traiettoria."""
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

        self.get_logger().info(
            f"[AdmittancePlanner] Nuova traiettoria da {x0[:3]} a {x_ref[:3]}"
        )

        t_vec, p_vals, rpy_vals = generate_trapezoidal_trajectory(
            x0, x_ref, dt=self.dt, v_max=self.v_max, a_max=self.a_max
        )

        self.traj_p = p_vals
        self.traj_rpy = rpy_vals
        self.current_index = 0

        # Reset dell'ammettenza ad ogni nuovo target (riparte da zero)
        self.delta_p[:] = 0.0
        self.delta_v[:] = 0.0

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
        LIVE_TIMEOUT = 0.5  # s: dopo questo tempo senza messaggi, mantieni ultima posizione
        if self.live_mode and self.live_target_pos is not None:
            elapsed = (self.get_clock().now() - self.live_mode_stamp).nanoseconds / 1e9
            if elapsed > LIVE_TIMEOUT:
                # Timeout: esci dal live mode, congela il target haptic come nuova traiettoria
                self.live_mode = False
                self.traj_p = [self.live_target_pos.copy()]
                self.traj_rpy = [np.array([0.0, 0.0, self.live_target_yaw])]
                self.current_index = 0
                self.get_logger().info("[AdmittancePlanner] Live mode TERMINATO - mantengo posizione haptic")
            else:
                # Setpoint diretto senza traiettoria
                p_cmd = self.live_target_pos + delta_p_enu
                self.publish_setpoint(p_cmd, self.live_target_yaw, delta_v_enu)
                return

        # -- Posizione e velocità nominali dalla traiettoria --
        if self.traj_p is None:
            if self.has_odom:
                self.publish_setpoint(self.current_pos, self.current_rpy[2])
            return

        idx = min(self.current_index, len(self.traj_p) - 1)
        p_nom = self.traj_p[idx]
        yaw_nom = self.traj_rpy[idx][2]
        # Velocità nominale: differenza finita in ENU
        idx_next = min(idx + 1, len(self.traj_p) - 1)
        v_nom = (self.traj_p[idx_next] - p_nom) / self.dt          # [m/s] ENU
        dyaw = self.traj_rpy[idx_next][2] - yaw_nom
        if dyaw >  np.pi: dyaw -= 2 * np.pi
        if dyaw < -np.pi: dyaw += 2 * np.pi
        yaw_rate_nom = dyaw / self.dt                               # [rad/s]

        # -- Composizione setpoint finale (delta_p/v già calcolati sopra) --
        p_cmd = p_nom + delta_p_enu
        v_cmd = v_nom + delta_v_enu

        self.publish_setpoint(p_cmd, yaw_nom, v_cmd)

        # -- Pubblica delta_p in ENU (per logging e RViz) --
        stamp = self.get_clock().now().to_msg()
        dp_msg = Vector3Stamped()
        dp_msg.header.stamp = stamp
        dp_msg.vector.x = float(self.delta_p[0])
        dp_msg.vector.y = float(self.delta_p[1])
        dp_msg.vector.z = float(self.delta_p[2])
        self.delta_p_pub.publish(dp_msg)

        # -- Pubblica delta_p in terna SENSORE (per plot) --
        dp_s = self._delta_p_sensor
        dp_s_msg = Vector3Stamped()
        dp_s_msg.header.stamp = stamp
        dp_s_msg.vector.x = float(dp_s[0])
        dp_s_msg.vector.y = float(dp_s[1])
        dp_s_msg.vector.z = float(dp_s[2])
        self.delta_p_sensor_pub.publish(dp_s_msg)

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

        # -- Pubblica velocità + yaw rate nominali (per logger) --
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'map'
        twist_msg.twist.linear.x  = float(v_nom[0])
        twist_msg.twist.linear.y  = float(v_nom[1])
        twist_msg.twist.linear.z  = float(v_nom[2])
        twist_msg.twist.angular.z = float(yaw_rate_nom)
        self.peg_ref_twist_pub.publish(twist_msg)

        # Avanza l'indice della traiettoria nominale
        if self.current_index < len(self.traj_p):
            self.current_index += 1

    # -- Integrazione ammettenza --

    def _integrate_admittance(self, R_sensor2enu: np.ndarray):
        """
        Integra la dinamica virtuale di ammettenza in TERNA SENSORE.

        Le matrici M, D, K sono diagonali per costruzione nel frame sensore.
        Integrare in sensor frame garantisce che gli assi siano completamente
        disaccoppiati: la forza su Z_s non genera mai displacement su X_s o Y_s.

        L'integrazione in ENU con K_enu(t) = R(t)@diag(K)@R(t)^T era scorretta:
        ad ogni timestep l'equilibrio della molla ruotava con il drone, generando
        cross-axis coupling in terna sensore.

        Output: self.delta_p e self.delta_v vengono aggiornati in ENU
                (= R_sensor2enu @ delta_p_s) per essere usati direttamente nel setpoint.
        """
        # -- Forza di ingresso in terna sensore --

        F_s = self.F_ext_sens.copy() # Per attivare tutti gli assi
        #F_s = np.array([0.0, 0.0, self.F_ext_sens[2]])

        # -- ODE disaccoppiata in sensor frame: M·a = F - D·v - K·p (element-wise) --
        a_s = (F_s - self.adm_D * self.delta_v_s - self.adm_K * self.delta_p_s) / self.adm_M

        # -- Integrazione Eulero esplicito (forward Euler) --
        v_s_k = self.delta_v_s.copy()    # salva v(k)
        self.delta_v_s += a_s * self.dt  # v(k+1)
        self.delta_p_s += v_s_k * self.dt  # p(k+1) con v(k)

        # -- Saturazione in terna sensore --
        delta_norm = np.linalg.norm(self.delta_p_s)
        if delta_norm > self.adm_max_delta:
            self.delta_p_s = self.delta_p_s / delta_norm * self.adm_max_delta

        # -- Output in ENU (per setpoint PX4) --
        self.delta_p = R_sensor2enu @ self.delta_p_s
        self.delta_v = R_sensor2enu @ self.delta_v_s


    # -- Pubblicazione setpoint ---------------------------

    def publish_setpoint(self, pos_enu: np.ndarray, yaw_enu: float,
                         vel_enu: np.ndarray = None):
        """
        Converte da ENU (con spawn offset) a NED locale (frame PX4) e pubblica.
        vel_enu [m/s]: se fornito, viene inviato come feedforward di velocità.
        """
        # Rimuovi spawn offset per tornare alle coordinate locali PX4
        lx = float(pos_enu[0] - self.get_parameter('start_x').value)
        ly = float(pos_enu[1] - self.get_parameter('start_y').value)
        lz = float(pos_enu[2] - self.get_parameter('start_z').value)

        msg = TrajectorySetpoint()
        # ENU → NED: [E, N, U] → [N, E, -U]
        pose_ned = _M_NED2ENU.T @ np.array([lx, ly, lz])
        msg.position = [float(pose_ned[0]), float(pose_ned[1]), float(pose_ned[2])]

        # Feedforward velocità (ENU → NED)
        if vel_enu is not None:
            vel_ned = _M_NED2ENU.T @ vel_enu
            msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]

        # Yaw: ENU → NED convention
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
