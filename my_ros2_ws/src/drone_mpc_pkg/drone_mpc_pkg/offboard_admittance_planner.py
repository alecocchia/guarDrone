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


# ── Frame: end_eff_sens → body FLU ──────────────────────────────────────────
# end_eff_sens ha pitch = -π/2 rispetto al frame modello (ENU-aligned quando yaw=0)
# Gz FT default: misure nel frame child, verso parent (convenzione: forza che il
# contatto esercita sull'end-effector, nel frame child = sensore).
# Per portare in sensor da body FLU: ruotiamo di π/2 attorno a Y (preso dall' SDF).

_R_BODY_TO_SENSOR = Rotation.from_euler('y', np.pi / 2.0).as_matrix()
_R_SENSOR_TO_BODY = _R_BODY_TO_SENSOR.T
# ── Conversione NED → ENU (stessa usata in offboard_trajectory_planner) ──────
_M_NED2ENU = np.array([[0., 1., 0 ],
                        [1., 0., 0.],
                        [0., 0., -1.]])

# ── Conversione FRD → FLU ────────────────────────────────────────────────────
_M_FRD2FLU = np.array([[1., 0., 0.],
                        [0., -1., 0.],
                        [0., 0., -1.]])


def quaternion_to_euler(w, x, y, z):
    """Quaternione → RPY (roll, pitch, yaw)."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class OffboardAdmittancePlanner(Node):
    """
    Planner traiettoria per il drone peg con controllo di ammettenza.

    In free-flight (|F_ext| < F_threshold) si comporta come OffboardTrajectoryPlanner.
    In contatto (|F_ext| >= F_threshold) integra una dinamica virtuale di ammettenza
    per modificare il target di posizione inviato a PX4.
    """

    def __init__(self):
        super().__init__('offboard_admittance_planner')

        # ── Parametri planner base ────────────────────────────────────────────
        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('a_max', 2.0)
        self.declare_parameter('dt', 0.01)   # 100 Hz (più alto di prima per l'ammettenza)

        # -- Parametri ammettenza --
        self.declare_parameter('F_threshold', 0.2)    # [N] soglia attivazione
        #self.declare_parameter('adm_mass', 1.0)       # [kg] massa virtuale
        #self.declare_parameter('adm_damping', 8.0)    # smorzamento virtuale
        #self.declare_parameter('adm_stiffness', 0.0)  # rigidezza virtuale (0 = ammortizzatore puro)
        self.declare_parameter('adm_max_delta', 0.75)  # [m] saturazione spostamento

        # -- Topic FT sensor --
        # Il topic cambia con l'ordine di spawn (_0 o _1).
        # Viene passato come parametro dal launch file.
        self.declare_parameter(
            'ft_topic',
            '/world/interaction/model/x500_interaction_0/joint/end_eff_sens_joint/force_torque'
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

        #wn=sqrt(K/M)
        #zeta=D/2*sqrt(K*M)     --> smorzamento critico = 1
        
        # Ora adm_K, adm_M, adm_D sono array numpy (1 per asse nel frame SENSOR)
        # Es: Kx=50 (laterale), Ky=50 (laterale), Kz=50 (assiale al peg)
        F_max_x = 2.0/10
        F_max_y = 2.0/10
        F_max_z = 8.0/10

        delta_x_max = 0.05
        delta_y_max = 0.05
        delta_z_max = 0.4

        adm_K_x = F_max_x/delta_x_max
        adm_K_y = F_max_y/delta_y_max
        adm_K_z = F_max_z/delta_z_max

        self.adm_K = np.array([adm_K_x, adm_K_y, adm_K_z])
        self.Ta = np.array([5,5,5])
        self.wn = 4 / self.Ta
        self.adm_M = self.adm_K / (self.wn**2)
        self.adm_D = 2 * np.sqrt(self.adm_K * self.adm_M)*np.array([1.5,1.5,1.2])
        self.adm_max_delta = self.get_parameter('adm_max_delta').get_parameter_value().double_value

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
        self.delta_p_pub = self.create_publisher(Vector3Stamped, 'delta_p', 10)
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

        # -- Stato interno (traiettoria) --
        self.current_pos = np.zeros(3)   # ENU + spawn offset
        self.current_rpy = np.zeros(3)
        self.R_flu2enu = np.eye(3)       # rotazione corrente body FLU → ENU
        self.has_odom = False
        self.offboard_traj_enabled = True

        self.traj_p = None
        self.traj_rpy = None
        self.current_index = 0

        # -- Stato ammettenza --
        # delta_p: spostamento accumulato dall'ammettenza rispetto alla traiettoria nominale
        # delta_v: velocità di tale spostamento
        self.delta_p = np.zeros(3)
        self.delta_v = np.zeros(3)

        # Forza esterna nel frame SENSOR (aggiornata dalla callback FT)
        self.F_ext_sens = np.zeros(3)
        self.admittance_active = False

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

        # Posizione: NED → ENU + spawn offset
        pos_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        pos_enu = _M_NED2ENU @ pos_ned
        self.current_pos[0] = pos_enu[0] + self.get_parameter('start_x').value
        self.current_pos[1] = pos_enu[1] + self.get_parameter('start_y').value
        self.current_pos[2] = pos_enu[2] + self.get_parameter('start_z').value

        # Orientamento
        rot_flu2enu = Rotation.from_matrix(R_flu2enu)
        self.current_rpy[:] = rot_flu2enu.as_euler('xyz')
        self.R_flu2enu = R_flu2enu

        # Yaw per il planner (ENU convention)
        _, _, y_ned = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self.current_rpy[2] = -y_ned + np.pi / 2.0

        self.has_odom = True

    def ft_cb(self, msg: Wrench):
        """
        Misura FT sensor → forza esterna in ENU.

        Gz Sim ForceTorque (nessun tag <frame> nel SDF):
          - Misure nel frame del child link (end_eff_sens), convenzione child→parent.
          - end_eff_sens ha pitch = -π/2 rispetto al modello (ENU-aligned a yaw=0).
          - Per passare al body FLU: R_SENSOR_TO_BODY = Ry(+π/2)
          - Per passare all'ENU: R_flu2enu (dall'odometria corrente)

        La convenzione child→parent significa che msg.force è la forza che il contatto
        applica sull'end-effector (quella che vogliamo compensare con l'ammettenza).
        """
        F_sensor = np.array([msg.force.x, msg.force.y, msg.force.z])
        F_norm = np.linalg.norm(F_sensor)

        was_active = self.admittance_active
        # Attiviamo l'ammettenza solo se superiamo la soglia E siamo sopra i 30 cm
        self.admittance_active = (F_norm >= self.F_threshold and self.current_pos[2] >= 0.3)

        # Se l'ammettenza NON deve agire (es. siamo a terra o forza debole), azzeriamo l'input.
        # In questo modo l'integrazione, se delta_p > 0, lo riporterà a zero dolcemente (K>0)
        
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
            _, _, t_yaw = quaternion_to_euler(
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            )

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

        # Reset dell'ammettenza ad ogni nuovo target (si riparte da zero)
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
        ocm.velocity = True    # feedforward velocità abilitato
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(ocm)

        if self.traj_p is None:
            # Nessun target: tieni la posizione corrente
            if self.has_odom:
                self.publish_setpoint(self.current_pos, self.current_rpy[2])
            return

        # -- Posizione e velocità nominali dalla traiettoria --
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

        # -- Aggiornamento ammettenza --
        # Integriamo SEMPRE l'equazione. Se non c'è contatto (admittance_active=False), 
        # F_ext_sens è [0,0,0], quindi la molla virtuale (K) riporterà naturalmente delta_p a zero!
        self._integrate_admittance()

        # -- Composizione setpoint finale --
        # Ruotiamo delta_p e delta_v dal frame SENSOR al frame ENU
        R_sensor2enu =  self.R_flu2enu @ _R_SENSOR_TO_BODY
        delta_p_enu = R_sensor2enu @ self.delta_p
        delta_v_enu = R_sensor2enu @ self.delta_v

        p_cmd = p_nom + delta_p_enu
        v_cmd = v_nom + delta_v_enu   # velocità totale (nominale + contributo ammettenza)

        self.publish_setpoint(p_cmd, yaw_nom, v_cmd)

        # -- Pubblica delta_p_enu (per logging e RViz) --
        dp_msg = Vector3Stamped()
        dp_msg.header.stamp = self.get_clock().now().to_msg()
        dp_msg.vector.x = float(self.delta_p[0])
        dp_msg.vector.y = float(self.delta_p[1])
        dp_msg.vector.z = float(self.delta_p[2])
        self.delta_p_pub.publish(dp_msg)

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

    def _integrate_admittance(self):
        """
        Integra la dinamica virtuale di ammettenza:
            M·Δp̈ + D·Δṗ + K·Δp = F_ext_enu
        con metodo di Eulero esplicito al passo dt.
        """
        F = self.F_ext_sens.copy()

        # Accelerazione virtuale nel frame SENSOR
        delta_a = (F - self.adm_D * self.delta_v - self.adm_K * self.delta_p) / self.adm_M
        #delta_a[0] = 0.0
        #delta_a[1] = 0.0
        # Integrazione Eulero
        self.delta_v += delta_a * self.dt
        self.delta_p += self.delta_v * self.dt

        # Saturazione: lo spostamento non può superare adm_max_delta
        delta_norm = np.linalg.norm(self.delta_p)
        if delta_norm > self.adm_max_delta:
            self.delta_p = self.delta_p / delta_norm * self.adm_max_delta


    # ── Pubblicazione setpoint ────────────────────────────────────────────────

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
