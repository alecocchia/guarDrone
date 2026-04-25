#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
from geometry_msgs.msg import Wrench, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# Utility del tuo pacchetto
from drone_mpc_pkg.common import quat_to_R, quat_to_RPY, RPY_to_R

def skew(w):
    wx, wy, wz = w
    return np.array([[0, -wz,  wy],
                     [wz,  0, -wx],
                     [-wy, wx,  0]], dtype=float)

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]], dtype=float)

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

class GeometricController(Node):
    """
    Controller geometrico (Lee, SE(3)) in UN SOLO LOOP:
      - A = m g e3 + Kx*ex + Kv*ev + m*acc_ref   (world)
      - b3_d = A / ||A||, heading da psi_ref -> Rd
      - f = A ⋅ (R e3)                           (BODY z)
      - eR = 0.5 vee(Rd^T R - R^T Rd),  eW = W - R^T Rd Wd
      - M = -KR eR - KW eW + W × J W
    Pubblica Wrench in BODY: force.z = f, torque = M
    """
    def __init__(self):
        super().__init__('geometric_controller')

        self.declare_parameter('omega_ref_world', False)  # ('true') 'world' per planner_prova, ('false') 'body' per OCP
        self.omega_ref_world = True if self.get_parameter('omega_ref_world').value == True else False
        #print(self.omega_ref_world)
        #print(self.get_parameter('omega_ref_world').value)

        # ---- Parametri fisici (coerenti con il modello) ----
        self.m = 1.28
        self.g = 9.81
        self.J = np.diag([0.023, 0.023, 0.022])
        self.e3 = np.array([0.0, 0.0, 1.0])
        
        # ---- Guadagni ----
        # Attitude
        Ta_rot = 0.2  * 2 * np.pi      #0.5
        Ta_rot_yaw = 0.8 * 2 * np.pi       # 5
        zita_rot = 0.7
        zita_rot_yaw = 0.7
        wn_rot = 4/(zita_rot*Ta_rot)
        wn_rot_yaw = 4/(zita_rot_yaw*Ta_rot_yaw)
        #wn_rot = 5
        #wn_rot_yaw = 15
        self.KR = np.array([(wn_rot)**2,(wn_rot)**2,wn_rot_yaw**2])*np.diag(self.J)
        #self.KR = np.array([2, 2, 1])
        #self.KW = np.array([1, 1, 1])
        self.KW = 2*np.array([zita_rot, zita_rot, zita_rot_yaw])*np.diag(self.J)*np.sqrt(self.KR/np.diag(self.J))

        print('KR:',self.KR)
        #print('Ki_att:', self.Ki_att)
        print('KW:', self.KW)
        
        # Position
        Ta_z= 3 #5
        Ta_xy = 3 #5
        zita = 0.7
        zita_xy = 0.7
        wn_z = 4/ (zita*Ta_z)
        wn_xy = 4/ (zita_xy*Ta_xy)
        #wn_z = wn_rot/50
        #wn_xy = wn_z
        self.Kx = np.array([wn_xy**2,wn_xy**2,wn_z**2])*self.m
        self.Ki = np.zeros(3)
        self.Ki = self.Kx/100
        #self.Ki=np.zeros(3)
        self.Kv = 2*np.array([zita_xy, zita_xy, zita])*self.m*np.sqrt(self.Kx/self.m)
        self.e_int = np.zeros(3)
        
        #self.Kx = np.array([2.0, 2.0, 3.0]) * self.m
        #self.Kv = np.array([2,  2,  6]) * self.m
        #self.Kv = 2*np.sqrt(self.Kx)*1.3

        print('Kx:',self.Kx)
        #print('Ki_pos:', self.Ki_pos)
        print('Kv:', self.Kv)
        

        # Limiti attuatori
        self.Fz_min = 0.0
        self.Fz_max = 4.0 * self.m * self.g
        self.tau_max = np.array([0.25, 0.25, 0.15])

        # Frequenza controllo unico
        self.rate_ctrl_hz = 500.0
        self.prev_t = None
        self.last_twist_ref_cb_time = 0.0
        

        # ---- Stato ----
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.W = np.zeros(3)

        # ---- Riferimenti (default: zeri; all'arrivo odom li latcheiamo) ----
        self.ref_p = np.zeros(3)
        self.ref_v = np.zeros(3)
        self.ref_acc = np.zeros(3)
        self.ref_acc_ang = np.zeros(3)
        self.psi_ref = 0.0
        self.Wd = np.zeros(3)               # tipicamente [0,0,psi_dot_ref]
        self._have_external_ref = False

        # Buffer
        self.f_cmd = 0.0
        self.Rd = np.eye(3)

        # ---- ROS I/O ----
        qos_ready = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.ready_pub = self.create_publisher(Bool, '/controller_ready', qos_ready)
        self._ready = False

        self.sub_odom = self.create_subscription(Odometry, '/odometry', self.cb_odom, 1)
        self.sub_ref_pose = self.create_subscription(PoseStamped, '/optimal_drone_pose', self.cb_ref_pose, 1)
        self.sub_ref_twist = self.create_subscription(TwistStamped, '/optimal_drone_twist', self.cb_ref_twist, 1)

        self.pub_wrench = self.create_publisher(Wrench, '/wrench_cmd', 1)

        # ---- Timer UNICO ----
        self.timer = self.create_timer(1.0/self.rate_ctrl_hz, self.step_ctrl)

        self.get_logger().info('GeometricController di Lee pronto.')

    # ----------------- Callbacks -----------------
    def cb_odom(self, msg: Odometry):
        self.p[:] = [msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.position.z]
        self.v[:] = [msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z]
        #print("Actual velocity from odometry: ", self.v)
        self.v = self.R @ self.v
        #print("Transformed velocity: ", self.v)
        q = msg.pose.pose.orientation
        self.R = np.array(quat_to_R([q.w, q.x, q.y, q.z]).full(), dtype=float)
        self.W[:] = [msg.twist.twist.angular.x,
                     msg.twist.twist.angular.y,
                     msg.twist.twist.angular.z]

        # Latch ref iniziale da odom finché non arriva un ref esterno
        if not self._have_external_ref:
            self.ref_p = self.p.copy()
            rpy = np.array(quat_to_RPY([q.w, q.x, q.y, q.z]).full()).reshape(-1)
            self.psi_ref = float(rpy[2])

        self._publish_ready_once()

    def cb_ref_pose(self, msg: PoseStamped):
        self.ref_p[:] = [msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z]
        q = msg.pose.orientation
        rpy = np.array(quat_to_RPY([q.w, q.x, q.y, q.z]).full()).reshape(-1)
        self.psi_ref = float(rpy[2])
        self._have_external_ref = True

    def cb_ref_twist(self, msg: TwistStamped):
        now_time = self.get_clock().now().nanoseconds*1e-9
        dt = max(1e-3, now_time - self.last_twist_ref_cb_time)
        v_prev = self.ref_v.copy()
        omega_prev = self.Wd.copy()
        self.ref_v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        # acc_ref stimata alla freq del loop unico
        self.ref_acc = (self.ref_v - v_prev) /dt
        self.Wd = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        self.ref_acc_ang=(self.Wd - omega_prev)/dt
        self.last_twist_ref_cb_time = now_time

    # ----------------- One-loop (Lee) -----------------
    def step_ctrl(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            self.prev_t = now
            return
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        # Errori pos/vel (world)    #vel arriva in body dall' odometria
        ex = self.ref_p - self.p
        ev = self.ref_v - self.v
        e_int = self.e_int + (self.ref_p - self.p)*dt

        # Forza desiderata in WORLD:
        # v_dot = g e3 - (f/m) R e3  =>  F_des = m g e3 + Kx*ex + Kv*ev + m*acc_ref
        F_des = (self.m * self.g * self.e3) + (self.Kx * ex) + (self.Kv * ev) + (self.Ki *e_int) + self.m * self.ref_acc
        if np.linalg.norm(F_des) < 1e-6:
            F_des = self.m * self.g * self.e3

        # (opzionale) limite acc laterale richiesta: scommenta se utile
        a_xy_max = 4.0  # m/s^2
        ax, ay, az = F_des / self.m
        ax = float(np.clip(ax, -a_xy_max, a_xy_max))
        ay = float(np.clip(ay, -a_xy_max, a_xy_max))
        F_des = self.m * np.array([ax, ay, az])

        # Rd: zbd || +F_des, heading = psi_ref
        if np.linalg.norm(F_des)> 1e-3 :
            zbd = F_des / np.linalg.norm(F_des)
        else :
            zbd = self.e3.copy()

        x_ref = np.array([np.cos(self.psi_ref), np.sin(self.psi_ref), 0.0])
        ybd = np.cross(zbd, x_ref)
        n2 = np.linalg.norm(ybd)
        if n2 < 1e-6:
            x_ref = np.array([1.0, 0.0, 0.0])
            ybd = np.cross(zbd, x_ref)
            n2 = np.linalg.norm(ybd)

        ybd /= n2
        xbd = np.cross(ybd, zbd)
        self.Rd = np.column_stack((xbd, ybd, zbd))

        # Thrust (BODY z): f = F_des · (R e3)
        self.f_cmd = float(self.e3.T @ self.R.T @ F_des)
        #self.f_cmd = float(np.clip(self.f_cmd, self.Fz_min, self.Fz_max))

        # Errori e momenti su SO(3)
        Re = self.Rd.T @ self.R         # errore in body frame
        eR = 0.5 * vee(Re - Re.T)
        # --- eW e feed-forward coerenti col frame di Wd ---
        #Calcolo la Wb_d in body frame attuale
        if self.omega_ref_world is False:
            # caso OCP: Wd in desired body frame (prima la porto nel mondo e poi nella body frame attuale)
            Wd_b  = self.R.T @ self.Rd @ self.Wd
            dWd_b = self.R.T @ self.Rd @ self.ref_acc_ang
        else:
            # (caso planner_prova): Wd in WORLD (la porto direttamente in body frame attuale)
            Wd_b  = self.R.T @ self.Wd
            dWd_b = self.R.T @ self.ref_acc_ang

        eW = self.W - Wd_b      # W arriva in body in quanto proviene dall'odometria  (child_frame = base)
        ff_att = - self.J @ (skew(self.W) @ Wd_b - dWd_b )
        tau_unsat = -(self.KR * eR) - (self.KW * eW) + skew(self.W) @ self.J @ (self.W) + ff_att
        tau = tau_unsat
        #tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)

        # Pubblica Wrench (BODY)
        w = Wrench()
        w.force.x = 0.0
        w.force.y = 0.0
        w.force.z = self.f_cmd
        w.torque.x, w.torque.y, w.torque.z = float(tau[0]), float(tau[1]), float(tau[2])
        self.pub_wrench.publish(w)

        self.e_int = e_int

    # ----------------- Ready -----------------
    def _publish_ready_once(self):
        if not self._ready:
            self._ready = True
            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info('Controller READY')

def main(args=None):
    rclpy.init(args=args)
    node = GeometricController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
