#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, pi , asin , atan2
from drone_mpc_pkg.common import quat_to_RPY,wrap_pi, min_angle, RPY_to_R, R_to_RPY

class QuadPIDController(Node):
    """
    PID a due anelli:
      - Outer (pos, world)  -> a_des (world)  [75 Hz]
      - Mapping a_des+psi   -> (phi_des, theta_des)
      - Inner (attitude)    -> torques (body)  [300 Hz]
      - Thrust Fz = m*(g + a_des_z)
    Pubblica Wrench(body) su /wrench_cmd.
    """
    def __init__(self):
        super().__init__('PID_controller')

        # ---- Parametri fisici (coerenti col SDF) ----
        self.m = 1.28
        self.g = 9.81
        self.J = np.diag([0.0229, 0.029, 0.0218])


        # QoS latched per "ready"
        qos_ready = QoSProfile(depth=1,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=QoSReliabilityPolicy.RELIABLE)
        self.ready_pub = self.create_publisher(Bool, '/controller_ready', qos_ready)
        self._ready = False

        # ---- Guadagni ----

        wmax_ref = 0.02*2*pi        # se ragiono con massima frequenza riferimento -> w_outer = 5-10 volte
        Ta_xy=4
        Ta_z=4
        zita_pos = 0.7
        wn_z = 4/ (zita_pos*Ta_z)
        print('wn', wn_z)
        wn_xy = 4/(zita_pos*Ta_xy)
        self.Kp_pos = np.array([wn_xy**2,wn_xy**2,wn_z**2])
        self.Ki_pos = np.array([0, 0, 0])
        self.Kd_pos = 2*zita_pos*np.sqrt(self.Kp_pos)
        #self.Kd_pos = np.array([0,0,np.sqrt(2*self.Kp_pos[2]*self.m)])
        #self.Kd_pos = np.array([0,0,0])
        print('Kp_pos:',self.Kp_pos)
        print('Ki_pos:', self.Ki_pos)
        print('Kd_pos:', self.Kd_pos)


        Ta_att = 0.5* 2 * np.pi
        zita_att = 0.7
        wn_att = 4/(zita_att*Ta_att)
        #self.Kp_att = np.array([0.1, 0.1, 0.1])
        I = np.diag(self.J)
        self.Kp_att = I*wn_att**2
        self.Ki_att = np.array([0, 0, 0])
        self.Kd_att = 2*zita_att*I*np.sqrt(self.Kp_att/I)
        #self.Kd_att = np.array([0,0,0])
        print('Kp_att:',self.Kp_att)
        print('Ki_att:', self.Ki_att)
        print('Kd_att:', self.Kd_att)

        # Limiti
        self.Fz_min = 0.0
        self.Fz_max = 4.0*self.m*self.g
        self.tau_max = np.array([0.25, 0.25, 0.15])
        self.angle_limit = 0.35
        self.a_xy_max = 4.0

        # Soft-start
        self.soft_start_T = 0.0

        # Topics
        self.odom_topic   = '/odometry'
        self.wrench_topic = '/wrench_cmd'
        self.ref_topic    = '/optimal_drone_pose'
        self.ref_twist_topic = '/optimal_drone_twist'

        # Stato
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.rpy = np.zeros(3)
        self.omega = np.zeros(3)

        # Riferimenti (hold iniziale)
        self.ref_p = np.zeros(3)
        self.ref_v = np.zeros(3)
        self.ref_yaw = 0.0
        self.ref_omega = np.zeros(3)
        self.ref_acc = np.zeros(3)

        # Integratori anti-windup
        self.e_pos_int = np.zeros(3)
        self.e_att_int = np.zeros(3)
        self.int_pos_limit = np.array([2.0, 2.0, 2.0])
        self.int_att_limit = np.array([0.5, 0.5, 0.5])

        # Timing
        self.prev_t_pos = None
        self.prev_t_att = None
        self.t0 = None

        # Frequenze (outer / inner)
        self.rate_pos_hz = 100.0
        self.rate_att_hz = 500.0

        self.first_odom = False
        self.first_ref = False
        self.last_twist_time = 0.0

        # Buffer condiviso: ultimo a_des calcolato dall’outer
        self.a_des = np.zeros(3)

        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 1)
        self.sub_ref  = self.create_subscription(PoseStamped, self.ref_topic, self.cb_ref, 1)
        self.pub_wrench = self.create_publisher(Wrench, self.wrench_topic, 1)
        self.sub_ref_twist = self.create_subscription(TwistStamped, self.ref_twist_topic, self.cb_ref_twist, 1)

        # Timers separati (outer/inner)
        self.timer_pos = self.create_timer(1.0/self.rate_pos_hz, self.step_outer_pos)
        self.timer_att = self.create_timer(1.0/self.rate_att_hz, self.step_inner_att)

        self.get_logger().info(f'PID up. odom={self.odom_topic}, ref={self.ref_topic}, cmd={self.wrench_topic} '
                               f'| outer={self.rate_pos_hz} Hz, inner={self.rate_att_hz} Hz')

    # --- Callbacks -----------------------------------------------------------
    def cb_odom(self, msg: Odometry):
        self.p[:] = [msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.position.z]
        self.v[:] = [msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z]

        q = msg.pose.pose.orientation
        self.rpy[:] = quat_to_RPY([q.w, q.x, q.y, q.z]).full().flatten()
        
        # self.v arriva dall'odometry che la pubblica in body frame -> ruotiamola in mondo
        self.R = np.array(RPY_to_R(self.rpy[0],self.rpy[1], self.rpy[2]))
        self.v[:] = self.R @ self.v[:]

        self.omega[:] = [msg.twist.twist.angular.x,
                         msg.twist.twist.angular.y,
                         msg.twist.twist.angular.z]

        if self.first_ref is False:
            if self.first_odom is True :
                self.ref_p = self.p.copy()
                self.ref_yaw = float(self.rpy[2])
                self.ref_v = np.zeros(3)
                self.ref_omega = np.zeros(3)
                self.ref_acc = np.zeros(3)
            else :
                self.first_odom = True
            
        if self.t0 is None:
            self.t0 = self.get_clock().now().nanoseconds * 1e-9


    def _publish_ready_once(self):
        if not self._ready:
            self._ready = True
            m = Bool(); m.data = True
            self.ready_pub.publish(m)
            self.get_logger().info("Controller READY")

    def cb_ref(self, msg: PoseStamped):
        self.ref_p = np.array([msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z], dtype=float)   # world
        q = msg.pose.orientation                                    # body
        _, _, yaw = quat_to_RPY([q.w, q.x, q.y, q.z]).full().flatten()
        self.ref_yaw = float(yaw)
    

    def cb_ref_twist(self, msg: TwistStamped):
        if self.ref_v is not None :
            v_prev = self.ref_v.copy()
        else :
            v_prev = np.zeros(3)
        now = self.get_clock().now().nanoseconds*1e-9
        last_twist_time = self.last_twist_time
        self.ref_v[:] = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]    # world
        self.ref_omega[:] = [msg.twist.angular.x, msg.twist.angular.y,msg.twist.angular.z]  # body
        dt = max(now-last_twist_time, 1e-4)
        self.ref_acc = (self.ref_v - v_prev) / dt
        self.last_twist_time = self.get_clock().now().nanoseconds*1e-9    


    # --- Outer loop (position -> a_des) -------------------------------------
    def step_outer_pos(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t_pos is None:
            self.prev_t_pos = now
            return
        dt = max(1e-3, now - self.prev_t_pos)
        self.prev_t_pos = now

        if self.ref_p is None or self.ref_yaw is None or self.ref_v is None:
            return

        e_pos = self.ref_p - self.p
        e_vel = self.ref_v - self.v

        # integratore pos con anti-windup
        self.e_pos_int += e_pos * dt
        self.e_pos_int = np.clip(self.e_pos_int, -self.int_pos_limit, self.int_pos_limit)

        a_des = self.ref_acc + (self.Kp_pos * e_pos) + (self.Ki_pos * self.e_pos_int) + (self.Kd_pos * e_vel)
        a_des[0:2] = np.clip(a_des[0:2], -self.a_xy_max, self.a_xy_max)
        a_des[2]   = float(np.clip(a_des[2], -4.0, 4.0))

        self.a_des[:] = a_des  # buffer per l’anello veloce

    # --- Inner loop (attitude -> torques & thrust) --------------------------
    def step_inner_att(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t_att is None:
            self.prev_t_att = now
            return
        if self.ref_p is None or self.ref_yaw is None or not self.first_odom:
            return
        dt = max(1e-3, now - self.prev_t_att)
        self.prev_t_att = now

        if self.ref_p is None or self.ref_yaw is None or self.ref_omega is None:
            return

        a_des = self.a_des.copy()
        psi_ref = self.ref_yaw
        sPsi, cPsi = sin(psi_ref), cos(psi_ref)

        ax, ay, az = a_des
        A = ax*cPsi + ay*sPsi
        B = ax*sPsi - ay*cPsi
        C = az + self.g

        den = max(np.sqrt(A*A + B*B + C*C) , 1e-12)  # evita div/0

        phi_des   = asin(B/den)
        theta_des = atan2(A, C) 

        phi_des   = np.clip(phi_des,   -self.angle_limit, self.angle_limit)
        theta_des = np.clip(theta_des, -self.angle_limit, self.angle_limit)

        # thrust body-z
        Fz_cmd = self.m*(np.sqrt(A*A + B*B + C*C))
        Fz_cmd = float(np.clip(Fz_cmd, self.Fz_min, self.Fz_max))

        # soft-start
        s = (now - (self.t0 or now)) / self.soft_start_T if self.soft_start_T > 1e-6 else 1.0
        s = float(np.clip(s, 0.0, 1.0))
        Fz_cmd *= s

        # attitude PID
        roll, pitch, yaw = self.rpy
        e_att = np.array([(phi_des - roll),
                          (theta_des - pitch),
                          wrap_pi(psi_ref - yaw)] , dtype=float)
        #print ("Riferimento attuale yaw: ", psi_ref)
        #print("Errore attale yaw : ",e_att[2])
        
        e_omega = self.ref_omega-self.omega

        # PID senza clip
        tau_unsat = (self.Kp_att*e_att) + (self.Ki_att*self.e_att_int) + (self.Kd_att*e_omega)
        tau = tau_unsat     # commentare se si vuole la saturazione
        # Clip ai limiti attuatori 
        tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)
        beta = 5.0  # 2–10 va bene
        self.e_att_int += (e_att + beta*(tau - tau_unsat)) * dt
        self.e_att_int = np.clip(self.e_att_int, -self.int_att_limit, self.int_att_limit)



        # publish wrench
        w = Wrench()
        w.force.x = 0.0
        w.force.y = 0.0
        w.force.z = float(Fz_cmd)
        w.torque.x = float(tau[0])
        w.torque.y = float(tau[1])
        w.torque.z = float(tau[2])
        self.pub_wrench.publish(w)
        
        self._publish_ready_once()
def main(args=None):
    rclpy.init(args=args)
    node = QuadPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
