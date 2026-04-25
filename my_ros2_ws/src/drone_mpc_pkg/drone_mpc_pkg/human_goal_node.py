#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class HumanGoalNode(Node):
    """
    Nodo ibrido per teleoperazione telecamera.
    Legge comandi discreti da 'human_goal_vec' o integrati da '/joy'.
    Pubblica un array [r, pan, tilt] su 'human_goal'.
    """
    def __init__(self):
        super().__init__('human_goal_node')

        self.declare_parameter('cmd_topic', 'human_goal_vec')
        self.declare_parameter('goal_topic', 'human_goal')

        cmd_topic = str(self.get_parameter('cmd_topic').value) or 'human_goal_vec'
        out_topic = str(self.get_parameter('goal_topic').value) or 'human_goal'

        qos_in = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        qos_out = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        qos_sensor = QoSProfile(depth=10)

        # Publisher ora è Float64MultiArray
        self.goal_pub = self.create_publisher(Float64MultiArray, out_topic, qos_out)
        
        # Subscribers
        self.sub = self.create_subscription(Float64MultiArray, cmd_topic, self.cmd_cb, qos_in)
        self.key_ref_sub = self.create_subscription(Float64MultiArray, '/online_ref', self.keyboard_ref_cb, qos_in)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, qos_sensor)

        # FIX 1: Inizializzato con coordinate sferiche di base
        self.current_spherical_ref = [2.0, 0.0, 0.0]  
        self.joy_active = False
        self.axes = [0.0] * 8

        # Parametri Joystick
        self.dt = 0.02   # 50 Hz
        self.v_pan_max = 0.5   
        self.v_tilt_max = 0.5  
        self.v_r_max = 1.0     
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Human Goal Node attivo. In attesa di comandi joypad...")

    def keyboard_ref_cb(self, msg: Float64MultiArray):
        if not self.joy_active and len(msg.data) >= 3:
            self.current_spherical_ref = [msg.data[0], msg.data[1], msg.data[2]]

    def joy_cb(self, msg: Joy):
        self.axes = msg.axes
        
        # Ignoriamo i cosi posteriori (L2/R2) che hanno valore di riposo 1.0 (bho la prima volta era così, la seconda no)
        # Controlliamo solo gli assi che ci interessano (0, 1 e 3)
        #if len(self.axes) >= 5:
        active_axes = [self.axes[0], self.axes[1], self.axes[3]]
        self.joy_active = any(abs(a) > 0.05 for a in active_axes)

    def cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        
        self.current_spherical_ref = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
        self.joy_active = False
         
        self.publish_goal()

    def control_loop(self):
        if not self.joy_active:
            return

        pan_cmd  = self.axes[0]
        tilt_cmd = self.axes[1]
        r_cmd    = self.axes[3]

        self.current_spherical_ref[1] += pan_cmd * self.v_pan_max * self.dt
        self.current_spherical_ref[2] += tilt_cmd * self.v_tilt_max * self.dt
        self.current_spherical_ref[0] += r_cmd * self.v_r_max * self.dt

        # Limiti
        # 1. Limiti sul Raggio (Zoom)
        r_min = 0.5  # Non avvicinarsi a meno di 50 cm
        r_max = 8.0  # Non allontanarsi a più di 8 metri
        self.current_spherical_ref[0] = max(r_min, min(r_max, self.current_spherical_ref[0]))
        #2. Limiti sul Tilt (Altezza)
        max_tilt = 30.0 * math.pi / 180.0
        self.current_spherical_ref[2] = max(-max_tilt, min(max_tilt, self.current_spherical_ref[2]))
        # Wrap-Around del pan orbitare a 360° infinitamente
        # Mantiene il valore sempre pulito nel range [-pi, +pi] senza bloccare il volo
        self.current_spherical_ref[1] = (self.current_spherical_ref[1] + math.pi) % (2 * math.pi) - math.pi
        self.publish_goal()

    def publish_goal(self):
        msg = Float64MultiArray()
        # Invia r, pan, tilt
        msg.data = [
            float(self.current_spherical_ref[0]),
            float(self.current_spherical_ref[1]),
            float(self.current_spherical_ref[2])
        ]
        self.goal_pub.publish(msg)

def main():
    rclpy.init()
    node = HumanGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()