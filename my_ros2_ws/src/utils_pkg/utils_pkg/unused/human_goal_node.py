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
    Pubblica un array [Xc, Yc, Zc, Pan_mutuo] su '/pov_target'.
    """
    def __init__(self):
        super().__init__('human_goal_node')

        self.declare_parameter('cmd_topic', 'human_goal_vec')
        self.declare_parameter('goal_topic', '/pov_target')

        cmd_topic = str(self.get_parameter('cmd_topic').value) or 'human_goal_vec'
        out_topic = str(self.get_parameter('goal_topic').value) or '/pov_target'

        qos_in = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        qos_out = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        qos_sensor = QoSProfile(depth=10)

        # Publisher ora è Float64MultiArray
        self.goal_pub = self.create_publisher(Float64MultiArray, out_topic, qos_out)
        
        # Subscribers
        self.sub = self.create_subscription(Float64MultiArray, cmd_topic, self.cmd_cb, qos_in)
        self.key_ref_sub = self.create_subscription(Float64MultiArray, '/online_ref', self.keyboard_ref_cb, qos_in)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, qos_sensor)
        self.actual_pov_sub = self.create_subscription(Float64MultiArray, '/actual_pov', self.actual_pov_cb, qos_in)
        self.joy_ref_pub = self.create_publisher(Float64MultiArray, '/joy_ref', 1)

        # FIX 1: Inizializzato con coordinate PoV: [Xc, Yc, Zc, Pan_mutuo]
        self.current_pov_ref = [4.0, 0.0, 0.0, 0.0]  
        self.joy_active = False
        self.axes = [0.0] * 8
        self.buttons = [0] * 15
        self.task_phase_active = False
        self.last_button_state = 0
        self.last_joy_active = False

        # Variabili per il Joypad separato
        self.joy_pov_ref = [2.0, 0.0, 0.0, 0.0] 
        self.joy_pov_dot = [0.0, 0.0, 0.0, 0.0]
        self.actual_pov = [2.0, 0.0, 0.0, 0.0]

        # Parametri Joystick
        self.dt = 0.02   # 50 Hz
        self.v_pan_max = 0.5   
        self.v_zc_max = 0.5  
        self.v_xc_max = 1.0     
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Human Goal Node attivo. In attesa di comandi joypad...")

    def keyboard_ref_cb(self, msg: Float64MultiArray):
        if not self.joy_active and len(msg.data) >= 4:
            self.current_pov_ref = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
            self.publish_goal()

    def actual_pov_cb(self, msg: Float64MultiArray):
        # Memorizziamo la posa reale attuale
        if len(msg.data) >= 4:
            self.actual_pov = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]

    def joy_cb(self, msg: Joy):
        self.axes = msg.axes
        self.buttons = msg.buttons
        
        # Ignoriamo i cosi posteriori (L2/R2) che hanno valore di riposo 1.0
        active_axes = [self.axes[0], self.axes[1], self.axes[3]]
        self.joy_active = any(abs(a) > 0.05 for a in active_axes) or (self.buttons[0] == 1)

    def cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return
        
        self.current_pov_ref = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2]), float(msg.data[3])]
        self.joy_active = False
         
        self.publish_goal()

    def control_loop(self):
        # --- Rilevamento fronte di salita joy_active ---
        if self.joy_active and not self.last_joy_active:
            # Quando iniziamo a usare il joypad, partiamo dalla posizione attuale del drone
            self.joy_pov_ref = list(self.actual_pov)
            self.get_logger().info("Joypad ACTIVE: Partenza dalla posizione reale del drone.")
        
        self.last_joy_active = self.joy_active

        if not self.joy_active:
            # Se il joypad non è attivo, non pubblichiamo nulla!
            return

        # Toggle Task Phase with button 0 (es. 'A' o 'Cross')
        if len(self.buttons) > 0:
            if self.buttons[0] == 1 and self.last_button_state == 0:
                self.task_phase_active = not self.task_phase_active
                if self.task_phase_active:
                    self.get_logger().info("Task Phase ACTIVE: Framing destro")
                    self.current_pov_ref[1] = -0.6 # Yc offset a destra
                    self.current_pov_ref[0] = 1.0  # Xc più vicino
                else:
                    self.get_logger().info("Task Phase INACTIVE: Framing centrato")
                    self.current_pov_ref[1] = 0.0
                    self.current_pov_ref[0] = 2.0
                self.publish_goal()
            self.last_button_state = self.buttons[0]

        # Comandi Joypad (velocità desiderate)
        print_axes_or_something = False # non usato
        pan_vel  = self.axes[0] * self.v_pan_max
        zc_vel   = self.axes[1] * self.v_zc_max
        xc_vel   = self.axes[3] * self.v_xc_max
        yc_vel   = 0.0 # Per ora non mappato ma disponibile

        # Integrazione
        self.joy_pov_ref[3] += pan_vel * self.dt
        self.joy_pov_ref[2] += zc_vel * self.dt
        self.joy_pov_ref[0] += xc_vel * self.dt

        # Limiti
        xc_min, xc_max = 1.5, 8.0
        self.joy_pov_ref[0] = max(xc_min, min(xc_max, self.joy_pov_ref[0]))
        max_zc = 0.5
        self.joy_pov_ref[2] = max(-max_zc, min(max_zc, self.joy_pov_ref[2]))
        self.joy_pov_ref[3] = (self.joy_pov_ref[3] + math.pi) % (2 * math.pi) - math.pi

        # Memorizziamo le velocità (dot)
        self.joy_pov_dot = [xc_vel, yc_vel, zc_vel, pan_vel]

        # Pubblicazione Joy Reference
        joy_msg = Float64MultiArray()
        joy_msg.data = [
            float(self.joy_pov_ref[0]), float(self.joy_pov_ref[1]), float(self.joy_pov_ref[2]), float(self.joy_pov_ref[3]),
            float(self.joy_pov_dot[0]), float(self.joy_pov_dot[1]), float(self.joy_pov_dot[2]), float(self.joy_pov_dot[3])
        ]
        self.joy_ref_pub.publish(joy_msg)

    def publish_goal(self):
        msg = Float64MultiArray()
        # Invia Xc, Yc, Zc, Pan_mutuo
        msg.data = [
            float(self.current_pov_ref[0]),
            float(self.current_pov_ref[1]),
            float(self.current_pov_ref[2]),
            float(self.current_pov_ref[3])
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