#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Controllo Peg Drone (PoseStamped)
---------------------------
Muovi il setpoint:
   w/s : Avanza/Indietreggia (Y)
   a/d : Sinistra/Destra (X)
   r/f : Su/Giu (Z)

Spazio: Forza l'invio del target
CTRL-C per uscire
"""

move_bindings = {
    'w': (0, 0.2, 0),
    's': (0, -0.2, 0),
    'a': (-0.2, 0, 0),
    'd': (0.2, 0, 0),
    'r': (0, 0, 0.2),
    'f': (0, 0, -0.2),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopPeg(Node):
    def __init__(self):
        super().__init__('teleop_peg')
        self.pub = self.create_publisher(PoseStamped, '/peg_target_pose', 10)
        # Sottoscriviamo all'odometria del peg (o pose corrente) per sapere da dove partire, ma
        # per semplicità partiamo da una posizione hardcoded o aspettiamo il primo messaggio
        self.sub = self.create_subscription(PoseStamped, '/peg_target_pose', self.pose_cb, 10)
        self.target = [-4.0, -4.0, 10.0]

    def pose_cb(self, msg):
        if self.target is None:
            self.target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.get_logger().info(f"Target agganciato: {self.target}")

    def run(self):
        print(msg)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            key = getKey()
            
            if key in move_bindings and self.target is not None:
                dx, dy, dz = move_bindings[key]
                self.target[0] += dx
                self.target[1] += dy
                self.target[2] += dz
                self.publish_target()
                print(f"\rTarget: X={self.target[0]:.2f}, Y={self.target[1]:.2f}, Z={self.target[2]:.2f}    ", end='')
            
            elif key == ' ':
                if self.target is not None:
                    self.publish_target()
            elif key == '\x03':
                break

    def publish_target(self):
        p = PoseStamped()
        p.header.frame_id = 'world'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(self.target[0])
        p.pose.position.y = float(self.target[1])
        p.pose.position.z = float(self.target[2])
        # Lasciamo l'orientamento a zero per ora (o quello che serve)
        p.pose.orientation.w = 1.0
        self.pub.publish(p)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPeg()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
