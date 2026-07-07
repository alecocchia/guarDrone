#!/usr/bin/env python3
# =============================================================================
# keyboard_client.py — Nodo ROS2 per input da tastiera dell'operatore
#
# Pubblica i comandi dell'operatore su /keyboard_input (std_msgs/String).
# Comandi riconosciuti:
#   - "ok"   → conferma passaggio alla fase successiva
#   - "stop" → richiede atterraggio d'emergenza
#
# Il nodo gira nel suo pane tmux dedicato, con terminale pulito.
# =============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardClientNode(Node):
    def __init__(self):
        super().__init__('keyboard_client_node')

        self.kb_pub = self.create_publisher(String, '/keyboard_input', 10)

        self.get_logger().info('Keyboard Client avviato.')
        self.get_logger().info('Comandi disponibili:')
        self.get_logger().info('  ok   → conferma passaggio fase')
        self.get_logger().info('  stop → atterraggio d\'emergenza')
        self.get_logger().info('─' * 40)

    def run(self):
        """Loop bloccante che legge da stdin e pubblica su /keyboard_input."""
        try:
            while rclpy.ok():
                try:
                    user_input = input('\n⏳ In attesa di comando (ok / stop): ').strip().lower()
                except EOFError:
                    break

                if not user_input:
                    continue

                msg = String()
                msg.data = user_input
                self.kb_pub.publish(msg)

                if user_input == 'ok':
                    self.get_logger().info('✅ Conferma inviata al supervisor.')
                elif user_input == 'stop':
                    self.get_logger().warn('🛑 Comando STOP inviato al supervisor!')
                else:
                    self.get_logger().info(f'📤 Inviato: "{user_input}"')

        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardClientNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
