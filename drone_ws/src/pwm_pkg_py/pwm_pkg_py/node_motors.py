#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
from std_msgs.msg import Int32MultiArray

# Pines BCM de la Raspberry (GPIO)
MOTOR_PINS = [17, 23, 27, 22]  # cámbialos según tu conexión
pi = pigpio.pi()

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'motor_speeds',   # nombre del tópico
            self.listener_callback,
            10)
        self.get_logger().info("Nodo de control de motores iniciado")

        # Inicializa pines
        for pin in MOTOR_PINS:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.set_servo_pulsewidth(pin, 0)  # Apaga al inicio

    def listener_callback(self, msg):
        motor_speeds = msg.data
        for i, speed in enumerate(motor_speeds):
            if i < len(MOTOR_PINS):
                # Limita rango típico ESC 1000–2000 µs
                speed = max(1000, min(2000, speed))
                pi.set_servo_pulsewidth(MOTOR_PINS[i], speed)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 0)
        pi.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
