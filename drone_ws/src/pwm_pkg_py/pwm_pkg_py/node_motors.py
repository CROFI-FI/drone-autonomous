#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
import time
from std_msgs.msg import Int32MultiArray

# Pines BCM de la Raspberry (GPIO)
MOTOR_PINS = [17, 23, 27, 22]  # cámbialos según tu conexión
pi = pigpio.pi()


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.get_logger().info("Iniciando nodo de control de motores...")
        self.last_msg_time = self.get_clock().now()  # Guarda el último tiempo de mensaje recibido
        self.setup_hardware()

        # Suscriptor
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'motor_speeds',
            self.listener_callback,
            1
        )

        # Timer para failsafe (revisa cada 0.1s)
        self.timer = self.create_timer(0.1, self.failsafe_check)

        self.get_logger().info("Nodo listo, esperando comandos en /motor_speeds")

    def setup_hardware(self):
        # Inicializa pines
        for pin in MOTOR_PINS:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.set_servo_pulsewidth(pin, 0)

        self.get_logger().info("Pines inicializados")

        # Armado de ESC
        self.get_logger().info("Armando motores (2000 → 1000 µs)...")
        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 2000)
        time.sleep(2)

        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 1000)
        time.sleep(2)

        self.get_logger().info("Motores armados y listos (esperando comandos)")

    def listener_callback(self, msg: Int32MultiArray):
        self.last_msg_time = self.get_clock().now()  # Actualiza tiempo del último mensaje
        motor_speeds = list(msg.data[:len(MOTOR_PINS)])
        motor_speeds += [1000] * (len(MOTOR_PINS) - len(motor_speeds))
        motor_speeds = [max(1000, min(2000, s)) for s in motor_speeds]

        for pin, speed in zip(MOTOR_PINS, motor_speeds):
            pi.set_servo_pulsewidth(pin, speed)

    def failsafe_check(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9  # en segundos
        if elapsed > 1.0:  # más de 1s sin datos
            for pin in MOTOR_PINS:
                pi.set_servo_pulsewidth(pin, 1000)
            self.get_logger().warn("Failsafe activado: motores al mínimo (1000 µs)")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

