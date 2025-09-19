#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
import time
import math

# Pines BCM de la Raspberry (GPIO)
MOTOR_PINS = [17, 23, 27, 22]
pi = pigpio.pi()

class MotorSineNode(Node):
    def __init__(self):
        super().__init__('motor_sine_node')
        self.get_logger().info("Iniciando nodo senoidal para motores...")
        self.setup_hardware()

        # Timer: cada 0.05s (~20Hz) actualizamos la señal
        self.t = 0.0
        self.timer = self.create_timer(0.15, self.update_motors)

    def setup_hardware(self):
        # Inicializa pines
        for pin in MOTOR_PINS:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.set_servo_pulsewidth(pin, 0)

        self.get_logger().info("Armando motores (2000 → 1000 µs)...")
        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 2000)
        time.sleep(2)

        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 1000)
        time.sleep(2)

        self.get_logger().info("Motores armados, iniciando señal senoidal")

    def update_motors(self):
        # Señal senoidal entre 1000 y 2000 µs
        speed = 1500 + 500 * math.sin(self.t)
        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, speed)

        self.t += 0.1 # Avanza fase

    def shutdown(self):
        # Se llama al terminar el nodo para apagar motores
        self.get_logger().info("Apagando motores (1000 µs)...")
        for pin in MOTOR_PINS:
            pi.set_servo_pulsewidth(pin, 1000)
        pi.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSineNode()
    try:
        rclpy.spin(node) # Mantiene vivo el nodo hasta Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
