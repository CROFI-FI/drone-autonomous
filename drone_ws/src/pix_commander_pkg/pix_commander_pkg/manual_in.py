import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #Se cambia Point por Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys

class InputNode(Node):
    def __init__(self):
        super().__init__('manual_input_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher_ = self.create_publisher(Twist, '/drone/cmd_pose', qos_profile)
        
        # Esperando conexión
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        # Verificar argumentos
        if len(sys.argv) != 5:
            self.get_logger().error('Comando corrrecto: ros2 run pix_commander_pkg manual X Y Z Angulo')
            sys.exit(1)

        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
            yaw = float(sys.argv[4])
        except ValueError:
            self.get_logger().error('Los valores deben ser flotantes')
            sys.exit(1)

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = yaw

        self.publisher_.publish(msg)
        self.get_logger().info(f'Coordenadas enviadas: Ir a X={x}, Y={y}, Z={z} A={yaw}')
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()