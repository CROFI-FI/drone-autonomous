import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
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
        
        self.publisher_ = self.create_publisher(Point, '/drone/cmd_pose', qos_profile)
        
        # Esperando conexión
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        # Verificar argumentos
        if len(sys.argv) != 4:
            self.get_logger().error('Comando corrrecto: ros2 run pix_commander_pkg manual <x> <y> <z>')
            # self.get_logger().error('Ejemplo: ros2 run pix_commander_pkg manual 10.0 5.0 2.5')
            sys.exit(1)

        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
        except ValueError:
            self.get_logger().error('Los valores deben ser flotantes')
            sys.exit(1)

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(f'Coordenadas enviadas: Ir a X={x}, Y={y}, Z={z}')
        # Salir 
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node) # Esto se detendrá por el sys.exit(0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()