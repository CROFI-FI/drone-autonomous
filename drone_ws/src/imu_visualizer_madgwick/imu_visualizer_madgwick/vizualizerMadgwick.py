import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker

class Imu_NodeVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.subscription = self.create_subscription(Quaternion, '/imu/orientation',self.update ,10)

        #self.timer = self.create_timer(0.05, self.update)
        self.publisher = self.create_publisher(Marker, '/imu/cube_marker', 10)


    def update(self, msg):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation = msg
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = Imu_NodeVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

