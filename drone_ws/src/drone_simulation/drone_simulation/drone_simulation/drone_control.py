import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class DroneRos2(Node):
    def __init__(self):
        super().__init__('drone_ros2')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.drone_callback)

    def drone_callback(self):
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 200.0
        msg.angular.z= 0.0

        self.publisher.publish(msg)

def main():
    rclpy.init()
    drone = DroneRos2()
    rclpy.spin(drone)        
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

