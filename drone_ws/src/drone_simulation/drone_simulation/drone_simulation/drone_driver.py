import rclpy 
from geometry_msgs.msg import Twist

class MyDrone:
    def init(self,webots_node, properties):
        self.__robot= webots_node.robot

        self.__front_left_motor = self.__robot.getDevice("front left propeller")
        self.__front_right_motor = self.__robot.getDevice("front right propeller")
        self.__rear_left_motor = self.__robot.getDevice("rear left propeller")
        self.__rear_right_motor = self.__robot.getDevice("rear right propeller")

        
        self.__motors = [
            self.__front_left_motor,
            self.__front_right_motor,
            self.__rear_left_motor,
            self.__rear_right_motor
        ]
        for m in self.__motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_drone_driver')
        self.__target_twist = Twist()
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.z
        angular_speed = self.__target_twist.angular.z

        for m in self.__motors:
            m.setVelocity(forward_speed)









