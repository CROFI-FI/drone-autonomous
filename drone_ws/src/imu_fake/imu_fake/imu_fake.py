import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Imu  # type: ignore
from std_msgs.msg import Header  # type: ignore
from geometry_msgs.msg import Quaternion, Vector3  # type: ignore
import math
import sys


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return (qx, qy, qz, qw)


class MPUData(Node):
    def __init__(self):
        super().__init__('mpu9250_fake')
        self.publisher_imu = self.create_publisher(Imu, '/imu/raw_data', 10)
        self.frame_id = "imu_link"

        self.get_logger().info("Nodo Fake IMU iniciado")
        self.get_logger().info("Escribir roll pitch yaw (en grados)")

        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.timer = self.create_timer(0.05, self.imu_fake_callback)

    def imu_fake_callback(self):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        r = math.radians(self.roll)
        p = math.radians(self.pitch)
        y = math.radians(self.yaw)

        qx, qy, qz, qw = euler_to_quaternion(r, p, y)

        imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        imu_msg.angular_velocity = Vector3()
        imu_msg.linear_acceleration = Vector3()

        self.publisher_imu.publish(imu_msg)

    def set_orientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = MPUData()

    try:
        while rclpy.ok():
            line = sys.stdin.readline().strip()
            if not line:
                continue
            try:
                roll, pitch, yaw = map(float, line.split())
                node.set_orientation(roll, pitch, yaw)
                node.get_logger().info(f"Orientación actualizada -> Roll: {roll}°, Pitch: {pitch}°, Yaw: {yaw}°")
            except ValueError:
                node.get_logger().warn("Formato inválido. Usa: roll pitch yaw (en grados)")
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
