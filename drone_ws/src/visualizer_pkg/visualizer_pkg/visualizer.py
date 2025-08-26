import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from smbus2 import SMBus
import math
import time

# Direcciones I2C del MPU9250
MPU_ADDR = 0x68
AK8963_ADDR = 0x0C  # Magnetómetro integrado

# Registros MPU9250
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    val = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.publisher_marker = self.create_publisher(Marker, '/imu_marker', 10)

        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)  # Despertar sensor

        self.timer = self.create_timer(0.05, self.update)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def update(self):
        try:
            ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H) / 16384.0
            ay = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H+2) / 16384.0
            az = read_word(self.bus, MPU_ADDR, ACCEL_XOUT_H+4) / 16384.0

            gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H) / 131.0
            gy = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H+2) / 131.0
            gz = read_word(self.bus, MPU_ADDR, GYRO_XOUT_H+4) / 131.0

            # Roll y pitch del acelerómetro
            acc_roll = math.atan2(ay, az)
            acc_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

            yaw = 0.0  # Luego añadimos magnetómetro y Kalman

            qx, qy, qz, qw = self.euler_to_quaternion(acc_roll, acc_pitch, yaw)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.x = ax * 9.81
            imu_msg.linear_acceleration.y = ay * 9.81
            imu_msg.linear_acceleration.z = az * 9.81
            imu_msg.angular_velocity.x = gx * math.pi/180
            imu_msg.angular_velocity.y = gy * math.pi/180
            imu_msg.angular_velocity.z = gz * math.pi/180
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.publisher_imu.publish(imu_msg)

            marker = Marker()
            marker.header.frame_id = "imu_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drone_body"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.5
            marker.scale.y = 0.3
            marker.scale.z = 0.15
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            self.publisher_marker.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"Error al leer MPU9250: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)