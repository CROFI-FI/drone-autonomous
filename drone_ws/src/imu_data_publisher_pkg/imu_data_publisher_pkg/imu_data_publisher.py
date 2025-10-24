import rclpy  # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import Imu  # type: ignore
from std_msgs.msg import Header # type: ignore
from geometry_msgs.msg import Quaternion, Vector3 # type: ignore
from smbus2 import SMBus # type: ignore
import math 
import struct
import float64

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE_FACTOR = 16384.0 #LSB para un rango de +-2g 
GYRO_SCALE_FACTOR = 131.0 #LSB deg/s para un rango de +-250 deg/s


def read_imu(bus,addr,reg):
    high = bus.read_byte_data(addr,reg)
    low = bus.read_byte_data(addr,reg +1)
    byte_data = bytes([high, low])
    #Se define que se trabaja con big-endian y entero de 16 bits con signo
    value, = struct.unpack('>h', byte_data)
    return value

class MPU_data_node(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher_node')

        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

        self.timer = self.create_timer(0.05, self.imu_data_callback)
        self.frame_id = "imu_link"

    def imu_data_callback(self):
        #Nuevo mensaje Imu
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        #Acelerometer
        ax_raw = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H)
        ay_raw = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+2)
        az_raw = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+4)

        ax = (ax_raw / ACCEL_SCALE_FACTOR) * 9.80665
        ay = (ay_raw / ACCEL_SCALE_FACTOR) * 9.80665
        az = (az_raw / ACCEL_SCALE_FACTOR) * 9.80665

        #Gyroscope
        gx_raw = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H)
        gy_raw = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+2)
        gz_raw = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+4)

        gx = (gx_raw / GYRO_SCALE_FACTOR) * (math.pi / 180)
        gy = (gy_raw / GYRO_SCALE_FACTOR) * (math.pi / 180)
        gz = (gz_raw / GYRO_SCALE_FACTOR) * (math.pi / 180)

        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = float64(ax)
        imu_msg.linear_acceleration.y = float64(ay)
        imu_msg.linear_acceleration.z = float64(az)

        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = float64(gx)
        imu_msg.angular_velocity.y = float64(gy)
        imu_msg.angular_velocity.z = float64(gz)

        imu_msg.orientation = Quaternion()
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        #Establecer la covarianza 
        imu_msg.orientation_covariance = [-1.0] * 9
        imu_msg.angular_velocity_covariance = [-1.0] * 9
        imu_msg.linear_acceleration_covariance = [-1.0] * 9

        self.publisher_imu.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU_data_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
        
        

