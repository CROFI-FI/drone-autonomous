import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ahrs.filters import Madgwick
from geometry_msgs.msg import Quaternion
import numpy as np 
import yaml

class Imu_Madgwick(Node):
    def __init__(self):
        super().__init__('imu_madgwick')
        self.subscription = self.create_subscription(Float32MultiArray, '/imu/raw_data', 10)
        
        with open("imu_calibration.yaml","r") as f:
            calib = yaml.safe_load(f)
        offsets_data = np.array([
            calib["accelerometer"]["ax_offset"],
            calib["accelerometer"]["ay_offset"],
            calib["accelerometer"]["az_offset"],
            calib["gyroscope"]["gx_offset"],
            calib["gyroscope"]["gy_offset"],
            calib["gyroscope"]["gz_offset"],
            calib["magnetometer"]["mx_offset"],
            calib["magnetometer"]["my_offset"],
            calib["magnetometer"]["mz_offset"]
        ])
        self.accel_scale = 16384.0
        self.gyro_scale = 131.0 
        self.mag_scale = 0.6

        ax = (offsets_data[0] / self.accel_scale) * 9.81
        ay = (offsets_data[1] / self.accel_scale) * 9.81
        az = (offsets_data[2] / self.accel_scale) * 9.81
        
        gx = (offsets_data[3] / self.gyro_scale) * 180.0
        gy = (offsets_data[4] / self.gyro_scale) * 180.0
        gz = (offsets_data[5] / self.gyro_scale) * 180.0
        
        mx = offsets_data[6] * self.mag_scale
        my = offsets_data[7] * self.mag_scale
        mz = offsets_data[8] * self.mag_scale

        self.data_offcal = [ax, ay, az, gx, gy, gz, mx, my, mz]
        
        self.madgwick = Madgwick(frecuency = 20.0)
        self.Q = np.array([1.0, 0.0, 0.0, 0.0])

        self.pub_quat = self.create_publisher(Quaternion, '/imu/orientation', 10)
        
        self.timer = self.create_timer(0.05, self.read_data)

    
    def read_data(self, msg):
        
        data = msg.data 
        if len(data) < 6:
            return
        
        AX = (data[0] / self.accel_scale ) * 9.81 - self.data_offcal[0]
        AY = (data[1] / self.accel_scale ) * 9.81 - self.data_offcal[1]
        AZ = (data[2] / self.accel_scale ) * 9.81 - self.data_offcal[2]
        
        GX = (data[3] / self.gyro_scale ) * (np.pi/180) - self.data_offcal[3]
        GY = (data[4] / self.gyro_scale ) * (np.pi/180) - self.data_offcal[4]
        GZ = (data[5] / self.gyro_scale ) * (np.pi/180) - self.data_offcal[5]

        MX = (data[6] / self.mag_scale ) - self.data_offcal[6]
        MY = (data[7] / self.mag_scale ) - self.data_offcal[7]
        MZ = (data[8] / self.mag_scale ) - self.data_offcal[8]

        acc = np.array([AX, AY, AZ])
        acc_norm = np.lianlg.norm(acc)
        if acc_norm > 0:
            acc /= acc_norm
        gyr = np.array([GX, GY, GZ])
        mag = np.array([MX, MY, MZ])

        self.Q = self.madgwick.updateMARG(self.Q, gyr=gyr, acc=acc, mag=mag)
        
        quat_msg = Quaternion()
        quat_msg.w = float(self.Q[0])
        quat_msg.x = float(self.Q[1])
        quat_msg.y = float(self.Q[2])
        quat_msg.z = float(self.Q[3])

        self.pub_quat.publish(quat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Imu_Madgwick()
    rclpy.spin(node)