import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math 
import yaml
import numpy as np


class Imu_CalibrationGyroAcce(Node):
    def __init__(self):
        super().__init__('imu_gyroacc_offsets')
        self.subscription = self.create_subscription(Float32MultiArray, '/imu/raw_data', self.read_data, 10)
    
        #self.timer = self.create_timer(0.05, self.read_data)

        self.data_buffer = np.zeros((200, 6))
        self.count = 0
        self.calibrated = False
        self.i = 0

        self.ax_offset = 0
        self.ay_offset = 0
        self.az_offset = 0
        self.gx_offset = 0
        self.gy_offset = 0
        self.gz_offset = 0

    def read_data(self, msg):

        if self.calibrated:
            return

        data = msg.data 
        if len(data) < 6:
            self.get_logger().warn(f"Dato incompleto recibido: {data}")
            return

        self.data_buffer[self.count] = data[:6]
        self.count += 1
        self.get_logger().info(f"Muestra {self.count}/200 recopilada")

        if self.count == 200:
            self.calibration_callback()
            self.calibrated = True
            self.get_logger().info("Calibración, finalizada, guardando YAML")
            #self.save_offsets() 

    def calibration_callback(self):
        means = np.mean(self.data_buffer, axis=0)
        self.ax_offset, self.ay_offset, self.az_offset, self.gx_offset, self.gy_offset, self.gz_offset = means

        offsets = {
            "accelerometer": {
                "ax_offset": round(float(self.ax_offset), 4),
                "ay_offset": round(float(self.ay_offset), 4),
                "az_offset": round(float(self.az_offset), 4),
            },
            "gyroscope": {
                "gx_offset": round(float(self.gx_offset), 4),
                "gy_offset": round(float(self.gy_offset), 4),
                "gz_offset": round(float(self.gz_offset), 4),
            },
            "magnetometer": {
                "mx_offset": 0,
                "my_offset": 0,
                "mz_offset": 0,
            }
        }
        with open("imu_calibration.yaml","w") as f:
            yaml.dump(offsets, f, default_flow_style=False, sort_keys=False)


def main(args=None):
    rclpy.init(args=args)
    node = Imu_CalibrationGyroAcce()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
