import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time 
import yaml
import os
import numpy as np

class Imu_CalibrationMagno(Node):
    def __init__(self):
        super().__init__('imu_magno_offsets')
        self.subscription = self.create_subscription(Float32MultiArray, '/imu/raw_data', 10)
        self.timer = self.create_timer(0.05, self.read_data)

        self.mag_data = []
        self.start_time = time.time()
        self.mag_duration = 30.0
        self.calibrated = False

        self.mx_offset = 0
        self.my_offset = 0
        self.mz_offset = 0

    def read_data(self, msg):

        if self.calibrated:
            return
        data = msg.data 
        if len(data) < 9:
            self.get_logger().warn(f"Dato incompleto recibido: {data}")
            return
        
        mx, my, mz = data[6], data[7], data[8]
        self.mag_data.append([mx, my, mz])
        
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Recolectando mag: {elapsed:.1f}/{self.mag_duration:.1} s")

        if elapsed >= self.mag_duration: 
            self.calibration_callback()
            self.calibrated = True
            self.get_logger().info("Calibración finalizada")

    
    def calibration_callback(self):
        mag_data = np.array(self.mag_data)

        mx_min, my_min, mz_min = np.min(mag_data, axis=0)
        mx_max, my_max, mz_max = np.max(mag_data, axis=0)

        self.mx_offset = (mx_max + mx_min) / 2.0
        self.my_offset = (my_max + my_min) / 2.0
        self.mz_offset = (mz_max + mz_min) / 2.0


        yaml_file = "imu_calibration.yaml"
        if os.path.exists(yaml_file):
            with open(yaml_file, "r") as f:
                calibration_data = yaml.safe_load(f)
        else:
            calibration_data = {}
        
        calibration_data["magnetometer"] = {
                "mx_offset": self.mx_offset,
                "my_offset": self.my_offset,
                "mz_offset": self.mz_offset,
            }
            
        with open(yaml_file,"w") as f:
            yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(5)
    node = Imu_CalibrationMagno()
    while rclpy.ok() and not node.calibrated:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("Cerrando nodo...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()