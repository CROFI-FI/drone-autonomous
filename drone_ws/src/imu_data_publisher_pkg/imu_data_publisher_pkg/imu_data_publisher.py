import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import time
import numpy as np

# MPU9250 addresses
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Magnetometer (AK8963) addresses
AK8963_ADDR = 0x0C
AK8963_WIA = 0x00
AK8963_ST1 = 0x02
AK8963_HXL = 0x03
AK8963_CNTL1 = 0x0A
AK8963_CNTL2 = 0x0B
AK8963_ASAX = 0x10

# Configuración de rangos (solo registro, no escala)
ACCEL_CONFIG = 0x1C  # ±2g
GYRO_CONFIG = 0x1B   # ±250°/s

def twos_complement(val, bits=16):
    if val & (1 << (bits-1)):
        return val - (1 << bits)
    return val

def read_imu(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    return twos_complement((high << 8) | low)

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_data_node')
        self.publisher_imu = self.create_publisher(Float32MultiArray, '/imu/raw_data', 10)

        self.bus = SMBus(1)

        # Despierta MPU
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Configura rangos crudos
        self.bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00)  # ±2g
        self.bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)   # ±250°/s

        # Habilita bypass para AK8963
        self.bus.write_byte_data(MPU_ADDR, INT_PIN_CFG, 0x02)
        time.sleep(0.05)

        # Inicializa AK8963
        whoami = self.bus.read_byte_data(AK8963_ADDR, AK8963_WIA)
        if whoami != 0x48:
            self.get_logger().warn(f"No se detecta AK8963, WHO_AM_I={hex(whoami)}")

        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL2, 0x01)
        time.sleep(0.1)
        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x0F)
        time.sleep(0.05)

        asax = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX)
        asay = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX+1)
        asaz = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX+2)
        self.adj_x = ((asax - 128) / 256.0) + 1.0
        self.adj_y = ((asay - 128) / 256.0) + 1.0
        self.adj_z = ((asaz - 128) / 256.0) + 1.0

        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x16)  # Modo continuo
        time.sleep(0.1)

        self.timer = self.create_timer(0.05, self.publish_imu_data)

    def publish_imu_data(self):
        try:
            # --- Datos crudos acelerómetro ---
            ax = float(read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H))
            ay = float(read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+2))
            az = float(read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+4))

            # --- Datos crudos giroscopio ---
            gx = float(read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H))
            gy = float(read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+2))
            gz = float(read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+4))

            # --- Datos crudos magnetómetro ---
            mx, my, mz = 0.0, 0.0, 0.0
            st1 = self.bus.read_byte_data(AK8963_ADDR, AK8963_ST1)
            if st1 & 0x01:
                data = self.bus.read_i2c_block_data(AK8963_ADDR, AK8963_HXL, 7)
                mx = twos_complement((data[1]<<8)|data[0],16) * self.adj_x
                my = twos_complement((data[3]<<8)|data[2],16) * self.adj_y
                mz = twos_complement((data[5]<<8)|data[4],16) * self.adj_z
                st2 = data[6]
                if st2 & 0x08:
                    self.get_logger().warn("Overflow magnetómetro, descartando muestra")
                    mx, my, mz = 0.0, 0.0, 0.0

            # --- Publica en float32 con 4 decimales ---
            valores = [ax, ay, az, gx, gy, gz, mx, my, mz]
            msg = Float32MultiArray()
            msg.data = [np.float32(round(v,4)) for v in valores]
            self.publisher_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error al leer MPU9250: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
