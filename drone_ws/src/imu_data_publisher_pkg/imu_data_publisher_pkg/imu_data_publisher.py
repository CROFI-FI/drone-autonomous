# Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import time

# --- MPU9250 (MPU6500 + AK8963) constants ---
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
ACCEL_XOUT_H = 0X3B  # Aceleración en X
GYRO_XOUT_H = 0x43   # Velocidad angular en X

# --- Magnetometer (AK8963) constants ---
AK8963_ADDR = 0x0C
AK8963_WIA = 0x00
AK8963_ST1 = 0x02
AK8963_HXL = 0x03
AK8963_ST2 = 0x09
AK8963_CNTL1 = 0x0A
AK8963_CNTL2 = 0x0B
AK8963_ASAX = 0x10

def twos_complement(val, bits=16):
    if val & (1 << (bits-1)):
        return val - (1 << bits)
    return val

def read_imu(bus, addr, reg):
    """Lectura de 16 bits con complemento a dos"""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    value = (high << 8) + low
    return twos_complement(value, 16)

class mpu_data_node(Node):
    def __init__(self):
        super().__init__('mpu9250_data_node')
        self.publisher_imu = self.create_publisher(Float32MultiArray,'/imu/raw_data', 10)

        # Inicializa bus I2C
        self.bus = SMBus(1)

        # Despierta el MPU
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Habilita bypass para acceder al AK8963 directamente
        self.bus.write_byte_data(MPU_ADDR, INT_PIN_CFG, 0x02)
        time.sleep(0.05)

        # Comprueba WHO_AM_I del AK8963
        whoami = self.bus.read_byte_data(AK8963_ADDR, AK8963_WIA)
        if whoami != 0x48:
            self.get_logger().warn(f"No se detecta AK8963, WHO_AM_I={hex(whoami)}")

        # Reset AK8963
        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL2, 0x01)
        time.sleep(0.1)

        # Lee valores ASA de calibración interna
        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x0F)
        time.sleep(0.05)
        asax = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX)
        asay = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX+1)
        asaz = self.bus.read_byte_data(AK8963_ADDR, AK8963_ASAX+2)
        self.adj_x = ((asax - 128) / 256.0) + 1.0
        self.adj_y = ((asay - 128) / 256.0) + 1.0
        self.adj_z = ((asaz - 128) / 256.0) + 1.0

        # Pone AK8963 en modo continuo, 16-bit, 100Hz
        self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x16)
        time.sleep(0.1)

        # Publica cada 50 ms
        self.timer = self.create_timer(0.05, self.publish_imu_data)
    
    def publish_imu_data(self):
        try:
            # --- Acelerómetro ---
            ax = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H)
            ay = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+2)
            az = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+4)
            
            # --- Giroscopio ---
            gx = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H)
            gy = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+2)
            gz = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+4)

            # --- Magnetómetro ---
            mx, my, mz = 0.0, 0.0, 0.0
            st1 = self.bus.read_byte_data(AK8963_ADDR, AK8963_ST1)
            if st1 & 0x01:  # Data ready
                data = self.bus.read_i2c_block_data(AK8963_ADDR, AK8963_HXL, 7)
                mx = twos_complement((data[1] << 8) | data[0], 16) * self.adj_x
                my = twos_complement((data[3] << 8) | data[2], 16) * self.adj_y
                mz = twos_complement((data[5] << 8) | data[4], 16) * self.adj_z
                st2 = data[6]
                if st2 & 0x08:  # Overflow
                    self.get_logger().warn("Overflow en magnetómetro, descartando muestra")
                    mx, my, mz = 0.0, 0.0, 0.0

            # --- Publica ---
            msg = Float32MultiArray()
            msg.data = [float(ax), float(ay), float(az),
                        float(gx), float(gy), float(gz),
                        float(mx), float(my), float(mz)]
            self.publisher_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error al leer MPU9250: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = mpu_data_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
