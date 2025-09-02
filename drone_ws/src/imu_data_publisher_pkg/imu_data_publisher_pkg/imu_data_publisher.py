"""
Nodo publicador de lecturas de información en crudo del sensor MPU9250, usando ROS2

Publica los datos en /imu/raw_data
"""

#Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import math

#I2C directions of the MPU9250
MPU_ADDR = 0x68
PWR_MGMT = 0x68
ACCEL_XOUT_H = 0X3B #Aceleración en X
GYRO_XOUT_H =  0x43 #Velocidad angular en X

#function registers write
def read_imu(bus, addr, reg):
    """
    Esta función se encarga de leer los registros de la IMU, para combinarlos en enteros
    usando el complento a dos para la asignación de signos (+,-), de esta forma devuelve el valor crudo
    de la aceleación y velociadad angular.

    bus = bus de conexión y comunicación con I2C con la rasp
    addr = dirección del MPU9250 (por defecto 0x68(low) o 0x69(high))
    reg = registro interno del MPU (Viene en el Datasheet)

    """
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    value = (high << 8) + low
    if value >= 0x8000:
        value >= -((65535 - value) + 1)
    return value

class mpu_data_node(Node):
    """
    Clase se encarga de inicializar el bus de datos del I2C y el sensor MPU9250, así como de la creación
    del publicador publisher_imu de forma ordenada combinando el acelerometro y el giroscopio.

    Se tendran que suscribir a "/imu/raw_data" el cual publica los datos en crudo.
    """
    def __init__(self):
        super().__init__('mpu9250_data_node')
        self.publisher_imu = self.create_publisher(Float32MultiArray,'/imu/raw_data', 10)

        #I2C and MPU9250 inicialized
        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

        #published each 50 ms
        self.timer = self.create_timer(0.05, self.publish_imu_data)
    
    def publish_imu_data(self):
        try:
            #Acelerometer
            ax = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H)
            ay = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+2)
            az = read_imu(self.bus, MPU_ADDR, ACCEL_XOUT_H+4)
            
            #Gyroscope
            gx = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H)
            gy = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+2)
            gz = read_imu(self.bus, MPU_ADDR, GYRO_XOUT_H+4)

            msg = Float32MultiArray()
            #Published in order (ax, ay, az, gx, gy, gz)
            self.publisher_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error al leer MPU9250: {e}")


def main(args= None):
    rclpy.init(args=args)
    node = mpu_data_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

