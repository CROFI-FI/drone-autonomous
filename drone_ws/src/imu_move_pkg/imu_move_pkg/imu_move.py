import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
import serial
import math

class SimpleKalman:
    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P = [[0, 0], [0, 0]]

    def update(self, new_angle, new_rate, dt):
        # Predicción
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[1][0] - self.P[0][1] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_bias * dt

        # Medición
        S = self.P[0][0] + self.r_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.publisher_marker = self.create_publisher(Marker, '/imu_marker', 10)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.timer = self.create_timer(0.05, self.read_serial)

        self.accel_scale = 1.0
        self.gyro_scale = math.pi / 180.0  # °/s → rad/s
        self.marker_id = 0

        # Filtros Kalman
        self.kalman_roll = SimpleKalman()
        self.kalman_pitch = SimpleKalman()
        self.kalman_yaw = SimpleKalman()

        self.last_time = self.get_clock().now()
        self.prev_yaw = 0.0

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

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            try:
                ax, ay, az, gx, gy, gz, mx, my, mz = map(float, line.split(','))
            except ValueError:
                self.get_logger().warn(f"Línea malformada, ignorando: {line}")
                return

            now = self.get_clock().now()
            dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
            self.last_time = now

            # Roll y pitch del acelerómetro
            acc_roll = math.atan2(ay, az)
            acc_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

            # Velocidad angular del giroscopio (rad/s)
            gyro_x = gx * self.gyro_scale
            gyro_y = gy * self.gyro_scale
            gyro_z = gz * self.gyro_scale

            # Filtro Kalman roll/pitch
            roll = self.kalman_roll.update(acc_roll, gyro_x, dt)
            pitch = self.kalman_pitch.update(acc_pitch, gyro_y, dt)

            # Yaw con magnetómetro
            norm = math.sqrt(mx**2 + my**2 + mz**2)
            mx_n = mx / norm
            my_n = my / norm
            mz_n = mz / norm

            mag_x_comp = mx_n * math.cos(pitch) + mz_n * math.sin(pitch)
            mag_y_comp = mx_n * math.sin(roll)*math.sin(pitch) + my_n*math.cos(roll) - mz_n*math.sin(roll)*math.cos(pitch)

            raw_yaw = math.atan2(-mag_y_comp, mag_x_comp)

            # Corregir discontinuidad
            if raw_yaw - self.prev_yaw > math.pi:
                raw_yaw -= 2*math.pi
            elif raw_yaw - self.prev_yaw < -math.pi:
                raw_yaw += 2*math.pi
            self.prev_yaw = raw_yaw

            # Filtro Kalman yaw
            yaw = self.kalman_yaw.update(raw_yaw, gyro_z, dt)

            # Quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            # Publicar IMU
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.x = ax * 9.81
            imu_msg.linear_acceleration.y = ay * 9.81
            imu_msg.linear_acceleration.z = az * 9.81
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1
            self.publisher_imu.publish(imu_msg)

            # Marker en RViz
            marker = Marker()
            marker.header.frame_id = "imu_link"
            marker.header.stamp = now.to_msg()
            marker.ns = "drone_body"
            marker.id = self.marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
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
            self.get_logger().warn(f"Error al leer serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
