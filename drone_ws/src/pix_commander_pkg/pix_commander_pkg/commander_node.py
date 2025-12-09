import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
#Point para NED [X Y Z] -> se cambia por Twist NED [X y Z A]
from geometry_msgs.msg import Twist
import math

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_node')

        # QoS para PX4
        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS para comunicación interna
        qos_profile_internal = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores a PX4
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_px4)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_px4)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_px4)

        # Suscriptores
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_px4)
        
        # Suscriptor para recibir órdenes del nodo manual.py  
        self.cmd_subscriber = self.create_subscription(
            Twist, '/drone/cmd_pose', self.cmd_pose_callback, qos_profile_internal) #Se cambia por Twist

        # Variables de estado
        self.vehicle_status = VehicleStatus()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.counter = 0
        
        # Posición Objetivo Inicial NED: x=0, y=0, z=-2.0  (2 metros de altura) yaw = 0.0 
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -2.0 
        self.target_yaw = 0.0

        # Timer a 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Esperando conectar")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state

    # Callback cuando ejecutas el comando
    def cmd_pose_callback(self, msg):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        # Se convierte altura positiva a NED negativo sistema PX4
        self.target_z = -1.0 * abs(msg.linear.z)
        # Se convierte grados a radianes si envías grados desde manual
        yaw_deg = msg.angular.z
        self.target_yaw = yaw_deg * (math.pi / 180.0)
        self.get_logger().info(f"Moviendo a X:{self.target_x: .2f} Y:{self.target_y: .2f} Z:{self.target_z: .2f} A:{self.target_yaw: .2f}")


    # Sacado de la documentación de la PX4
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("ARMANDO motores")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            1.0, 
            6.0 
        )
        self.get_logger().info("Activando modo OFFBOARD")

    def timer_callback(self):
        # Heartbeat Offboard
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)

        # Secuencia de inicio
        if self.counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # Publicar SIEMPRE la posición objetivo actual
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj_msg.position = [self.target_x, self.target_y, self.target_z]
        traj_msg.yaw = self.target_yaw

        self.trajectory_setpoint_publisher.publish(traj_msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()