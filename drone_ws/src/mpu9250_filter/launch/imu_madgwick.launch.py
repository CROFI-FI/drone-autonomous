from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que publica la IMU cruda
        Node(
            package='mpu9250_filter',
            executable='mpu9250_filter',   # este nombre debe coincidir con el entry point en setup.py
            name='mpu9250_publisher',
            output='screen'
        ),
        # Nodo Madgwick de imu_tools
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            remappings=[
                ('imu/data_raw', '/imu/raw_data'),
                ('imu/data', '/imu/filtered_data')
            ],
            parameters=[
                {'frequency': 20.0},
                {'gain': 0.1}
            ]
        )
    ])
