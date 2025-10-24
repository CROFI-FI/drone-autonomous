# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     config_path = os.path.join(
#             get_package_share_directory('imu_fake_p'),
#             'config',
#             'imu_config.rviz'
#         )
#     return LaunchDescription([
#         Node(
#             package='imu_fake_p',
#             namespace='imu_fake_p',
#             executable='imu_fake_p',
#             output='screen'
#         ),
#         Node(
#             package='imu_filter_madgwick',
#             executable='imu_filter_madgwick_node',
#             name='imu_filter',
#             output='screen',
#             remappings=[
#                 ('imu/data_raw', 'imu/raw_data'),
#                 ('imu/data', '/imu/data')
#             ]
#         ),
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d',config_path]
#         )
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__),  # carpeta launch
        '..', 'urdf', 'imu_box.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='imu_fake_p',
            executable='imu_fake_p',
            name='imu_fake_node_p',
            output='screen'
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                "use_mag": False,
                "world_frame": "enu"
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
