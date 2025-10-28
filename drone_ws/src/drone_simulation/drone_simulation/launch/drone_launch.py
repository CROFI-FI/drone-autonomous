import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('drone_simulation')
    robot_description_path=os.path.join(package_dir, 'resource', 'my_drone.urdf') #URDF da las instruccioes (?)

    #Webots launcher
    webots=WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'drone_sim.wbt')
    )

    #DroneDriver node
    my_drone_driver=WebotsController(
        robot_name='pumas_drone',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    #DroneControl node
    drone_ros2_node=Node(
        package='drone_simulation',
        executable='drone_control',
        name='drone_control',
        output='screen' 
    )

    
    return LaunchDescription([
        webots,
        my_drone_driver,
        drone_ros2_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])