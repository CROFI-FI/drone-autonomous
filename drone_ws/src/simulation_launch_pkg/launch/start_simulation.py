import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Detección Automática de Entorno
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'
    
    # Obtener el 'share directory' de este paquete
    simulation_pkg_share_dir = get_package_share_directory('simulation_launch_pkg')
    
    # share_pkg -> share -> pkg -> install -> ws_root
    # Busca automaticamente el workspace
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(simulation_pkg_share_dir))))
    
    repo_root = os.path.dirname(ws_root)
    
    # Definir rutas fijas relativas a la raíz del repositorio
    # Por si no esta en home
    px4_autopilot_dir = os.path.join(repo_root, 'PX4-Autopilot')
    px4_ros_com_ws_path = os.path.join(repo_root, 'px4_ros_com_ws')
    px4_ros_com_setup_path = os.path.join(px4_ros_com_ws_path, 'install', 'setup.bash')
    drone_ws_setup_path = os.path.join(ws_root, 'install', 'setup.bash')


    # Argumento de Lanzamiento solo para QGroundControl
    qgc_path_arg = DeclareLaunchArgument(
        'qgc_path',
        default_value='QGroundControl',
        description='Comando o ruta completa al ejecutable de QGroundControl.'
    )
    qgc_path = LaunchConfiguration('qgc_path')


    # Lanzamiento PX4 SITL + Gazebo
    start_sitl_cmd = ExecuteProcess(
        cmd=['bash', '-c', 
             f'source {ros_setup_path} && '
             f'source {px4_ros_com_setup_path} && '
             f'make px4_sitl gz_x500'
            ],
        cwd=px4_autopilot_dir, 
        output='screen',
    )

    # Iniciar QGroundControl
    start_qgc_cmd = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[qgc_path], 
                output='screen',
            )
        ]
    )

    # Iniciar el Agente MicroXRCE
    start_microxrce_agent_cmd = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     f'source {ros_setup_path} && '
                     f'source {px4_ros_com_setup_path} && '
                     f'MicroXRCEAgent udp4 -p 8888'
                    ],
                output='screen',
            )
        ]
    )

    # Lanzar el nodo de prueba offboard_control
    start_offboard_control_cmd = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     f'source {ros_setup_path} && '
                     f'source {px4_ros_com_setup_path} && '
                     f'source {drone_ws_setup_path} && '
                     f'ros2 run pix_commander_pkg commander'
                    #  f'ros2 run px4_ros_com offboard_control' #Depende del nodo se puede cambiar el nodo iniciado
                    ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        qgc_path_arg,
        start_sitl_cmd,
        start_qgc_cmd,
        start_microxrce_agent_cmd,
        start_offboard_control_cmd
    ])