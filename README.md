# drone-autonomous
Proyecto en desarrollo de un dron autónomo diseñado por CROFI en colaboración con el laboratorio de Robótica de la Facultad de Ingeniería. Este sistema integra control de vuelo, sensorización y visión computacional para realizar tareas de transporte de objetos, navegación autónoma y evasión de obstáculos.

Guía para correr el nodo imu_box_pkg (visualizador del MPU6050/MPU9250 en RVIZ2):

Cargar codigo de ESP32 mediante Arduino IDE se encuentran en la carpeta Arduino.

Entrar a Drone_ws y correr colcon build junto con source install/setup.bash (entorno ros2)

Ejecutar el nodo de de box_node para usar sin magnetometro (MPU6050) Si se quiere usar el magnetometro (MPU9250) usar imu_move

En otra terminal ejecutar el entorno de ros2 y despues ejecutar Rviz2

CONFIGURACIÓN DE RVIZ2:
Seleccionar en fixed frame "imu_link"
Agregar un Marker y en topic seleccionar /imu_marker

Con esto ya se podría visualizar el movimeinto en tiempo real de la IMU en Rviz2.

