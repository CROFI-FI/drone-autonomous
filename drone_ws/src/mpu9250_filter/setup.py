from setuptools import setup

package_name = 'mpu9250_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de launch
        ('share/' + package_name + '/launch', ['launch/imu_madgwick.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_correo@example.com',
    description='Nodo publisher para MPU9250 y filtro Madgwick usando imu_tools',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Esto hace que puedas correr:
            # ros2 run mpu9250_filter mpu9250_filter
            'mpu9250_filter = mpu9250_filter.mpu9250_filter:main',
        ],
    },
)
