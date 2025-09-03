from setuptools import find_packages, setup

package_name = 'imu_offsets_pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humubuntu',
    maintainer_email='dragonoidhor@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_GyroAcc = imu_offsets_pkg_py.imu_GyroAcc:main',
            'imu_Mag = imu_offsets_pkg_py.imu_Mag:main',
        ],
    },
)
