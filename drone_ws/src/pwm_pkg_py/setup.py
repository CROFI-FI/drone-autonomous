from setuptools import find_packages, setup

package_name = 'pwm_pkg_py'

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
    maintainer='pumitas',
    maintainer_email='rosberry.drone@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'node_motors = pwm_pkg_py.node_motors:main',
        'sin_test = pwm_pkg_py.sin_test:main'
        ],
    },
)
