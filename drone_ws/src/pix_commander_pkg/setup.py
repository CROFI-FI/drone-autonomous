from setuptools import find_packages, setup

package_name = 'pix_commander_pkg'

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
    maintainer='quique',
    maintainer_email='enriquemedranorabak@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'commander = pix_commander_pkg.commander_node:main',
            'manual = pix_commander_pkg.manual_in:main',
        ],
    },
)
