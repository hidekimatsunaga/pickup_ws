from setuptools import find_packages, setup

package_name = 'serial_transciever'

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
    maintainer='matsunaga-h',
    maintainer_email='hide.matsuhide0312@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = serial_transciever.serial_node_test:main',
            'angle_serial_node = serial_transciever.angle_serial_node:main',
            'motor_angle_publisher = serial_transciever.motor_angle_publisher:main',
            'angle_serial_manual_node = serial_transciever.angle_serial_manual_node:main',
            'angle_arucopose_csv = serial_transciever.angle_arucopose_csv:main',
            'joy_offset_command_node = serial_transciever.joy_angle_command_node:main',
            'chokudo_cameraswing_air_serial_node = serial_transciever.chokudo_cameraswing_air_serial_node:main',
            'motor_manual_chokudo_node = serial_transciever.motor_manual_chokudo_node:main',
            'relay_controller = serial_transciever.relay_controller:main',
            'allmotor_manual_switch_node = serial_transciever.allmotor_manual_switch_node:main',
            'flag_relay_bridge = serial_transciever.flag_relay_bridge:main',
            'auto_motor_publisher = serial_transciever.auto_motor_publisher:main',
        ],
    },
)
