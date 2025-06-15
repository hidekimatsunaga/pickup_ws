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
            'motor_angle_publisher = serial_transciever.motor_angle_publisher:main'
        ],
    },
)
