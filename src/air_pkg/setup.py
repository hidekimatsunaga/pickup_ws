from setuptools import find_packages, setup

package_name = 'air_pkg'

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
    maintainer_email='matsunaga-h@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = air_pkg.hello_node:main',
            'arduino_reader = air_pkg.arduino_reader:main'
        ],
    },
)
