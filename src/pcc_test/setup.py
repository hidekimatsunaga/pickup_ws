from setuptools import find_packages, setup

package_name = 'pcc_test'

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
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pcc_move_node = pcc_test.pcc_move_node:main',
            'pcc_visualizer_node = pcc_test.pcc_visualizer_node:main',
            'pcc_target_publisher = pcc_test.pcc_target_publisher:main',
        ],
    },
)
