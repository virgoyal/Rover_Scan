from setuptools import setup
import os
from glob import glob

package_name = 'crack_scan_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simulates crack scan using rover, sensor, and visualization nodes.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_position_node = crack_scan_sim.rover_position_node:main',
            'sensor_position_node = crack_scan_sim.sensor_position_node:main',
            'sensor_value_node = crack_scan_sim.sensor_value_node:main',
            'matrix_visualization_node = crack_scan_sim.matrix_visualization_node:main',
            # add more nodes here as needed
        ],
    },
)