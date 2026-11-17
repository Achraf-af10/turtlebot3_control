"""Install the ROS hardware tracker, relay, configuration and launch files."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'trajectory_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Hardware velocity trajectory tracking for TurtleBot3 Burger',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_reader = trajectory_controller.odometry_reader:main',
            'tracking_node = trajectory_controller.tracking_node:main',
            'command_guard = trajectory_controller.command_guard:main',
        ],
    },
)
