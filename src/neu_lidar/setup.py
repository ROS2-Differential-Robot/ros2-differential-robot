from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'neu_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.*'))),
        (os.path.join('share', package_name, 'model', 'world'), glob(os.path.join('model', 'world', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joystick_twist=neu_lidar.joystick_twist:main",
            'publisher_table_num = neu_lidar.publisher_table_num:main',
            'subscriber_table_num = neu_lidar.subscriber_table_num:main',
            'pose_recorder = neu_lidar.pose_recorder:main',
            'goal_sender = neu_lidar.goal_sender:main',
        ],
    },
)
