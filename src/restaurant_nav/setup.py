from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'restaurant_nav'

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
        (os.path.join('share', package_name, package_name), glob(os.path.join(package_name, '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed-hehsam',
    maintainer_email='ahmedhesham1652001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_table_num = restaurant_nav.publisher_table_num:main',
            'pose_recorder = restaurant_nav.pose_recorder:main',
            'goal_sender = restaurant_nav.goal_sender:main',
            "joystick_twist=neu_lidar.joystick_twist:main",
        ],
    },
)
