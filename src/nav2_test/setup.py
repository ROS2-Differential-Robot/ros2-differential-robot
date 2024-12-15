from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_test'

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
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='aliishomie@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "twist_stamper = nav2_test.twist_stamper:main",
            "initial_pose_publisher = nav2_test.initial_pose_publisher:main"
        ],
    },
)
