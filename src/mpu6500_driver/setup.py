from setuptools import find_packages, setup

package_name = 'mpu6500_driver'

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
    maintainer='ubuntu',
    maintainer_email='aliishomie@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6500_driver = mpu6500_driver.mpu6500_driver:main',
            'mpu6500_driver_2 = mpu6500_driver.mpu6500_driver_2:main',
        ],
    },
)
