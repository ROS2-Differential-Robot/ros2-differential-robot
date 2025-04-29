#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# MPU-6500 Register Addresses
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68  # Default I2C address for MPU-6500

# Sensitivity scale factors
ACCEL_SENSITIVITY = 16384.0  # LSB/g for ±2g
GYRO_SENSITIVITY = 16.4      # LSB/(°/s) for ±2000°/s


class MPU6500_Driver(Node):

    def __init__(self):
        super().__init__("mpu6500_driver")

        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/adam/imu", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01  # 100 Hz
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            # Read Accelerometer raw values
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw values
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Convert to physical units
            self.imu_msg_.linear_acceleration.x = acc_x / ACCEL_SENSITIVITY * 9.8
            self.imu_msg_.linear_acceleration.y = acc_y / ACCEL_SENSITIVITY * 9.8
            self.imu_msg_.linear_acceleration.z = acc_z / ACCEL_SENSITIVITY * 9.8
            self.imu_msg_.angular_velocity.x = gyro_x / GYRO_SENSITIVITY
            self.imu_msg_.angular_velocity.y = gyro_y / GYRO_SENSITIVITY
            self.imu_msg_.angular_velocity.z = gyro_z / GYRO_SENSITIVITY

            # Timestamp
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)            # Sample rate divider
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)           # Exit sleep mode
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)               # No DLPF
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0x18)       # ±2000 °/s
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0x00)      # ±2g
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)           # Enable data ready interrupt
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
            self.get_logger().info("Couldn't connect to MPU6500")

    def read_raw_data(self, addr):
        # Read high and low bytes
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr + 1)
        value = ((high << 8) | low)

        # Convert to signed 16-bit int
        if value > 32767:
            value -= 65536
        return value


def main():
    rclpy.init()
    mpu_driver = MPU6500_Driver()
    rclpy.spin(mpu_driver)
    mpu_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
