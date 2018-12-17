"""
Implementation of the MPU9250 9-DOF sensor chip
"""

from ak8963 import AK8963
from mpu6500 import MPU6500

class MPU9250:

    def __init__(self, fifo_mode=False):
        self.mpu6500 = MPU6500(0x68, fifo_mode)
        self.ak8963 = AK8963()

    def read_accel(self):
        """Read accelerometer data from ACCEL_OUT register"""
        return self.mpu6500.read_accel()

    def read_gyro(self):
        """Read gyroscope data from GYRO_OUT register"""
        return self.mpu6500.read_gyro()

    def read_magnet(self):
        """Read magnetometer data from AK8963_MAGNET_OUT register"""
        return self.ak8963.read_magnet()

    def read_temperature(self):
        """Read temperature in degrees C"""
        return self.mpu6500.read_temperature()
