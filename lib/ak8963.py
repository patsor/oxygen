"""
Implementation of the MPU9250 9-DOF sensor chip
"""

import smbus
import time

from i2c import I2CDevice
from bitops import *
from math_func import mean

# AK8963 I2C slave address
SLAVE_ADDRESS = 0x0C
# Device ID
DEVICE_ID = 0x71

# AK8963 Register Addresses
WIA = 0x00
INFO = 0x01
ST1 = 0x02
HXL = 0x03
HXH = 0x04
HYL = 0x05
HYH = 0x06
HZL = 0x07
HZH = 0x08
ST2 = 0x09
CNTL1 = 0x0A
CNTL2 = 0x0B # do not access
ASTC = 0x0C
TS1 = 0x0D
TS2 = 0x0E
I2CDIS = 0x0F
ASAX = 0x10
ASAY = 0x11
ASAZ = 0x12

# CNTL1 Mode select
# Power down mode
MODE_DOWN = 0x00
# One shot data output
MODE_ONE = 0x01
# FuseROM access mode
MODE_FUSEROM = 0x0f

# Continous data output 8Hz
MODE_C8HZ = 0x02
# Continous data output 100Hz
MODE_C100HZ = 0x06

# Magneto Scale Select
# 14bit output
BIT_14 = 0x00
# 16bit output
BIT_16 = 0x01

mag_full_scale_dict = {
    0x00: 1.6673,
    0x01: 6.6694
}

bus = smbus.SMBus(1)

class AK8963:
    def __init__(self, address=SLAVE_ADDRESS):
        self.device = I2CDevice(address)
        self.mag_offs = [0.0, 0.0, 0.0]
        self.configure()
#        self.calibrate()

    def get_device_id(self):
        return self.device.read_register(WIA)

    def get_info(self):
        return self.device.read_register(INFO)

    def get_data_ready(self):
        return self.device.read_register_bit(ST1, 0)

    def get_data_overrun(self):
        return self.device.read_register_bit(ST1, 1)

    def read_mag_x_raw(self):
        data = self.device.read_register_burst(HXL, 2)
        return bytes_to_int16(data[0], data[1])

    def read_mag_y_raw(self):
        data = self.device.read_register_burst(HYL, 2)
        return bytes_to_int16(data[0], data[1])

    def read_mag_z_raw(self):
        data = self.device.read_register_burst(HZL, 2)
        return bytes_to_int16(data[0], data[1])

    def read_mag_raw(self):
        # check data ready
#        if self.get_data_ready():
        data = self.device.read_register_burst(HXL, 7)
        # check overflow
        if not (data[6] & 0x08):
            x = bytes_to_int16(data[0], data[1])
            y = bytes_to_int16(data[2], data[3])
            z = bytes_to_int16(data[4], data[5])
            return (x, y, z)
        return (0.0, 0.0, 0.0)

    def get_mag_overflow(self):
        return self.device.read_register_bit(ST2, 3)

    def get_bit_mode_from_st2(self):
        return self.device.read_register_bit(ST2, 4)

    def get_operation_mode(self):
        data = self.device.read_register(CNTL1)
        return data & 0x0f

    def set_operation_mode(self, value):
        data = self.device.read_register(CNTL1)
        val = set_bits(data, 0, 4, value)
        self.device.write_register(CNTL1, value)

    def get_bit_mode(self):
        return self.device.read_register_bit(CNTL1, 4)

    def set_bit_mode(self, value):
        self.device.write_register_bit(CNTL1, 4, value)

    def reset(self):
        self.device.write_register(CNTL2, 0x01)

    def set_i2c_disable(self):
        self.device.write_register(I2CDIS, 0x1b)

    def configure(self):
        """Configure AK8963"""
        
        # set power down mode
        self.set_operation_mode(MODE_DOWN)
        time.sleep(0.01)

        # set read FuseROM mode
        self.set_operation_mode(MODE_FUSEROM)
        time.sleep(0.01)

        self.sens_adj = self.get_sens_adj()

        # set power down mode
        self.set_operation_mode(MODE_DOWN)
        time.sleep(0.01)

        self.set_operation_mode(MODE_C100HZ)
        time.sleep(0.01)

        self.set_mag_mode(BIT_16)

    def get_sens_adj(self):
        """Get sensitivity adjustment values."""
        data = self.device.read_register_burst(ASAX, 3)
        sens_adj = [1.0, 1.0, 1.0]
        for i in range(3):
            sens_adj[i] += (data[i] - 128) / 256.0
        return sens_adj

    def set_mag_mode(self, mode):
        try:
            self.mres = mag_full_scale_dict[mode]
        except KeyError:
            self.mres = mag_full_scale_dict[BIT_16]
            mode = BIT_16

        self.set_bit_mode(mode)


    # TODO: Fix calibration, save values to external file
    def calibrate(self, num_samples=512):
        """Calibrates sensors by writing offsets to specific registers"""

        print("Calibrating, please do not move sensor...")
        
        mag_buff = [[], [], []]
        mean_mag = [0.0, 0.0, 0.0]

        time.sleep(1.0)
        for i in range(num_samples):
            mag = self.read_mag_raw()
            time.sleep(0.01)
            
            for i in range(3):
                mag_buff[i].append(mag[i])

        # Normalize sums to get average count biases
        for i in range(3):
            mean_mag[i] = mean(mag_buff[i])

        print("Done.")
        print("MX (min={};mean={};max={}".format(
            min(mag_buff[0]),
            mean_mag[0],
            max(mag_buff[0])))
        print("MY (min={};mean={};max={}".format(
            min(mag_buff[1]),
            mean_mag[1],
            max(mag_buff[1])))
        print("MZ (min={};mean={};max={}".format(
            min(mag_buff[2]),
            mean_mag[2],
            max(mag_buff[2])))
        
        self.mag_offs = mean_mag

    def read_magnet(self):
        """Read magnetometer data from AK8963_MAGNET_OUT register"""
        _x, _y, _z = self.read_mag_raw()
        x = (_x - self.mag_offs[0]) / self.mres * self.sens_adj[0]
        y = (_y - self.mag_offs[1]) / self.mres * self.sens_adj[1]
        z = (_z - self.mag_offs[2]) / self.mres * self.sens_adj[2]
        
        return (x, y, z)
