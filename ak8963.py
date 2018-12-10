"""
Implementation of the MPU9250 9-DOF sensor chip
"""

import smbus
import time

from bitops import *

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
## Power down mode
MODE_DOWN = 0x00
## One shot data output
MODE_ONE = 0x01
## FuseROM access mode
MODE_FUSEROM = 0x0f

## Continous data output 8Hz
MODE_C8HZ = 0x02
## Continous data output 100Hz
MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
BIT_14 = 0x00
## 16bit output
BIT_16 = 0x01

## smbus
bus = smbus.SMBus(1)

## AK8963 I2C Control class
class AK8963:
    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configure()
    
    def configure(self):
        """Configure AK8963"""
        
        # set power down mode
        bus.write_byte_data(self.address, CNTL1, MODE_DOWN)
        time.sleep(0.01)

        # set read FuseROM mode
        bus.write_byte_data(self.address, CNTL1, MODE_FUSEROM)
        time.sleep(0.01)

        # read coef data
        data = bus.read_i2c_block_data(self.address, ASAX, 3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0

        # set power down mode
        bus.write_byte_data(self.address, CNTL1, MODE_DOWN)
        time.sleep(0.01)

        self.set_mode(BIT_16)

    def calibrate(self):
        """Calibrates sensors by writing offsets to specific registers"""

        mag_bias = [0, 0, 0]

        for i in range(512):
            mag = [0, 0, 0]
            mag_data = bus.read_i2c_block_data(self.address, HXL, 7)
            
            time.sleep(0.01)
#            print("Read data:", accel_data, gyro_data)
            
            for i in range(3):
                mag[i] = bytes_to_int16(mag_data[i*2], mag_data[i*2 + 1])
                mag_bias[i] += mag[i]
#            print("Read data (int):", accel, gyro)

        # Normalize sums to get average count biases
        for i in range(3):
            mag_bias[i] = mag_bias[i] / 512
        print("Calculated bias:", mag_bias)
        
        self.mag_bias = mag_bias

    def set_mode(self, mode):
        if mode == BIT_14:
            self.mres = 4912.0 / 8190.0
        elif mode == BIT_16:
            self.mres = 4912.0 / 32760.0
        else:
            print("MFS Mode not available {}. Using default (BIT_16).".format(mode))
            self.mres = 4912.0 / 32760.0

        # set scale&continous mode
        bus.write_byte_data(self.address, CNTL1, (mode << 4 | MODE_C8HZ))
        time.sleep(0.01)

    def read_magnet(self):
        """Read magnetometer data from AK8963_MAGNET_OUT register"""
        xyz = [0, 0, 0]

        # check data ready
        drdy = bus.read_byte_data(self.address, ST1)
        if drdy & 0x01 :
            data = bus.read_i2c_block_data(self.address, HXL, 7)

            # check overflow
            if (data[6] & 0x08) != 0x08:
                for i in range(3):
                    xyz[i] = bytes_to_int16(data[i*2], data[i*2 + 1])
                    xyz[i] *= (self.mres * self.magXcoef)
        return xyz
