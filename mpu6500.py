"""
Implementation of the MPU9250 9-DOF sensor chip
"""

import smbus
import time

from bitops import *

# MPU9250 Default I2C slave address
SLAVE_ADDRESS = 0x68
# Device ID
DEVICE_ID = 0x71

# MPU-9250 Register Addresses
# Gyroscope Self-Test Registers - Serial IF: R/W, Reset value: 0x00
SELF_TEST_X_GYRO = 0x00
SELF_TEST_Y_GYRO = 0x01
SELF_TEST_Z_GYRO = 0x02
# Accelerometer Self-Test Registers - Serial IF: R/W, Reset value: 0x00
SELF_TEST_X_ACCEL = 0x0D
SELF_TEST_Y_ACCEL = 0x0E
SELF_TEST_Z_ACCEL = 0x0F
# Gyro Offset Registers - Serial IF: R/W, Reset value: 0x00
XG_OFFSET_H = 0x13
XG_OFFSET_L = 0x14
YG_OFFSET_H = 0x15
YG_OFFSET_L = 0x16
ZG_OFFSET_H = 0x17
ZG_OFFSET_L = 0x18
# Sample Rate Divider - Serial IF: R/W, Reset value: 0x00
# Divides the internal sample rate (see register CONFIG) to generate the
# sample rate that controls sensor data output rate, FIFO sample rate.
# NOTE: This register is only effective when Fchoice = b'11
# (fchoice_b register bits are b'00), and (0 < dlpf_cfg < 7),
# such that the average filter's
# output is selected (see chart below).
# This is the update rate of sensor register.
# SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
SMPLRT_DIV = 0x19
# Configuration
# Bit    Name          Function
# [7]    -             Reserved
# [6]    FIFO_MODE     When set to "1", when the fifo is full, additional writes will not be written to fifo.
#                      When set to "0", when the fifo is full, additional writes will be written to the fifo,
#                      replacing the oldest data.
# [5:3]  EXT_SYNC_SET  Enables the FSYNC pin data to be sampled.
#                      EXT_SYNC_SET   FSYNC bit location
#                      0              function disabled
#                      1              TEMP_OUT_L[0]
#                      2              GYRO_XOUT_L[0]
#                      3              GYRO_YOUT_L[0]
#                      4              GYRO_ZOUT_L[0]
#                      5              ACCEL_XOUT_L[0]
#                      6              ACCEL_YOUT_L[0]
#                      7              ACCEL_ZOUT_L[0]
#                      Fsync will be latched to capture short strobes. This will be done such that if
#                      Fsync toggles, the latched value toggles, but won't toggle again until the new
#                      latched value is captured by the sample rate strobe. This is a requirement for
#                      working with some 3rd party devices that have fsync strobes shorter than our
#                      sample rate.
# [2:0] DLPF_CFG       For the DLPF to be used, fchoice[1:0] must be set to b'11,
#                      fchoice_b[1:0] is b'00.
# FCHOICE DLPF_CFG Gyroscope                            Temperature Sensor
# <1> <0>          Bandwidth (Hz) Delay (ms)  Fs (kHz)   Bandwidth (Hz) Delay (ms)
#  x   0     x     8800            0.064      32         4000            0.04
#  0   1     x     3600            0.11       32         4000            0.04
#  1   1     0      250            0.97        8         4000            0.04
#  1   1     1      184            2.9         1          188            1.9
#  1   1     2       92            3.9         1           98            2.8
#  1   1     3       41            5.9         1           42            4.8
#  1   1     4       20            9.9         1           20            8.3
#  1   1     5       10           17.85        1           10           13.4
#  1   1     6        5           33.48        1            5           18.6
#  1   1     7     3600            0.17        8         4000            0.04
CONFIG = 0x1A
# Gyroscope Configuration - Serial IF: R/W, Reset value: 0x00
# Bit   Name              Function
# [7]   XGYRO_Cten        X Gyro self-test
# [6]   YGYRO_Cten        Y Gyro self-test
# [5]   ZGYRO_Cten        Z Gyro self-test
# [4:3] GYRO_FS_SEL[1:0]  Gyro Full Scale Select:
#                         00 = +250dps
#                         01= +500 dps
#                         10 = +1000 dps
#                         11 = +2000 dps
# [2]   -                 Reserved
# [1:0] Fchoice_b[1:0]    Used to bypass DLPF as shown in table 1 above. NOTE:
#                         Register is Fchoice_b (inverted version of Fchoice), table 1 uses
#                         Fchoice (which is the inverted version of this register).
GYRO_CONFIG = 0x1B
# Accelerometer Configuration - Serial IF: R/W, Reset value: 0x00
# Bit   Name                Function
# [7]   ax_st_en            X Accel self-test
# [6]   ay_st_en            Y Accel self-test
# [5]   az_st_en            Z Accel self-test
# [4:3] ACCEL_FS_SEL[1:0]   Accel Full Scale Select:
#                           +/-2g (00), +/-4g (01), +/-8g (10), +/-16g (11)
# [2:0] -                   Reserved
ACCEL_CONFIG = 0x1C
# Accelerometer Configuration 2 - Serial IF: R/W, Reset value: 0x00
# Bit   Name                Function
# [7:6] -                   Reserved
# [5:4] -                   Reserved
# [3]   accel_fchoice_b     Used to bypass DLPF as shown in table 2 below. NOTE: This register
#                           contains accel_fchoice_b (the inverted version of accel_fchoice as
#                           described in the table below).
# [2:0] A_DLPFCFG           Accelerometer low pass filter setting as shown in table 2 below. 
ACCEL_CONFIG_2 = 0x1D
# Low Power Accelerometer ODR Control - Serial IF: R/W, Reset value: 0x00
LP_ACCEL_ODR = 0x1E
# Wake on Motion Threshold - Serial IF: R/W, Reset value: 0x00
# Bit   Name                Function
# [7:0] WOM_Threshold       This register holds the threshold value for the Wake on Motion Interrupt for
#                           accel x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg.
WOM_THR = 0x1F
# FIFO Enable - Serial IF: R/W, Reset value: 0x00
# Bit   Name                Function
# [7]   TEMP_OUT            1 - Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate
#                           If enabled, buffering of data occurs even if data path is in standby.
#                           0 - function is disabled
# [6]   GYRO_XOUT           1 - Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample
#                           rate; If enabled, buffering of data occurs even if data path is in standby.
#                           0 - function is disabled
# [5]   GYRO_YOUT           1 - Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample
#                           rate; If enabled, buffering of data occurs even if data path is in standby.
#                           0 - function is disabled
#                           NOTE: Enabling any one of the bits corresponding to the Gyros or Temp data paths,
#                           data is buffered into the FIFO even though that data path is not enabled.
# [4]   GYRO_ZOUT           1 - Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample
#                           rate; If enabled, buffering of data occurs even if data path is in standby.
#                           0 - function is disabled
# [3]   ACCEL               1 - write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
#                           ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at
#                           the sample rate;
#                           0 - function is disabled
# [2]   SLV_2               1 - write EXT_SENS_DATA registers associated to SLV_2 (as determined by
#                           I2C_SLV0_CTRL, I2C_SLV1_CTRL, and I2C_SL20_CTRL) to the FIFO at
#                           the sample rate;
#                           0 - function is disabled
# [1]   SLV_1               1 - write EXT_SENS_DATA registers associated to SLV_1 (as determined by
#                           I2C_SLV0_CTRL and I2C_SLV1_CTRL) to the FIFO at the sample rate;
#                           0 - function is disabled
# [0]   SLV_0               1 - write EXT_SENS_DATA registers associated to SLV_0 (as determined by
#                           I2C_SLV0_CTRL) to the FIFO at the sample rate;
#                           0 - function is disabled
#                           NOTE: See I2C_SLV3_CTRL register to enable this feature for SLV_3
FIFO_EN = 0x23
# I2C Master Control - Serial IF: R/W, Reset value: 0x00
I2C_MST_CTRL = 0x24
# I2C Slave 0 Control
I2C_SLV0_ADDR = 0x25
I2C_SLV0_REG = 0x26
I2C_SLV0_CTRL = 0x27
# I2C Slave 1 Control
I2C_SLV1_ADDR = 0x28
I2C_SLV1_REG = 0x29
I2C_SLV1_CTRL = 0x2A
# I2C Slave 2 Control
I2C_SLV2_ADDR = 0x2B
I2C_SLV2_REG = 0x2C
I2C_SLV2_CTRL = 0x2D
# I2C Slave 3 Control
I2C_SLV3_ADDR = 0x2E
I2C_SLV3_REG = 0x2F
I2C_SLV3_CTRL = 0x30
# I2C Slave 4 Control
I2C_SLV4_ADDR = 0x31
I2C_SLV4_REG = 0x32
I2C_SLV4_DO = 0x33
I2C_SLV4_CTRL = 0x34
I2C_SLV4_DI = 0x35
# I2C Master Status - Serial IF: R/C, Reset value: 0x00
I2C_MST_STATUS = 0x36
# INT Pin / Bypass Enable Configuration - Serial IF: R/W, Reset value: 0x00
INT_PIN_CFG = 0x37
# Interrupt Enable - Serial IF: R/W, Reset value: 0x00
INT_ENABLE = 0x38
# Interrupt Status - Serial IF: R/C, Reset value: 0x00
INT_STATUS = 0x3A
# Accelerometer Measurements - Serial IF: SyncR, Reset value: 0x00 (if sensor disabled)
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40
# Temperature Measurement - Serial IF: SyncR, Reset value: 0x00 (if sensor disabled)
# TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21degC
# Where Temp_degC is the temperature in degrees C
# measured by the temperature sensor. TEMP_OUT is
# the actual output of the temperature sensor.
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
# Gyroscope Measurements - Serial IF: SyncR, Reset value: 0x00 (if sensor disabled)
# GYRO_XYZOUT = Gyro_Sensitivity * XYZ_angular_rate
# Nominal FS_SEL = 0
# Conditions Gyro_Sensitivity = 131 LSB/(deg/s)
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48
# External Sensor Data - Serial IF: SyncR, Reset value: 0x00
EXT_SENS_DATA_00 = 0x49
EXT_SENS_DATA_01 = 0x4A
EXT_SENS_DATA_02 = 0x4B
EXT_SENS_DATA_03 = 0x4C
EXT_SENS_DATA_04 = 0x4D
EXT_SENS_DATA_05 = 0x4E
EXT_SENS_DATA_06 = 0x4F
EXT_SENS_DATA_07 = 0x50
EXT_SENS_DATA_08 = 0x51
EXT_SENS_DATA_09 = 0x52
EXT_SENS_DATA_10 = 0x53
EXT_SENS_DATA_11 = 0x54
EXT_SENS_DATA_12 = 0x55
EXT_SENS_DATA_13 = 0x56
EXT_SENS_DATA_14 = 0x57
EXT_SENS_DATA_15 = 0x58
EXT_SENS_DATA_16 = 0x59
EXT_SENS_DATA_17 = 0x5A
EXT_SENS_DATA_18 = 0x5B
EXT_SENS_DATA_19 = 0x5C
EXT_SENS_DATA_20 = 0x5D
EXT_SENS_DATA_21 = 0x5E
EXT_SENS_DATA_22 = 0x5F
EXT_SENS_DATA_23 = 0x60
# I2C Slave 0/1/2/3 Data Out - Serial IF: R/W, Reset value: 0x00
I2C_SLV0_DO = 0x63
I2C_SLV1_DO = 0x64
I2C_SLV2_DO = 0x65
I2C_SLV3_DO = 0x66
# I2C Master Delay Control - Serial IF: R/W, Reset value: 0x00
I2C_MST_DELAY_CTRL = 0x67
# Signal Path Reset - Serial IF: R/W, Reset value: 0x00
SIGNAL_PATH_RESET = 0x68
# Accelerometer Interrupt Control - Serial IF: R/W, Reset value: 0x00
ACCEL_INTEL_CTRL = 0x69
# User Control - Serial IF: R/W, Reset value: 0x00
USER_CTRL = 0x6A
# Power Management 1 - Serial IF: R/W, Reset value: : (Depends on PU_SLEEP_MODE bit, see below)
# Bit   Name                Function
# [7]   H_RESET             1 - Reset the internal registers and restores the default settings. Write a 1 to
#                           set the reset, the bit will auto clear.
# [6]   SLEEP               When set, the chip is set to sleep mode (After OTP loads, the
#                           PU_SLEEP_MODE bit will be written here)
# [5]   CYCLE               When set, and SLEEP and STANDBY are not set, the chip will cycle
#                           between sleep and taking a single sample at a rate determined by
#                           LP_ACCEL_ODR register
#                           NOTE: When all accelerometer axis are disabled via PWR_MGMT_2
#                           register bits and cycle is enabled, the chip will wake up at the rate
#                           determined by the respective registers above, but will not take any samples.
# [4]   GYRO_STANDBY        When set, the gyro drive and pll circuitry are enabled, but the sense paths
#                           are disabled. This is a low power mode that allows quick enabling of the
#                           gyros.
# [3]   PD_PTAT             Power down internal PTAT voltage generator and PTAT ADC
# [2:0] CLKSEL[2:0]         Code    Clock Source
#                           0       Internal 20MHz oscillator
#                           1       Auto selects the best available clock source - PLL if ready, else
#                                   use the Internal oscillator
#                           2       Auto selects the best available clock source - PLL if ready, else
#                                   use the Internal oscillator
#                           3       Auto selects the best available clock source - PLL if ready, else
#                                   use the Internal oscillator
#                           4       Auto selects the best available clock source - PLL if ready, else
#                                   use the Internal oscillator
#                           5       Auto selects the best available clock source - PLL if ready, else
#                                   use the Internal oscillator
#                           6       Internal 20MHz oscillator
#                           7       Stops the clock and keeps timing generator in reset
#                                   (After OTP loads, the inverse of PU_SLEEP_MODE bit will be written to
#                                   CLKSEL[0])
PWR_MGMT_1 = 0x6B
# Power Management 2 - Serial IF: R/W, Reset value: 0x00
# Bit   Name                Function
# [7:6] -                   Reserved
# [5]   DISABLE_XA          1 - X accelerometer is disabled
#                           0 - X accelerometer is on
# [4]   DISABLE_YA          1 - Y accelerometer is disabled
#                           0 - Y accelerometer is on
# [3]   DISABLE_ZA          1 - Z accelerometer is disabled
#                           0 - Z accelerometer is on
# [2]   DISABLE_XG          1 - X gyro is disabled
#                           0 - X gyro is on
# [1]   DISABLE_YG          1 - Y gyro is disabled
#                           0 - Y gyro is on
# [0]   DISABLE_ZG          1 - Z gyro is disabled
#                           0 - Z gyro is on
PWR_MGMT_2 = 0x6C
# FIFO Count Registers - Serial IF: Read Only, Reset value: 0x00
# Bit     Name                Function
# [15:13] -                   Reserved
# [12:8]  FIFO_CNT_H          High byte
# [7:0]   FIFO_CNT_L          Low byte
FIFO_COUNTH = 0x72
FIFO_COUNTL = 0x73
# FIFO Read Write - Serial IF: R/W, Reset value: 0x00
FIFO_R_W = 0x74
# Who Am I - Serial IF: Read Only, Reset value: 0x68
WHO_AM_I = 0x75
# Accelerometer Offset Registers - Serial IF: R/W, Reset value: 0x00
XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E

## Gyro Full Scale Select 250dps
GFS_250DPS = 0x00
## Gyro Full Scale Select 500dps
GFS_500DPS = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000DPS = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000DPS = 0x03
## Accel Full Scale Select 2G
AFS_2G = 0x00
## Accel Full Scale Select 4G
AFS_4G = 0x01
## Accel Full Scale Select 8G
AFS_8G = 0x02
## Accel Full Scale Select 16G
AFS_16G = 0x03

## smbus
bus = smbus.SMBus(1)

## MPU9250 I2C Control class
class MPU6500:

    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configure()
        self.self_test()
        self.calibrate()
        
    def configure(self):
        """Configure MPU9250 with gfs (Gyro Full Scale Select) and 
        afs (Accel Full Scale Select"""

        # Write a one to bit 7 reset bit; toggle reset device
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Clear sleep mode bit, enable all sensors
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # auto select clock source
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.2)

        self.set_gyro_mode(GFS_1000DPS)
        self.set_accel_mode(AFS_8G)

        # DLPF_CFG (Gryo Bandwidth: 41Hz, Gyro Delay: 5.9ms,FS: 1kHz. Temp Bandwidth: 42Hz. Temp Delay: 4.8ms)
#        bus.write_byte_data(self.address, CONFIG, 0x03)
        # sample rate divider - Sample Rate: Internal Sample Rate / (1 + SMPLRT_DIV)
        bus.write_byte_data(self.address, SMPLRT_DIV, 0x00)
        time.sleep(0.1)

        # Enable bypass mode
        bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.1)

        # Set MST_CLK rate to 400kHz
#        bus.write_byte_data(self.address, I2C_MST_CTRL, 0x09)

    def self_test(self):
        accel_data = bus.read_i2c_block_data(self.address, SELF_TEST_X_ACCEL, 3)
        gyro_data = bus.read_i2c_block_data(self.address, SELF_TEST_X_GYRO, 3)
#        print("Self test:", accel_data, gyro_data)

    def calibrate(self, num_samples=512):
        """Calibrates sensors by writing offsets to specific registers"""

        accel_offs_x = 0.0
        accel_offs_y = 0.0
        accel_offs_z = 0.0
        gyro_offs_x = 0.0
        gyro_offs_y = 0.0
        gyro_offs_z = 0.0

        for i in range(num_samples):
            accel_data = bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 6)
            gyro_data = bus.read_i2c_block_data(self.address, GYRO_XOUT_H, 6)
            
            time.sleep(0.01)
            ax = bytes_to_int16(accel_data[1], accel_data[0])
            ay = bytes_to_int16(accel_data[3], accel_data[2])
            az = bytes_to_int16(accel_data[5], accel_data[4])
            gx = bytes_to_int16(gyro_data[1], gyro_data[0])
            gy = bytes_to_int16(gyro_data[3], gyro_data[2])
            gz = bytes_to_int16(gyro_data[5], gyro_data[4])

            accel_offs_x += ax
            accel_offs_y += ay
            accel_offs_z += az
            gyro_offs_x += gx
            gyro_offs_y += gy
            gyro_offs_z += gz
#            print("Read data (int):", accel, gyro)

        # Normalize sums to get average count biases
        accel_offs_x /= num_samples
        accel_offs_y /= num_samples
        accel_offs_z /= num_samples
        gyro_offs_x /= num_samples
        gyro_offs_y /= num_samples
        gyro_offs_z /= num_samples

 #       print("Calculated biases:", accel_bias, gyro_bias)
        
        # Remove gravity from the z-axis accelerometer bias calculation
        if(accel_offs_z > 0):
            accel_offs_z -= int(1 / self.ares)
        else:
            accel_offs_z += int(1 / self.ares)

        self.accel_offs_x = accel_offs_x
        self.accel_offs_y = accel_offs_y
        self.accel_offs_z = accel_offs_z
        self.gyro_offs_x = gyro_offs_x
        self.gyro_offs_y = gyro_offs_y
        self.gyro_offs_z = gyro_offs_z

#        self.set_accel_bias(accel_bias)
#        self.set_gyro_bias(gyro_bias)

 #       print("Internal biases:", self.get_accel_bias(), self.get_gyro_bias())

    def set_accel_mode(self, mode):
        if mode == AFS_2G:
            self.ares = 2.0 / 32768.0
        elif mode == AFS_4G:
            self.ares = 4.0 / 32768.0
        elif mode == AFS_8G:
            self.ares = 8.0 / 32768.0
        elif mode == AFS_16G:
            self.ares = 16.0 / 32768.0
        else:
            print("GFS Mode not available {}. Using default (AFS_2G).".format(gfs))
            self.ares = 2.0 / 32768.0

        bus.write_byte_data(self.address, ACCEL_CONFIG, mode << 3)
        
    def set_gyro_mode(self, mode):
        if mode == GFS_250DPS:
            self.gres = 250.0 / 32768.0
        elif mode == GFS_500DPS:
            self.gres = 500.0 / 32768.0
        elif mode == GFS_1000DPS:
            self.gres = 1000.0 / 32768.0
        elif mode == GFS_2000DPS:
            self.gres = 2000.0 / 32768.0
        else:
            print("GFS Mode not available {}. Using default (GFS_250).".format(gfs))
            self.gres = 250.0 / 32768.0

        bus.write_byte_data(self.address, GYRO_CONFIG, mode << 3)

    def get_accel_bias(self):
        accel_bias = [0, 0, 0]
        accel_data = bus.read_i2c_block_data(self.address, XA_OFFSET_H, 6)
        for i in range(3):
            accel_bias[i] = bytes_to_int16(accel_data[i*2 + 1], accel_data[i*2])

        return accel_bias
            
    def set_accel_bias(self, accel_bias):
        accel_bias_fact = self.get_accel_bias()
        data = [0, 0, 0, 0, 0, 0]
        for i in range(3):
            accel_reg_bias = accel_bias_fact[i] - (accel_bias[i] & ~1)
            print("Set accel bias:", i, accel_reg_bias)
            data[i*2] = (accel_reg_bias >> 8) & 0xff
            data[i*2 + 1] = (accel_reg_bias & temp_comp_mask[i]) & 0xff
        bus.write_byte_data(self.address, XA_OFFSET_H, data)

    def get_gyro_bias(self):
        gyro_bias = [0, 0, 0]

        gyro_data = bus.read_i2c_block_data(self.address, XG_OFFSET_H, 6)
        for i in range(3):
            gyro_bias[i] = bytes_to_int16(gyro_data[i*2 + 1], gyro_data[i*2])
        return gyro_bias

    def set_gyro_bias(self, gyro_bias):
        data = [0, 0, 0, 0, 0, 0]
        for i in range(3):
            data[i*2] = (-gyro_bias[i] >> 8) & 0xff
            data[i*2 + 1] = -gyro_bias[i] & 0xff
        bus.write_i2c_block_data(self.address, XG_OFFSET_H, data)

    def search_device(self):
        """Check if WHO_AM_I register read equals DEVICE_ID"""
        who_am_i = bus.read_byte_data(self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID):
            return True
        else:
            return False

    def check_data_ready(self):
        """Checks if data is ready to be read from sensor"""
        drdy = bus.read_byte_data(self.address, INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    def read_accel(self):
        """Read accelerometer data from ACCEL_OUT register"""
        data = bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 6)
        x = (bytes_to_int16(data[1], data[0]) - self.accel_offs_x) * self.ares
        y = (bytes_to_int16(data[3], data[2]) - self.accel_offs_y) * self.ares
        z = (bytes_to_int16(data[5], data[4]) - self.accel_offs_z) * self.ares

        return (x, y, z)

    def read_gyro(self):
        """Read gyroscope data from GYRO_OUT register"""
        data = bus.read_i2c_block_data(self.address, GYRO_XOUT_H, 6)
        x = (bytes_to_int16(data[1], data[0]) - self.gyro_offs_x) * self.gres
        y = (bytes_to_int16(data[3], data[2]) - self.gyro_offs_y) * self.gres
        z = (bytes_to_int16(data[5], data[4]) - self.gyro_offs_z) * self.gres

        return (x, y, z)

    def read_temperature(self):
        """Read temperature in degrees C"""
        data = bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
        temp = bytes_to_int16(data[1], data[0])

        temp = temp / 333.87 + 21.0
        return temp
