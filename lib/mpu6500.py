"""
Implementation of the MPU6500 6-DOF sensor chip
(Accelerometer, Gyroscope)
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
XG_OFFS_USR_H = 0x13
XG_OFFS_USR_L = 0x14
YG_OFFS_USR_H = 0x15
YG_OFFS_USR_L = 0x16
ZG_OFFS_USR_H = 0x17
ZG_OFFS_USR_L = 0x18
# Sample Rate Divider - Serial IF: R/W, Reset value: 0x00
SMPLRT_DIV = 0x19
# Configuration
CONFIG = 0x1A
# Gyroscope Configuration - Serial IF: R/W, Reset value: 0x00
GYRO_CONFIG = 0x1B
# Accelerometer Configuration - Serial IF: R/W, Reset value: 0x00
ACCEL_CONFIG = 0x1C
# Accelerometer Configuration 2 - Serial IF: R/W, Reset value: 0x00
ACCEL_CONFIG_2 = 0x1D
# Low Power Accelerometer ODR Control - Serial IF: R/W, Reset value: 0x00
LP_ACCEL_ODR = 0x1E
# Wake on Motion Threshold - Serial IF: R/W, Reset value: 0x00
WOM_THR = 0x1F
# FIFO Enable - Serial IF: R/W, Reset value: 0x00
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
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
# Gyroscope Measurements - Serial IF: SyncR, Reset value: 0x00 (if sensor disabled)
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
I2C_SLV0_DO = 0x63
I2C_SLV1_DO = 0x64
I2C_SLV2_DO = 0x65
I2C_SLV3_DO = 0x66
I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET = 0x68
ACCEL_INTEL_CTRL = 0x69
USER_CTRL = 0x6A
# Power Management 1 + 2
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
# FIFO Count registers, used to determine current FIFO buffer size
FIFO_COUNTH = 0x72
FIFO_COUNTL = 0x73
FIFO_R_W = 0x74
WHO_AM_I = 0x75
# Accelerometer offset registers - Serial IF: R/W, Reset value: 0x00
XA_OFFS_H = 0x77
XA_OFFS_L = 0x78
YA_OFFS_H = 0x7A
YA_OFFS_L = 0x7B
ZA_OFFS_H = 0x7D
ZA_OFFS_L = 0x7E


# Sample Rate Modes
SAMPLE_RATE_1KHZ = 0x00
SAMPLE_RATE_500HZ = 0x01
SAMPLE_RATE_333HZ = 0x02
SAMPLE_RATE_250HZ = 0x03
SAMPLE_RATE_200HZ = 0x04
SAMPLE_RATE_166HZ = 0x05
SAMPLE_RATE_143HZ = 0x06
SAMPLE_RATE_125HZ = 0x07
SAMPLE_RATE_111HZ = 0x08
SAMPLE_RATE_100HZ = 0x09
SAMPLE_RATE_50HZ = 0x13

# Clock Source Modes
CLK_20MHZ_OSC = 0x00
CLK_AUTO = 0x01

# DLPF Modes
DLPF_GT_250HZ_4000HZ = 0x00
DLPF_GT_184HZ_188HZ = 0x01
DLPF_GT_92HZ_98HZ = 0x02
DLPF_GT_41HZ_42HZ = 0x03
DLPF_GT_20HZ_20HZ = 0x04
DLPF_GT_10HZ_10HZ = 0x05
DLPF_GT_5HZ_5HZ = 0x06
DLPF_GT_3600HZ_4000HZ = 0x07

# FIFO Write Modes
FIFO_MODE_NO_OVERWRITE = 0x40

# Gyro Full Scale Select Modes (degrees/sec)
GFS_250DPS = 0x00
GFS_500DPS = 0x01
GFS_1000DPS = 0x02
GFS_2000DPS = 0x03

# Accel Full Scale Select Modes (g)
AFS_2G = 0x00
AFS_4G = 0x01
AFS_8G = 0x02
AFS_16G = 0x03

# Load I2C interface
bus = smbus.SMBus(1)

def mean(nums_list):
    return float(sum(nums_list)) / len(nums_list)

def to_bin_str(value):
    return format(value, "08b")

class MPU6500:
    """Holds all the sensor functionality."""
    
    def __init__(self, address=SLAVE_ADDRESS, fifo_mode=False):
        """Initialize sensor and calibrate."""
        self.address = address
        self.accel_offs = [0, 0, 0]
        self.gyro_offs = [0, 0, 0]

        self.configure(fifo_mode)

        self.print_cfg()

        self.calibrate(fifo_mode)

    def read_register_burst(self, register, n):
        """Reads n bytes from register with auto-increment."""
        return bus.read_i2c_block_data(self.address, register, n)

    def write_register_burst(self, register, data):
        """Writes n bytes to register with auto-increment."""
        bus.write_i2c_block_data(self.address, register, data)

    def read_register(self, register):
        """Reads value from register."""
        return bus.read_byte_data(self.address, register)

    def write_register(self, register, value):
        """Writes value to register."""
        bus.write_byte_data(self.address, register, value)

    def read_register_bit(self, register, index):
        """Reads specific bit at 'index' from register."""
        data = self.read_register(register)
        return check_bit(data, index)

    def write_register_bit(self, register, index, value):
        """Write 'value' into specific bit at 'index' to register."""
        data = self.read_register(register)
        val = set_bit_value(data, index, value)
        self.write_register(register, val)

    def get_self_test_x_gyro(self):
        """Get gyroscope x-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_X_GYRO)

    def set_self_test_x_gyro(self, value):
        """Set gyroscope x-axis self test value."""
        self.write_register(SELF_TEST_X_GYRO, value)

    def get_self_test_y_gyro(self):
        """Get gyroscope y-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Y_GYRO)

    def set_self_test_y_gyro(self, value):
        """Set gyroscope y-axis self test value."""
        self.write_register(SELF_TEST_Y_GYRO, value)

    def get_self_test_z_gyro(self):
        """Get gyroscope z-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Z_GYRO)

    def set_self_test_z_gyro(self, value):
        """Set gyroscope z-axis self test value."""
        self.write_register(SELF_TEST_Z_GYRO, value)

    def get_self_test_gyro(self):
        """Get gyroscope self test output from all axes."""
        return self.read_register_burst(SELF_TEST_X_GYRO, 3)

    def set_self_test_gyro(self, values):
        """Set gyroscope self test values of all axes."""
        self.write_register_burst(SELF_TEST_X_GYRO, values)

    def get_self_test_x_accel(self):
        """Get accelerometer x-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_X_ACCEL)

    def set_self_test_x_accel(self, value):
        """Set accelerometer x-axis self test value."""
        self.write_register(SELF_TEST_X_ACCEL, value)

    def get_self_test_y_accel(self):
        """Get accelerometer y-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Y_ACCEL)

    def set_self_test_y_accel(self, value):
        """Set accelerometer y-axis self test value."""
        self.write_register(SELF_TEST_Y_ACCEL, value)

    def get_self_test_z_accel(self):
        """Get accelerometer z-axis self test output.

        This value is generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Z_ACCEL)

    def set_self_test_z_accel(self, value):
        """Set accelerometer z-axis self test value."""
        self.write_register(SELF_TEST_Z_ACCEL, value)

    def get_self_test_accel(self):
        """Get accelerometer self test output from all axes."""
        return self.read_register_burst(SELF_TEST_X_ACCEL, 3)

    def set_self_test_accel(self, values):
        """Set accelerometer self test values of all axes."""
        self.write_register_burst(SELF_TEST_X_ACCEL, values)

    def get_gx_offs_usr(self):
        """Get gyroscope x-axis Offset."""
        data = self.read_register_burst(XG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_gx_offs_usr(self, value):
        """Set gyroscope x-axis offset."""
        (b0, b1) = int16_to_bytes(value)
        self.write_register_burst(XG_OFFS_USR_H, [b1, b0])

    def get_gy_offs_usr(self):
        """Get gyroscope y-axis offset."""
        data = self.read_register_burst(YG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_gy_offs_usr(self, value):
        """Set gyroscope y-axis offset."""
        (b0, b1) = int16_to_bytes(value)
        self.write_register_burst(YG_OFFS_USR_H, [b1, b0])

    def get_gz_offs_usr(self):
        """Get gyroscope z-axis offset."""
        data = self.read_register_burst(ZG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_gz_offs_usr(self, value):
        """Set gyroscope z-axis offset."""
        (b0, b1) = int16_to_bytes(value)
        self.write_register_burst(ZG_OFFS_USR_H, [b1, b0])

    def get_gyro_offs_usr(self):
        """Get gyroscope offset for all axes."""
        data = self.read_register_burst(XG_OFFS_USR_H, 6)
        x = bytes_to_int16(data[1], data[0])
        y = bytes_to_int16(data[3], data[2])
        z = bytes_to_int16(data[5], data[4])
        return (x, y, z)

    def set_gyro_offs_usr(self, values):
        """Set gyroscope offset for all axes."""
        x_l, x_h = int16_to_bytes(values[0])
        y_l, y_h = int16_to_bytes(values[1])
        z_l, z_h = int16_to_bytes(values[2])
        data = [x_h, x_l, y_h, y_l, z_h, z_l]
        self.write_register_burst(XG_OFFS_USR_H, data)

    def get_smplrt_div(self):
        """Get sample rate divider value."""
        return self.read_register(SMPLRT_DIV)

    def set_smplrt_div(self, value):
        """Set sample rate divider value."""
        self.write_register(SMPLRT_DIV, value)

    def get_fifo_mode(self):
        """Get FIFO write mode."""
        return self.read_register_bit(CONFIG, 6)
        
    def set_fifo_mode(self, value):
        """Set FIFO write mode."""
        self.write_register_bit(CONFIG, 6, value)

    def get_ext_fsync_set(self):
        """Get external FSYNC setup."""
        data = self.read_register(CONFIG)
        return data & 0x38

    def set_ext_fsync_set(self, value):
        """Set external FSYNC setup."""
        data = self.read_register(CONFIG)
        val = set_bits(data, 3, 3, value)
        self.write_register(CONFIG, val)

    def get_gyro_temp_dlpf_cfg(self):
        """Get gyrscope and temperature sensor DLPF config."""
        data = self.read_register(CONFIG)
        return data & 0x07

    def set_gyro_temp_dlpf_cfg(self, value):
        """Set gyroscope and temperature sensor DLPF config."""
        data = self.read_register(CONFIG)
        val = set_bits(data, 0, 3, value)
        self.write_register(CONFIG, val)

    # Gyroscope Configuration Methods
        
    def get_gx_st_en(self):
        """Get gyroscope self test enabled value for x-axis."""
        return self.read_register_bit(GYRO_CONFIG, 7)

    def set_gx_st_en(self, value):
        """Set gyroscope self test enabled value for x-axis."""
        self.write_register_bit(GYRO_CONFIG, 7, value)

    def get_gy_st_en(self):
        """Get gyroscope self test enabled value for y-axis."""
        return self.read_register_bit(GYRO_CONFIG, 6)

    def set_gy_st_en(self, value):
        """Set gyroscope self test enabled value for y-axis."""
        self.write_register_bit(GYRO_CONFIG, 6, value)

    def get_gz_st_en(self):
        """Get gyroscope self test enabled value for z-axis."""
        return self.read_register_bit(GYRO_CONFIG, 5)

    def set_gz_st_en(self, value):
        """Set gyroscope self test enabled value for z-axis."""
        self.write_register_bit(GYRO_CONFIG, 5, value)

    def get_gyro_fs_sel(self):
        """Get gyroscope full scale select mode."""
        data = self.read_register(GYRO_CONFIG)
        return (data & 0x18) >> 3

    def set_gyro_fs_sel(self, value):
        """Set gyroscope full scale select mode."""
        data = self.read_register(GYRO_CONFIG)
        val = set_bits(data, 3, 2, value)
        self.write_register(GYRO_CONFIG, val)

    def get_gyro_fchoice_b(self):
        """Get gyroscope FCHOICE_B value (inverted to FCHOICE)."""
        data = self.read_register(GYRO_CONFIG)
        return data & 0x03

    def set_gyro_fchoice_b(self, value):
        """Set gyroscope FCHOICE_B value (inverted FCHOICE)."""
        data = self.read_register(GYRO_CONFIG)
        val = set_bits(data, 0, 2, value)
        self.write_register(GYRO_CONFIG, val)

    # Accelerometer Configuration Methods

    def get_ax_st_en(self):
        """Get accelerometer self test enabled value for x-axis."""
        return self.read_register_bit(ACCEL_CONFIG, 7)

    def set_ax_st_en(self, value):
        """Set accelerometer self test enabled value for x-axis."""
        self.write_register_bit(ACCEL_CONFIG, 7, value)

    def get_ay_st_en(self):
        """Get accelerometer self test enabled value for y-axis."""
        return self.read_register_bit(ACCEL_CONFIG, 6)

    def set_ay_st_en(self, value):
        """Set accelerometer self test enabled value for y-axis."""
        self.write_register_bit(ACCEL_CONFIG, 6, value)

    def get_az_st_en(self):
        """Get accelerometer self test enabled value for z-axis."""
        return self.read_register_bit(ACCEL_CONFIG, 5)

    def set_az_st_en(self, value):
        """Set accelerometer self test enabled value for z-axis."""
        self.write_register_bit(ACCEL_CONFIG, 5, value)

    def get_accel_fs_sel(self):
        """Get accelerometer full scale select mode."""
        data = self.read_register(ACCEL_CONFIG)
        return (data & 0x18) >> 3

    def set_accel_fs_sel(self, value):
        """Set accelerometer full scale select mode."""
        data = self.read_register(ACCEL_CONFIG)
        val = set_bits(data, 3, 2, value)
        self.write_register(ACCEL_CONFIG, val)

    # Accelerometer Configuration 2 Methods
        
    def get_accel_fchoice_b(self):
        """Get accelerometer FCHOICE_B value (inverted to FCHOICE)."""
        return self.read_register_bit(ACCEL_CONFIG_2, 3)

    def set_accel_fchoice_b(self, value):
        """Get accelerometer FCHOICE_B value (inverted to FCHOICE)."""
        self.write_register_bit(ACCEL_CONFIG_2, 3, value)

    def get_accel_dlpf_cfg(self):
        """Get accelerometer DLPF config."""
        data = self.read_register(ACCEL_CONFIG_2)
        return data & 0x07
        
    def set_accel_dlpf_cfg(self, value):
        """Set accelerometer DLPF config."""
        data = self.read_register(ACCEL_CONFIG_2)
        val = set_bits(data, 0, 3, value)
        self.write_register(ACCEL_CONFIG_2, val)

    def get_accel_lp_odr(self):
        """Get accelerometer low power ODR (Output Data Rate).

        This is the frequency of waking up the chip to take
        a sample of accel data.
        """
        data = self.read_register(LP_ACCEL_ODR)
        return data & 0x0f

    def set_accel_lp_odr(self, value):
        """Set accelerometer low power ODR (Output Data Rate).

        This is the frequency of waking up the chip to take
        a sample of accel data.
        """
        data = self.read_register(LP_ACCEL_ODR)
        val = set_bits(data, 0, 4, value)
        self.write_register(LP_ACCEL_ODR, val)

    def get_wom_threshold(self):
        """Get Wake-on Motion Threshold."""
        return self.read_register(WOM_THR)
        
    def set_wom_threshold(self, value):
        """Set Wake-on Motion Threshold."""
        self.write_register(WOM_THR, value)

    def get_temp_fifo_en(self):
        """Return temperature sensor FIFO enabled status.

        1 - Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        return self.read_register_bit(FIFO_EN, 7)

    def set_temp_fifo_en(self, value):
        """Set temperature sensor FIFO enabled status.

        1 - Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        self.write_register_bit(FIFO_EN, 7, value)

    def get_gx_fifo_en(self):
        """Return gyroscope x-axis FIFO enabled status.

        1 - Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        return self.read_register_bit(FIFO_EN, 6)

    def set_gx_fifo_en(self, value):
        """Set gyroscope x-axis FIFO enabled status.

        1 - Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        self.write_register_bit(FIFO_EN, 6, value)

    def get_gy_fifo_en(self):
        """Return gyroscope y-axis FIFO enabled status.

        1 - Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        return self.read_register_bit(FIFO_EN, 5)

    def set_gy_fifo_en(self, value):
        """Set gyroscope y-axis FIFO enabled status.

        1 - Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        self.write_register_bit(FIFO_EN, 5, value)

    def get_gz_fifo_en(self):
        """Return gyroscope z-axis FIFO enabled status.

        1 - Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        return self.read_register_bit(FIFO_EN, 4)

    def set_gz_fifo_en(self, value):
        """Set gyroscope z-axis FIFO enabled status.

        1 - Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        NOTE: If enabled, buffering of data occurs even if data path is in standby.
        """
        self.write_register_bit(FIFO_EN, 4, value)

    def get_accel_fifo_en(self):
        """Get accelerometer FIFO enabled status.

        1 - Write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
        ACCEL_ZOUT_H and ACCEL_ZOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        """
        return self.read_register_bit(FIFO_EN, 3)

    def set_accel_fifo_en(self, value):
        """Set accelerometer FIFO enabled status.

        1 - Write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
        ACCEL_ZOUT_H and ACCEL_ZOUT_L to the FIFO at the sample rate;
        0 - function is disabled
        """
        self.write_register_bit(FIFO_EN, 3, value)

    def get_slv2_fifo_en(self):
        """Get SLV_2 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_2 to the FIFO at the sample rate;
        0 - function is disabled
        """
        return self.read_register_bit(FIFO_EN, 2)

    def set_slv2_fifo_en(self, value):
        """Set SLV_2 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_2 to the FIFO at the sample rate;
        0 - function is disabled
        """
        self.write_register_bit(FIFO_EN, 2, value)

    def get_slv1_fifo_en(self):
        """Get SLV_1 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_1 to the FIFO at the sample rate;
        0 - function is disabled
        """
        return self.read_register_bit(FIFO_EN, 1)

    def set_slv1_fifo_en(self, value):
        """Set SLV_1 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_1 to the FIFO at the sample rate;
        0 - function is disabled
        """
        self.write_register_bit(FIFO_EN, 1, value)

    def get_slv0_fifo_en(self):
        """Get SLV_0 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_0 to the FIFO at the sample rate;
        0 - function is disabled
        """
        return self.read_register_bit(FIFO_EN, 0)

    def set_slv0_fifo_en(self, value):
        """Set SLV_0 FIFO enabled status.

        1 - Write EXT_SENS_DATA registers associated to SLV_0 to the FIFO at the sample rate;
        0 - function is disabled
        """
        self.write_register_bit(FIFO_EN, 0, value)

    def set_accel_gyro_fifo_en(self):
        """Set accelerometer and gyroscope FIFO enabled status."""
        data = self.read_register(FIFO_EN)
        val = set_bits(data, 3, 4, 0x0f)
        self.write_register(FIFO_EN, val)

    def get_mult_mst_en(self):
        return self.read_register_bit(I2C_MST_CTRL, 7)

    def set_mult_mst_en(self, value):
        self.write_register_bit(I2C_MST_CTRL, 7, value)

    def get_wait_for_es(self):
        return self.read_register_bit(I2C_MST_CTRL, 6)

    def set_wait_for_es(self, value):
        self.write_register_bit(I2C_MST_CTRL, 6, value)

    def get_slv3_fifo_en(self):
        return self.read_register_bit(I2C_MST_CTRL, 5)

    def set_slv3_fifo_en(self, value):
        self.write_register_bit(I2C_MST_CTRL, 5, value)

    def get_i2c_mst_p_nsr(self):
        return self.read_register_bit(I2C_MST_CTRL, 4)

    def set_i2c_mst_p_nsr(self, value):
        self.write_register_bit(I2C_MST_CTRL, 4, value)

    def get_i2c_mst_clk(self):
        data = self.read_register(I2C_MST_CTRL)
        return data & 0x0f

    def set_i2c_mst_clk(self, value):
        data = self.read_register(I2C_MST_CTRL)
        val = set_bits(data, 0, 4, value)
        self.write_register(I2C_MST_CTRL, val)
        
    ## Getters and setters for I2C Slave devices and external sensor data ####
    # TODO: create
    ##

    def get_int_pin_active_low(self):
        return self.read_register_bit(INT_PIN_CFG, 7)

    def set_int_pin_active_low(self, value):
        self.write_register_bit(INT_PIN_CFG, 7, value)

    def get_int_pin_open_drain(self):
        return self.read_register_bit(INT_PIN_CFG, 6)

    def set_int_pin_open_drain(self, value):
        self.write_register_bit(INT_PIN_CFG, 6, value)

    def get_int_pin_latch_int_en(self):
        return self.read_register_bit(INT_PIN_CFG, 5)

    def set_int_pin_latch_int_en(self, value):
        self.write_register_bit(INT_PIN_CFG, 5, value)

    def get_int_pin_any_read_clear(self):
        return self.read_register_bit(INT_PIN_CFG, 4)

    def set_int_pin_any_read_clear(self, value):
        self.write_register_bit(INT_PIN_CFG, 4, value)

    def get_int_pin_active_low_fsync(self):
        return self.read_register_bit(INT_PIN_CFG, 3)

    def set_int_pin_active_low_fsync(self, value):
        self.write_register_bit(INT_PIN_CFG, 3, value)

    def get_int_pin_fsync_int_mode_en(self):
        return self.read_register_bit(INT_PIN_CFG, 2)

    def set_int_pin_fsync_int_mode_en(self, value):
        self.write_register_bit(INT_PIN_CFG, 2, value)

    def get_int_pin_bypass_en(self):
        return self.read_register_bit(INT_PIN_CFG, 1)

    def set_int_pin_bypass_en(self, value):
        """
        Enable bypass mode of sensor 
        to allow third party sensor at I2C_AUX
        """
        self.write_register_bit(INT_PIN_CFG, 1, value)


    def get_wom_en(self):
        return self.read_register_bit(INT_ENABLE, 6)

    def set_wom_en(self, value):
        self.write_register_bit(INT_ENABLE, 6, value)

    def get_fifo_overflow_en(self):
        return self.read_register_bit(INT_ENABLE, 4)

    def set_fifo_overflow_en(self, value):
        self.write_register_bit(INT_ENABLE, 4, value)

    def get_fsync_int_en(self):
        return self.read_register_bit(INT_ENABLE, 3)

    def set_fsync_int_en(self, value):
        self.write_register_bit(INT_ENABLE, 3, value)

    def get_raw_ready_en(self):
        return self.read_register_bit(INT_ENABLE, 0)

    def set_raw_ready_en(self, value):
        self.write_register_bit(INT_ENABLE, 0, value)

    def disable_interrupts(self):
        """Disable Interrupts"""
        self.write_register(INT_ENABLE, 0x00)

    def get_wom_int(self):
        return self.read_register_bit(INT_STATUS, 6)

    def get_fifo_overflow_int(self):
        return self.read_register_bit(INT_STATUS, 4)

    def get_fsync_int(self):
        return self.read_register_bit(INT_STATUS, 3)

    def get_raw_data_ready_int(self):
        return self.read_register_bit(INT_STATUS, 0)

    def read_accel_x_raw(self):
        data = self.read_register_burst(ACCEL_XOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_accel_y_raw(self):
        data = self.read_register_burst(ACCEL_YOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_accel_z_raw(self):
        data = self.read_register_burst(ACCEL_ZOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_accel_raw(self):
        data = self.read_register_burst(ACCEL_XOUT_H, 6)
        x = bytes_to_int16(data[1], data[0])
        y = bytes_to_int16(data[3], data[2])
        z = bytes_to_int16(data[5], data[4])
        return (x, y, z)

    def read_temp_raw(self):
        data = self.read_register_burst(TEMP_OUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_gyro_x_raw(self):
        data = self.read_register_burst(GYRO_XOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_gyro_y_raw(self):
        data = self.read_register_burst(GYRO_YOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_gyro_z_raw(self):
        data = self.read_register_burst(GYRO_ZOUT_H, 2)
        return bytes_to_int16(data[1], data[0])

    def read_gyro_raw(self):
        data = self.read_register_burst(GYRO_XOUT_H, 6)
        x = bytes_to_int16(data[1], data[0])
        y = bytes_to_int16(data[3], data[2])
        z = bytes_to_int16(data[5], data[4])
        return (x, y, z)

    def get_gyro_reset(self):
        return self.read_register_bit(SIGNAL_PATH_RESET, 2)

    def set_gyro_reset(self, value):
        self.write_register_bit(SIGNAL_PATH_RESET, 2, value)

    def get_accel_reset(self):
        return self.read_register_bit(SIGNAL_PATH_RESET, 1)

    def set_accel_reset(self, value):
        self.write_register_bit(SIGNAL_PATH_RESET, 1, value)

    def get_temp_reset(self):
        return self.read_register_bit(SIGNAL_PATH_RESET, 0)

    def set_temp_reset(self, value):
        self.write_register_bit(SIGNAL_PATH_RESET, 0, value)

    def get_accel_intel_en(self):
        return self.read_register_bit(ACCEL_INTEL_CTRL, 7)

    def set_accel_intel_en(self, value):
        self.write_register_bit(ACCEL_INTEL_CTRL, 7, value)

    def get_accel_intel_mode(self):
        return self.read_register_bit(ACCEL_INTEL_CTRL, 6)

    def set_accel_intel_mode(self, value):
        self.write_register_bit(ACCEL_INTEL_CTRL, 6, value)
        
    def get_fifo_en(self):
        return self.read_register_bit(USER_CTRL, 6)

    def set_fifo_en(self, value):
        self.write_register_bit(USER_CTRL, 6, value)

    def get_i2c_mst_en(self):
        return self.read_register_bit(USER_CTRL, 5)

    def set_i2c_mst_en(self, value):
        self.write_register_bit(USER_CTRL, 5, value)

    def get_i2c_interface_disable(self):
        return self.read_register_bit(USER_CTRL, 4)

    def set_i2c_interface_disable(self, value):
        self.write_register_bit(USER_CTRL, 4, value)

    def set_fifo_reset(self):
        self.write_register_bit(USER_CTRL, 2, True)

    def set_i2c_mst_reset(self):
        self.write_register_bit(USER_CTRL, 1, True)

    def set_signal_condition_reset(self):
        self.write_register_bit(USER_CTRL, 0, True)

    def set_h_reset(self):
        self.write_register_bit(PWR_MGMT_1, 7, True)

    def set_sleep(self):
        self.write_register_bit(PWR_MGMT_1, 6, True)

    def set_cycle(self):
        self.write_register_bit(PWR_MGMT_1, 5, True)

    def set_gyro_standby(self):
        self.write_register_bit(PWR_MGMT_1, 4, True)

    def set_power_down_ptat(self):
        self.write_register_bit(PWR_MGMT_1, 3, True)

    def get_clock_source(self):
        data = self.read_register(PWR_MGMT_1)
        return data & 0x07

    def set_clock_source(self, value):
        data = self.read_register(PWR_MGMT_1)
        val = set_bits(data, 0, 3, value)
        self.write_register(PWR_MGMT_1, val)

    def reset_pwr_mgmt_1(self):
        self.write_register(PWR_MGMT_1, 0x00)

    def get_disable_ax(self):
        return self.read_register_bit(PWR_MGMT_2, 5)

    def set_disable_ax(self, value):
        self.write_register_bit(PWR_MGMT_2, 5)

    def get_disable_ay(self):
        return self.read_register_bit(PWR_MGMT_2, 4)

    def set_disable_ay(self, value):
        self.write_register_bit(PWR_MGMT_2, 4)

    def get_disable_az(self):
        return self.read_register_bit(PWR_MGMT_2, 3)

    def set_disable_az(self, value):
        self.write_register_bit(PWR_MGMT_2, 3)

    def get_disable_gx(self):
        return self.read_register_bit(PWR_MGMT_2, 2)

    def set_disable_gx(self, value):
        self.write_register_bit(PWR_MGMT_2, 2)

    def get_disable_gy(self):
        return self.read_register_bit(PWR_MGMT_2, 1)

    def set_disable_gy(self, value):
        self.write_register_bit(PWR_MGMT_2, 1)

    def get_disable_gz(self):
        return self.read_register_bit(PWR_MGMT_2, 0)

    def set_disable_gz(self, value):
        self.write_register_bit(PWR_MGMT_2, 0)

    def disable_accel(self):
        data = self.read_register(PWR_MGMT_2)
        val = set_bits(data, 3, 3, 0x07)
        self.write_register(PWR_MGMT_2, val)
        
    def disable_gyro(self):
        data = self.read_register(PWR_MGMT_2)
        val = set_bits(data, 0, 3, 0x07)
        self.write_register(PWR_MGMT_2, val)

    def enable_sensors(self):
        self.write_register(PWR_MGMT_2, 0x00)

    def get_fifo_count(self):
        data = self.read_register_burst(FIFO_COUNTH, 2)
        return bytes_to_int16(data[1], data[0])

    def get_fifo_rw(self):
        return self.read_register(FIFO_R_W)

    def set_fifo_rw(self, value):
        self.write_register(FIFO_RW, value)

    def read_fifo_raw(self):
        data = self.read_register_burst(FIFO_R_W, 12)
        ax = bytes_to_int16(data[1], data[0])
        ay = bytes_to_int16(data[3], data[2])
        az = bytes_to_int16(data[5], data[4])
        gx = bytes_to_int16(data[7], data[6])
        gy = bytes_to_int16(data[9], data[8])
        gz = bytes_to_int16(data[11], data[10])

        return ((ax, ay, az), (gx, gy, gz))

    def get_whoami(self):
        return self.read_register(WHOAMI)

    def get_ax_offs(self):
        data = self.read_register_burst(XA_OFFS_H, 2)
        return bytes_to_int15(data[1], data[0])

    def set_ax_offs(self, value):
        (b0, b1) = int15_to_bytes(value)
        self.write_register_burst(XA_OFFS_H, [b1, b0])

    def get_ay_offs(self):
        data = self.read_register_burst(YA_OFFS_H, 2)
        return bytes_to_int15(data[1], data[0])

    def set_ay_offs(self, value):
        (b0, b1) = int15_to_bytes(value)
        self.write_register_burst(YA_OFFS_H, [b1, b0])

    def get_az_offs(self):
        data = self.read_register_burst(ZA_OFFS_H, 2)
        return bytes_to_int15(data[1], data[0])

    def set_az_offs(self, value):
        (b0, b1) = int15_to_bytes(value)
        self.write_register_burst(ZA_OFFS_H, [b1, b0])

    def get_accel_offs(self):
        x = self.get_ax_offs()
        y = self.get_ay_offs()
        z = self.get_az_offs()
        return (x, y, z)

    def set_accel_offs(self, values):
        self.set_ax_offs(values[0])
        self.set_ay_offs(values[1])
        self.set_az_offs(values[2])
        
    def enable_fifo(self):
        """Enable FIFO mode"""
        self.set_fifo_en(True)
        
        # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes)
        self.set_accel_gyro_fifo_en()

    def disable_i2c_master(self):
        """Disable I2C Master"""
        self.write_register(I2C_MST_CTRL, 0x00)
        self.set_i2c_mst_en(False)

    def print_cfg(self):
        print("Gyro Self Test", self.get_self_test_gyro())
        print("Accel Self Test", self.get_self_test_accel())
        print("Gyro Offsets", self.get_gyro_offs_usr())
        print("Accel Offsets", self.get_accel_offs())
        print("SMPLRT_DIV", self.get_smplrt_div())
        # Main Configuration
        print("Configuration")
        print(to_bin_str(self.read_register(CONFIG)))
        print("FIFO_MODE", self.get_fifo_mode())
        print("EXT_FSYNC_SET", self.get_ext_fsync_set())
        print("DLPF_CFG Gyro/Temp", self.get_gyro_temp_dlpf_cfg())
        # Gyroscope Configuration
        print("Gyroscope Configuration")
        print(to_bin_str(self.read_register(GYRO_CONFIG)))
        print("Gyro X Self Test Enabled", self.get_gx_st_en())
        print("Gyro Y Self Test Enabled", self.get_gy_st_en())
        print("Gyro Z Self Test Enabled", self.get_gz_st_en())
        print("GYRO_FS_SEL", self.get_gyro_fs_sel())
        print("FCHOICE_B", self.get_gyro_fchoice_b())
        # Accelerometer Configuration
        print("Accelerometer Configuration")
        print(to_bin_str(self.read_register(ACCEL_CONFIG)))
        print("Accel X Self Test Enabled", self.get_ax_st_en())
        print("Accel Y Self Test Enabled", self.get_ay_st_en())
        print("Accel Z Self Test Enabled", self.get_az_st_en())
        print("ACCEL_FS_SEL", self.get_accel_fs_sel())
        print("Accelerometer Configuration 2")
        print(to_bin_str(self.read_register(ACCEL_CONFIG_2)))
        print("FCHOICE_B", self.get_accel_fchoice_b())
        print("DLPF_CFG Accel", self.get_accel_dlpf_cfg())
        # WOM Threshold
        print("WOM_THR", self.get_wom_threshold())
        # FIFO_EN Configuration
        print("FIFO Enable")
        print(to_bin_str(self.read_register(FIFO_EN)))
        print("FIFO TEMP_OUT", self.get_temp_fifo_en())
        print("FIFO GYRO_XOUT", self.get_gx_fifo_en())
        print("FIFO GYRO_YOUT", self.get_gy_fifo_en())
        print("FIFO GYRO_ZOUT", self.get_gz_fifo_en())
        print("FIFO ACCEL_OUT", self.get_accel_fifo_en())
        print("FIFO SLV_2", self.get_slv2_fifo_en())
        print("FIFO SLV_1", self.get_slv1_fifo_en())
        print("FIFO SLV_0", self.get_slv0_fifo_en())
        # I2C Master Control
        print("I2C Master Control")
        print(to_bin_str(self.read_register(I2C_MST_CTRL)))
        print("MULT_MST_EN", self.get_mult_mst_en())
        print("WAIT_FOR_ES", self.get_wait_for_es())
        print("FIFO SLV_3", self.get_slv3_fifo_en())
        print("I2C_MST_P_NSR", self.get_i2c_mst_p_nsr())
        print("I2C_MST_CLK", self.get_i2c_mst_clk())
        # User Control
        print("User Control")
        print(to_bin_str(self.read_register(USER_CTRL)))
        print("FIFO_EN", self.get_fifo_en())
        print("I2C_MST_EN", self.get_i2c_mst_en())
        print("I2C_IF_DIS", self.get_i2c_interface_disable())
        print("Power Management 1")
        print(to_bin_str(self.read_register(PWR_MGMT_1)))
        print("Clock Source", self.get_clock_source())
        print("Power Management 2")
        print(to_bin_str(self.read_register(PWR_MGMT_2)))

    def configure(self, fifo_mode=False):
        if fifo_mode:
            self._configure_fifo()
        else:
            self._configure()
        
    def _configure_fifo(self):
        """Configure MPU9250 in FIFO mode."""

        self.set_h_reset()
        time.sleep(0.1)
        
        # Reset PWR_MGMT_1 register to clear sleep mode bit and enable device
        self.reset_pwr_mgmt_1()
        time.sleep(0.1)
        
        # auto select clock source
        self.set_clock_source(CLK_AUTO)

        # Enable all sensors
        self.enable_sensors()
        time.sleep(0.2)

        # Configure device for bias calculation
        self.disable_interrupts()
        self.set_fifo_en(False)
        self.set_clock_source(CLK_20MHZ_OSC)
        self.disable_i2c_master()
        self.set_fifo_reset()
        time.sleep(0.015)
        
        # Configure MPU6050 gyro and accelerometer for bias calculation
        self.set_fifo_mode(FIFO_MODE_NO_OVERWRITE)
        self.set_gyro_temp_dlpf_cfg(DLPF_GT_184HZ_188HZ)
        self.set_smplrt_div(SAMPLE_RATE_100HZ)
        self.set_gyro_mode(GFS_1000DPS)
        self.set_accel_mode(AFS_8G)
        self.set_int_pin_bypass_en(True)
        time.sleep(0.1)
        self.enable_fifo()
        
    def _configure(self):
        """Configure MPU9250"""

        # Write a one to bit 7 reset bit; toggle reset device
        self.set_h_reset()
        time.sleep(0.1)

        # Clear sleep mode bit, enable all sensors
        self.reset_pwr_mgmt_1()
        time.sleep(0.1)
        
        # auto select clock source
        self.set_clock_source(CLK_AUTO)
        time.sleep(0.2)

        # Enable all sensors
        self.enable_sensors()
        time.sleep(0.2)
        
        self.set_gyro_mode(GFS_1000DPS)
        self.set_accel_mode(AFS_8G)
        self.set_gyro_temp_dlpf_cfg(DLPF_GT_184HZ_188HZ)
        self.set_smplrt_div(SAMPLE_RATE_1KHZ)
        time.sleep(0.1)

        # Enable bypass mode
        self.set_int_pin_bypass_en(True)
        time.sleep(0.1)

        # Set MST_CLK rate to 400kHz
#        self.write_register(I2C_MST_CTRL, 0x09)

    def self_test(self):
        accel_data = self.read_register_burst(SELF_TEST_X_ACCEL, 3)
        gyro_data = self.read_register_burst(SELF_TEST_X_GYRO, 3)
#        print("Self test:", accel_data, gyro_data)

    def calibrate(self, fifo_mode=False):
        if fifo_mode:
            self._calibrate_fifo()
        else:
            self._calibrate()

    def _calibrate_fifo(self, num_samples=512):
        """Calibrates sensors using FIFO by writing offsets to specific registers."""
        print("Calibrating, please do not move sensor...")
        accel_offs = [0.0, 0.0, 0.0]
        gyro_offs = [0.0, 0.0, 0.0]

        for n in range(num_samples):
            # 100 Hz => sleep time 0.01 to accumulate 1 sample (12 bytes)
            time.sleep(0.008)
            #            if i % 32 == 0:
            fifo_count = self.get_fifo_count()
#            print(fifo_count)
            if fifo_count % 12 != 0:
                self.set_fifo_reset()
                continue
            # empty FIFO and start again
            accel, gyro = self.read_fifo_raw()
            for i in range(3):
                accel_offs[i] += accel[i]
                gyro_offs[i] += gyro[i]

        # Normalize sums to get average count biases
        for i in range(3):
            accel_offs[i] /= num_samples
            gyro_offs[i] /= num_samples
        
        # Remove gravity from the z-axis accelerometer bias calculation
        if(accel_offs[2] > 0):
            accel_offs[2] -= int(1 / self.ares)
        else:
            accel_offs[2] += int(1 / self.ares)

        print("Calculdated offsets:", accel_offs, gyro_offs)

        for i in range(3):
            accel_offs[i] = accel_offs_fact[i] - (accel_offs[i] / 4)

        self.set_accel_offs([int(round(x)) for x in accel_offs])
        self.set_gyro_offs_usr([-int(round(x)) for x in gyro_offs])
                
    def _calibrate(self, num_samples=512):
        """Calibrates sensors by writing offsets to specific registers."""
        
        print("Calibrating, please do not move sensor...")
        accel_buff = [[], [], []]
        gyro_buff = [[], [], []]

        mean_accel = [0.0, 0.0, 0.0]
        mean_gyro = [0.0, 0.0, 0.0]
        accel_offs_fact = self.get_accel_offs()
        
        for i in range(num_samples):
            accel = self.read_accel_raw()
            gyro = self.read_gyro_raw()

            for j in range(3):
                accel_buff[j].append(accel[j])
                gyro_buff[j].append(gyro[j])

        # Normalize sums to get average count biases
        for i in range(3):
            mean_accel[i] = mean(accel_buff[i])
            mean_gyro[i] = mean(gyro_buff[i])
        
        print("Done.")
        print("AX (min={};mean={};max={}".format(
            min(accel_buff[0]),
            mean_accel[0],
            max(accel_buff[0])))
        print("AY (min={};mean={};max={}".format(
            min(accel_buff[1]),
            mean_accel[1],
            max(accel_buff[1])))
        print("AZ (min={};mean={};max={}".format(
            min(accel_buff[2]),
            mean_accel[2],
            max(accel_buff[2])))
        
        print("GX (min={};mean={};max={}".format(
            min(gyro_buff[0]),
            mean_gyro[0],
            max(gyro_buff[0])))
        print("GY (min={};mean={};max={}".format(
            min(gyro_buff[1]),
            mean_gyro[1],
            max(gyro_buff[1])))
        print("GZ (min={};mean={};max={}".format(
            min(gyro_buff[2]),
            mean_gyro[2],
            max(gyro_buff[2])))

        # Prepare accel offset data to write offsets to offset register
        # Remove gravity from the z-axis accelerometer bias calculation
        if(mean_accel[2] > 0):
            mean_accel[2] -= (1 / self.ares)
        else:
            mean_accel[2] += (1 / self.ares)
        
        for i in range(3):
            mean_accel[i] = accel_offs_fact[i] - (mean_accel[i] / 4)


        self.set_accel_offs([int(round(x)) for x in mean_accel])
        self.set_gyro_offs_usr([-int(round(x)) for x in mean_gyro])

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

        self.set_accel_fs_sel(mode)
        
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

        self.set_gyro_fs_sel(mode)

    def search_device(self):
        """Check if WHO_AM_I register read equals DEVICE_ID"""
        who_am_i = self.get_whoami()
        if(who_am_i == DEVICE_ID):
            return True
        else:
            return False

    def check_data_ready(self):
        """Checks if data is ready to be read from sensor"""
        drdy = self.read_register(INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    def read_accel_gyro(self):
        """Read gyroscope data and accelerometer data from FIFO."""
        _ax, _ay, _az, _gx, _gy, _gz = self.read_fifo_raw()
        ax = (_ax - self.accel_offs_x) * self.ares
        ay = (_ay - self.accel_offs_y) * self.ares
        az = (_az - self.accel_offs_z) * self.ares
        gx = (_gx - self.gyro_offs_x) * self.gres
        gy = (_gy - self.gyro_offs_y) * self.gres
        gz = (_gz - self.gyro_offs_z) * self.gres

        return (ax, ay, az, gx, gy, gz)
        
    def read_accel(self):
        """Read accelerometer data from ACCEL_OUT register."""
        _x, _y, _z = self.read_accel_raw()
        x = (_x - self.accel_offs[0]) * self.ares
#        x = _x * self.ares
        y = (_y - self.accel_offs[1]) * self.ares
#        y = _y * self.ares
        z = (_z - self.accel_offs[2]) * self.ares
#        z = _z * self.ares

        return (x, y, z)
    
    def read_gyro(self):
        """Read gyroscope data from GYRO_OUT register."""
        _x, _y, _z = self.read_gyro_raw()
        x = (_x - self.gyro_offs[0]) * self.gres
#        x = _x * self.gres
        y = (_y - self.gyro_offs[1]) * self.gres
#        y = _y * self.gres
        z = (_z - self.gyro_offs[2]) * self.gres
#        z = _z * self.gres

        return (x, y, z)

    def read_temperature(self):
        """Read temperature in degrees Celcius."""
        temp_raw = self.read_temp_raw()

        temp = temp_raw / 333.87 + 21.0
        return temp
