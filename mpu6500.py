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
XG_OFFSET_H = 0x13
XG_OFFSET_L = 0x14
YG_OFFSET_H = 0x15
YG_OFFSET_L = 0x16
ZG_OFFSET_H = 0x17
ZG_OFFSET_L = 0x18
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
XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E


# Sample Rate Modes
SAMPLE_RATE_1KHZ = 0x00

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

class MPU6500:

    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configure_fifo()
#        self.configure()
        self.self_test()
        self.calibrate_fifo()
#        self.calibrate()

    def read_register_burst(self, register, n):
        """Reads n bytes from register with auto-increment"""
        return bus.read_i2c_block_data(self.address, register, n)

    def write_register_burst(self, register, data):
        """Writes n bytes to register with auto-increment"""
        bus.write_i2c_block_data(self.address, register, data)

    def read_register(self, register):
        """Reads value from register"""
        return bus.read_byte_data(self.address, register)

    def write_register(self, register, value):
        """Writes value to register"""
        bus.write_byte_data(self.address, register, value)

    def read_register_bit(self, register, index):
        """Reads specific bit at 'index' from register"""
        data = self.read_register(register)
        return check_bit(data, index)

    def write_register_bit(self, register, index, value):
        """Write 'value' into specific bit at 'index' to register"""
        data = self.read_register(register)
        val = set_bit_value(data, index, value)
        self.write_register(register, val)

    def get_self_test_x_gyro(self):
        """
        Get gyroscope x-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_X_GYRO)

    def set_self_test_x_gyro(self, value):
        """Set gyroscope x-axis self test value"""
        self.write_register(SELF_TEST_X_GYRO, value)

    def get_self_test_y_gyro(self):
        """
        Get gyroscope y-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Y_GYRO)

    def set_self_test_y_gyro(self, value):
        """Set gyroscope y-axis self test value"""
        self.write_register(SELF_TEST_Y_GYRO, value)

    def get_self_test_z_gyro(self):
        """
        Get gyroscope z-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Z_GYRO)

    def set_self_test_z_gyro(self, value):
        """Set gyroscope z-axis self test value"""
        self.write_register(SELF_TEST_Z_GYRO, value)

    def get_self_test_gyro(self):
        """Get gyroscope self test output from all axis"""
        return self.read_register_burst(SELF_TEST_X_GYRO, 3)

    def set_self_test_gyro(self, values):
        """Set gyroscope self test values of all axis"""
        bus.write_i2c_block_data(self.address, SELF_TEST_X_GYRO, values)

    def get_self_test_x_accel(self):
        """
        Get accelerometer x-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_X_ACCEL)

    def set_self_test_x_accel(self, value):
        """Set accelerometer x-axis self test value"""
        self.write_register(SELF_TEST_X_ACCEL, value)

    def get_self_test_y_accel(self):
        """
        Get accelerometer y-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Y_ACCEL)

    def set_self_test_y_accel(self, value):
        """Set accelerometer y-axis self test value"""
        self.write_register(SELF_TEST_Y_ACCEL, value)

    def get_self_test_z_accel(self):
        """
        Get accelerometer z-axis self test output
        generated during manufacturing tests.
        """
        return self.read_register(SELF_TEST_Z_ACCEL)

    def set_self_test_z_accel(self, value):
        """Set accelerometer z-axis self test value"""
        self.write_register(SELF_TEST_Z_ACCEL, value)

    def get_self_test_accel(self):
        """Get accelerometer self test output from all axis"""
        return self.read_register_burst(SELF_TEST_X_ACCEL, 3)

    def set_self_test_accel(self, values):
        """Set accelerometer self test values of all axis"""
        bus.write_i2c_block_data(self.address, SELF_TEST_X_ACCEL, values)

    def get_xg_offs_usr(self):
        data = self.read_register_burst(XG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_xg_offs_usr(self, value):
        data = int16_to_bytes(value)
        bus.write_i2c_block_data(self.address, XG_OFFS_USR_H, data)

    def get_yg_offs_usr(self):
        data = self.read_register_burst(YG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_yg_offs_usr(self, value):
        data = int16_to_bytes(value)
        bus.write_i2c_block_data(self.address, YG_OFFS_USR_H, data)

    def get_zg_offs_usr(self):
        data = self.read_register_burst(ZG_OFFS_USR_H, 2)
        return bytes_to_int16(data[1], data[0])

    def set_zg_offs_usr(self, value):
        data = int16_to_bytes(value)
        bus.write_i2c_block_data(self.address, ZG_OFFS_USR_H, data)

    def get_gyro_offs_usr(self):
        data = self.read_register_burst(ZG_OFFS_USR_H, 6)
        x = bytes_to_int16(data[1], data[0])
        y = bytes_to_int16(data[3], data[2])
        z = bytes_to_int16(data[5], data[4])
        return (x, y, z)

    def set_gyro_offs_usr(self, values):
        x = int16_to_bytes(values[0])
        y = int16_to_bytes(values[1])
        z = int16_to_bytes(values[2])
        data = [x[0], x[1], y[0], y[1], z[0], z[1]]
        bus.write_i2c_block_data(self.address, XG_OFFS_USR_H, data)

    def get_smplrt_div(self):
        return self.read_register(SMPLRT_DIV)

    def set_smplrt_div(self, value):
        self.write_register(SMPLRT_DIV, value)

    def get_fifo_mode(self):
        """Get FIFO write mode"""
        return self.read_register_bit(CONFIG, 6)
        
    def set_fifo_mode(self, value):
        """Set FIFO write mode"""
        self.write_register_bit(CONFIG, 6, value)

    def get_ext_fsync_set(self):
        data = self.read_register(CONFIG)
        return data & 0x38

    def set_ext_fsync_set(self, value):
        data = self.read_register(CONFIG)
        val = set_bits(data, 3, 3, value)
        self.write_register(CONFIG, val)

    def get_gyro_temp_dlpf_cfg(self):
        data = self.read_register(CONFIG)
        return data & 0x07

    def set_gyro_temp_dlpf_cfg(self, value):
        data = self.read_register(CONFIG)
        val = set_bits(data, 0, 3, value)
        self.write_register(CONFIG, val)

    def get_gx_st_en(self):
        return read_register_bit(GYRO_CONFIG, 7)

    def set_gx_st_en(self, value):
        self.write_register_bit(GYRO_CONFIG, 7, value)

    def get_gy_st_en(self):
        return self.read_register_bit(GYRO_CONFIG, 6)

    def set_gy_st_en(self, value):
        self.write_register_bit(GYRO_CONFIG, 6, value)

    def get_gz_st_en(self):
        return self.read_register_bit(GYRO_CONFIG, 5)

    def set_gz_st_en(self, value):
        self.write_register_bit(GYRO_CONFIG, 5, value)

    def get_gyro_fs_sel(self):
        data = self.read_register(GYRO_CONFIG)
        return data & 0x18

    def set_gyro_fs_sel(self, value):
        data = self.read_register(GYRO_CONFIG)
        val = set_bits(data, 3, 2, value)
        self.write_register(GYRO_CONFIG, val)

    def get_gyro_fchoice_b(self):
        data = self.read_register(GYRO_CONFIG)
        return data & 0x03

    def set_gyro_fchoice_b(self, value):
        data = self.read_register(GYRO_CONFIG)
        val = set_bits(data, 0, 2, value)
        self.write_register(GYRO_CONFIG, val)

    def get_ax_st_en(self):
        return self.read_register_bit(ACCEL_CONFIG, 7)

    def set_ax_st_en(self, value):
        self.write_register_bit(ACCEL_CONFIG, 7, value)

    def get_ay_st_en(self):
        return self.read_register_bit(ACCEL_CONFIG, 6)

    def set_ay_st_en(self, value):
        self.write_register_bit(ACCEL_CONFIG, 6, value)

    def get_az_st_en(self):
        return self.read_register_bit(ACCEL_CONFIG, 5)

    def set_az_st_en(self, value):
        self.write_register_bit(ACCEL_CONFIG, 5, value)

    def get_accel_fs_sel(self):
        data = self.read_register(ACCEL_CONFIG)
        return data & 0x18

    def set_accel_fs_sel(self, value):
        data = self.read_register(ACCEL_CONFIG)
        val = set_bits(data, 3, 2, value)
        self.write_register(ACCEL_CONFIG, val)

    def get_accel_fchoice_b(self):
        return self.read_register_bit(ACCEL_CONFIG_2, 3)

    def set_accel_fchoice_b(self, value):
        self.write_register_bit(ACCEL_CONFIG_2, 3, value)

    def get_accel_dlpf_cfg(self):
        data = self.read_register(ACCEL_CONFIG_2)
        return data & 0x07
        
    def set_accel_dlpf_cfg(self, value):
        data = self.read_register(ACCEL_CONFIG_2)
        val = set_bits(data, 0, 3, value)
        self.write_register(ACCEL_CONFIG_2, val)

    def get_accel_lp_odr(self):
        data = self.read_register(LP_ACCEL_ODR)
        return data & 0x0f

    def set_accel_lp_odr(self, value):
        data = self.read_register(LP_ACCEL_ODR)
        val = set_bits(data, 0, 4, value)
        self.write_register(LP_ACCEL_ODR, val)

    def get_wom_threshold(self):
        return self.read_register(WOM_THR)
        
    def set_wom_threshold(self, value):
        self.write_register(WOM_THR, value)

    def get_temp_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 7)

    def set_temp_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 7, value)

    def get_gx_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 6)

    def set_gx_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 6, value)

    def get_gy_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 5)

    def set_gy_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 5, value)

    def get_gz_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 4)

    def set_gz_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 4, value)

    def get_accel_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 3)

    def set_accel_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 3, value)

    def get_slv2_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 2)

    def set_slv2_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 2, value)

    def get_slv1_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 1)

    def set_slv1_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 1, value)

    def get_slv0_fifo_en(self):
        return self.read_register_bit(FIFO_EN, 0)

    def set_slv0_fifo_en(self, value):
        self.write_register_bit(FIFO_EN, 0, value)

    def set_accel_gyro_fifo_en(self):
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
        data = self.read_register(FIFO_COUNTH, 2)
        return bytes_to_int16(data[1], data[0])

    def get_fifo_rw(self):
        return self.read_register(FIFO_R_W)

    def set_fifo_rw(self, value):
        self.write_register(FIFO_RW, value)

    def get_whoami(self):
        return self.read_register(WHOAMI)

    def get_ax_offset(self):
        data = self.read_register_burst(XA_OFFS_H, 2)
        return bytes_to_int16(data[1], data[0])

    def get_ay_offset(self):
        data = self.read_register_burst(YA_OFFS_H, 2)
        return bytes_to_int16(data[1], data[0])

    def get_az_offset(self):
        data = self.read_register_burst(ZA_OFFS_H, 2)
        return bytes_to_int16(data[1], data[0])

    def get_accel_offset(self):
        data = self.read_register_burst(XA_OFFS_H, 6)
        x = bytes_to_int16(data[1], data[0])
        y = bytes_to_int16(data[3], data[2])
        z = bytes_to_int16(data[5], data[4])
        return (x, y, z)


        
    def enable_fifo(self):
        """Enable FIFO mode"""
        self.set_fifo_en(True)
        
        # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes)
        self.set_accel_gyro_fifo_en()

    def disable_i2c_master(self):
        """Disable I2C Master"""
        self.write_register(I2C_MST_CTRL, 0x00)
        self.set_i2c_mst_en(False)
        
    def configure_fifo(self):
        """Configure MPU9250 in FIFO mode"""

        self.set_h_reset()
        time.sleep(0.1)
        # auto select clock source
        self.set_clock_source(0x01)

        # Enable all sensors
        self.enable_sensors()
        time.sleep(0.2)

        # Configure device for bias calculation
        self.disable_interrupts()
        self.set_fifo_en(False)
        self.set_clock_source(0x00)
        self.disable_i2c_master()
        self.set_fifo_reset()
        time.sleep(0.015)
        
        # Configure MPU6050 gyro and accelerometer for bias calculation
        self.set_fifo_mode(FIFO_MODE_NO_OVERWRITE)
        self.set_gyro_temp_dlpf_cfg(0x01)

        self.set_smplrt_div(SAMPLE_RATE_1KHZ)

        #        self.write_register(GYRO_CONFIG, 0x00)
        #        self.write_register(ACCEL_CONFIG, 0x00)

        self.set_gyro_mode(GFS_1000DPS)
        self.set_accel_mode(AFS_8G)
        self.set_int_pin_bypass_en()
        time.sleep(0.1)
        self.enable_fifo()
        
    def configure(self):
        """Configure MPU9250"""

        # Write a one to bit 7 reset bit; toggle reset device
        self.write_register(PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Clear sleep mode bit, enable all sensors
        self.write_register(PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # auto select clock source
        self.write_register(PWR_MGMT_1, 0x01)
        time.sleep(0.2)

        # Enable all sensors
        self.write_register(PWR_MGMT_2, 0x00)
        time.sleep(0.2)

        self.set_gyro_mode(GFS_1000DPS)
        self.set_accel_mode(AFS_8G)

        # DLPF_CFG (Gryo Bandwidth: 41Hz, Gyro Delay: 5.9ms,FS: 1kHz. Temp Bandwidth: 42Hz. Temp Delay: 4.8ms)
#        self.write_register(CONFIG, 0x03)
        # sample rate divider - Sample Rate: Internal Sample Rate / (1 + SMPLRT_DIV)
        self.write_register(SMPLRT_DIV, 0x00)
        time.sleep(0.1)

        # Enable bypass mode
        self.write_register(INT_PIN_CFG, 0x02)
        time.sleep(0.1)

        # Set MST_CLK rate to 400kHz
#        self.write_register(I2C_MST_CTRL, 0x09)

    def self_test(self):
        accel_data = self.read_register_burst(SELF_TEST_X_ACCEL, 3)
        gyro_data = self.read_register_burst(SELF_TEST_X_GYRO, 3)
#        print("Self test:", accel_data, gyro_data)

    def calibrate_fifo(self, num_samples=512):
        """Calibrates sensors using FIFO by writing offsets to specific registers"""
        accel_offs_x = 0.0
        accel_offs_y = 0.0
        accel_offs_z = 0.0
        gyro_offs_x = 0.0
        gyro_offs_y = 0.0
        gyro_offs_z = 0.0

#        print(num_samples)

#        print(fifo_count)
#        packet_count = fifo_count / 12 # How many sets of full gyro and accelerometer data for averaging
        i = 0
        while i < num_samples:
            self.write_register(FIFO_EN, 0x78)     # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
            time.sleep(0.04) # accumulate 40 samples in 40 milliseconds = 480 bytes
        
            # At end of sample accumulation, turn off FIFO sensor read
            self.write_register(FIFO_EN, 0x00)        # Disable gyro and accelerometer sensors for FIFO
#            while self.get_fifo_count() < 12:
#                pass
            fifo_count = self.get_fifo_count()
            num_packets = fifo_count / 12
            if i + num_packets > num_samples:
                num_packets = num_samples - i
            i += num_packets
#            print(fifo_count)
            for n in range(num_packets):
                data = self.read_register_burst(FIFO_R_W, 12) # read data for averaging
#                print(data)

            # [247, 32, 11, 130, 7, 106, 255, 208, 0, 35, 0, 11]
            # Correct order
            # ([247, 26, 11, 151, 7, 85], [255, 205, 0, 40, 0, 7])
            # => (-2280.076171875, 2959.427734375, -2228.927734375, -42.455078125, 39.6640625, 8.689453125)
            
                ax = bytes_to_int16(data[1], data[0])
                ay = bytes_to_int16(data[3], data[2])
                az = bytes_to_int16(data[5], data[4])
                gx = bytes_to_int16(data[7], data[6])
                gy = bytes_to_int16(data[9], data[8])
                gz = bytes_to_int16(data[11], data[10])

#            print(ax, ay, az, gx, gy, gz)
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
#        print(accel_offs_x, accel_offs_y, accel_offs_z, gyro_offs_x, gyro_offs_y, gyro_offs_z)
                
    def calibrate(self, num_samples=512):
        """Calibrates sensors by writing offsets to specific registers"""

        accel_offs_x = 0.0
        accel_offs_y = 0.0
        accel_offs_z = 0.0
        gyro_offs_x = 0.0
        gyro_offs_y = 0.0
        gyro_offs_z = 0.0

        for i in range(num_samples):
            ax, ay, az = self.read_accel_raw()
#            accel_data = self.read_register_burst(ACCEL_XOUT_H, 6)
            gx, gy, gz = self.read_gyro_raw()
#            gyro_data = self.read_register_burst(GYRO_XOUT_H, 6)
#            time.sleep(0.01)
#            ax = bytes_to_int16(accel_data[1], accel_data[0])
#            ay = bytes_to_int16(accel_data[3], accel_data[2])
#            az = bytes_to_int16(accel_data[5], accel_data[4])
#            gx = bytes_to_int16(gyro_data[1], gyro_data[0])
#            gy = bytes_to_int16(gyro_data[3], gyro_data[2])
#            gz = bytes_to_int16(gyro_data[5], gyro_data[4])
#            print(ax, ay, az, gx, gy, gz)
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

        print(accel_offs_x, accel_offs_y, accel_offs_z, gyro_offs_x, gyro_offs_y, gyro_offs_z)

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

        self.write_register(ACCEL_CONFIG, mode << 3)
        
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
#       self.write_register(GYRO_CONFIG, mode << 3)
            
    def set_accel_bias(self, accel_bias):
        accel_bias_fact = self.read_accel_offset()
        data = [0, 0, 0, 0, 0, 0]
        for i in range(3):
            accel_reg_bias = accel_bias_fact[i] - (accel_bias[i] & ~1)
            print("Set accel bias:", i, accel_reg_bias)
            data[i*2] = (accel_reg_bias >> 8) & 0xff
            data[i*2 + 1] = (accel_reg_bias & temp_comp_mask[i]) & 0xff
        self.write_register_burst(XA_OFFSET_H, data)

    def set_gyro_bias(self, gyro_bias):
        data = [0, 0, 0, 0, 0, 0]
        for i in range(3):
            data[i*2] = (-gyro_bias[i] >> 8) & 0xff
            data[i*2 + 1] = -gyro_bias[i] & 0xff
        self.write_register_burst(XG_OFFSET_H, data)

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
        """Read gyroscope data and accelerometer data from FIFO"""
        data = self.read_register_burst(FIFO_R_W, 12)
        ax = (bytes_to_int16(data[1], data[0]) - self.accel_offs_x) * self.ares
        ay = (bytes_to_int16(data[3], data[2]) - self.accel_offs_y) * self.ares
        az = (bytes_to_int16(data[5], data[4]) - self.accel_offs_z) * self.ares
        gx = (bytes_to_int16(data[7], data[6]) - self.gyro_offs_x) * self.gres
        gy = (bytes_to_int16(data[9], data[8]) - self.gyro_offs_y) * self.gres
        gz = (bytes_to_int16(data[11], data[10]) - self.gyro_offs_z) * self.gres

        return (ax, ay, az, gx, gy, gz)
        
    def read_accel(self):
        """Read accelerometer data from ACCEL_OUT register"""
        data = self.read_register_burst(ACCEL_XOUT_H, 6)
        x = (bytes_to_int16(data[1], data[0]) - self.accel_offs_x) * self.ares
        y = (bytes_to_int16(data[3], data[2]) - self.accel_offs_y) * self.ares
        z = (bytes_to_int16(data[5], data[4]) - self.accel_offs_z) * self.ares

        return (x, y, z)
    
    def read_gyro(self):
        """Read gyroscope data from GYRO_OUT register"""
        data = self.read_register_burst(GYRO_XOUT_H, 6)
        x = (bytes_to_int16(data[1], data[0]) - self.gyro_offs_x) * self.gres
        y = (bytes_to_int16(data[3], data[2]) - self.gyro_offs_y) * self.gres
        z = (bytes_to_int16(data[5], data[4]) - self.gyro_offs_z) * self.gres

        return (x, y, z)

    def read_temperature(self):
        """Read temperature in degrees C"""
        data = self.read_register_burst(TEMP_OUT, 2)
        temp = bytes_to_int16(data[1], data[0])

        temp = temp / 333.87 + 21.0
        return temp
