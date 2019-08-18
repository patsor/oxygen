#!/usr/bin/env python

import smbus

from bitops import *

class I2CDevice(object):
    def __init__(self, address):
        self.bus = smbus.SMBus(1)
        self.address = address
    
    def read_register(self, register):
        """Reads value from register."""
        return self.bus.read_byte_data(self.address, register)
    
    def write_register(self, register, value):
        """Writes value to register."""
        self.bus.write_byte_data(self.address, register, value)
        
    def read_register_burst(self, register, n):
        """Reads n bytes from register with auto-increment."""
        return self.bus.read_i2c_block_data(self.address, register, n)
    
    def write_register_burst(self, register, data):
        """Writes n bytes to register with auto-increment."""
        self.bus.write_i2c_block_data(self.address, register, data)
        
    def read_register_bit(self, register, index):
        """Reads specific bit at 'index' from register."""
        data = self.read_register(register)
        return check_bit(data, index)
    
    def write_register_bit(self, register, index, value):
        """Write 'value' into specific bit at 'index' to register."""
        data = self.read_register(register)
        val = set_bit_value(data, index, value)
        self.write_register(register, val)
