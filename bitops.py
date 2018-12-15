"""
Library for byte value conversions
"""

def set_bit(val, offs):
    """Set bit at offset 'offs' in value"""
    return val | (1 << offs)

def set_bit_value(val, offs, value):
    """Set bit at offset 'offs' to a specific value in 'val'"""
    if value:
        return set_bit(val, offs)
    else:
        return clear_bit(val, offs)

def set_bits(val, offs, n, bit_value):
    """Set number of bits 'n' at offset 'offs' in value to 'bit_value'"""
    val = clear_bits(val, offs, n)
    val |= (bit_value << offs)
    return val

def clear_bit(val, offs):
    """Clear bit at offset 'offs' in value"""
    return val & ~(1 << offs)

def clear_bits(val, offs, n):
    """ Clear number of bits 'n' at specific offset 'offs' in value"""
    return val & ~((2**n - 1) << offs)

def toggle_bit(val, offs):
    """Toggle bit at offset 'offs' in value"""
    return val ^ (1 << offs)

def check_bit(val, offs):
    """Test bit at offset 'offs' in value"""
    return val & (1 << offs)

def bytes_to_int16(b0, b1):
    """Convert two bytes to 16-bit signed integer"""
    value = b0 | (b1 << 8)
    if value > 32767:
        value -= 65536
    return value

def int16_to_bytes(value):
    """Convert 16-bit signed integer to bytes"""
    data = [0, 0]
    data[0] = (value >> 8) & 0xff
    data[1] = value & 0xff
    return data

def bytes_to_uint16(b0, b1):
    """Convert two bytes to 16bit signed integer"""
    value = b0 | (b1 << 8)
    return value

def bytes_to_int32(b0, b1, b2, b3):
    """Convert two bytes to 16bit signed integer"""
    value = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    if value > 2147483647:
        value -= 4294967296
    return value

def bytes_to_uint32(b0, b1, b2, b3):
    """Convert two bytes to 16bit signed integer"""
    value = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    return value
