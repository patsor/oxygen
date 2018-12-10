"""Library for byte value conversions"""

def bytes_to_int16(b0, b1):
    """Convert two bytes to 16bit signed integer"""
    value = b0 | (b1 << 8)
    if value > 32767:
        value -= 65536
    return value
                            
