import unittest
from bitops import *

class TestBitOps(unittest.TestCase):
    def setUp(self):
        pass

    def test_set_bit(self):
        self.assertEqual(set_bit(0x00, 2), 0x04)
        self.assertEqual(set_bit(0x03, 6), 0x43)
        self.assertEqual(set_bit(0x7f, 7), 0xff)

    def test_set_bit_value(self):
        self.assertEqual(set_bit_value(0x00, 2, True), 0x04)
        self.assertEqual(set_bit_value(0x03, 1, False), 0x01)
        self.assertEqual(set_bit_value(0x7f, 7, True), 0xff)

    def test_set_bits(self):
        self.assertEqual(set_bits(0x00, 2, 3, 0x07), 0x1c)
        self.assertEqual(set_bits(0x00, 0, 4, 0x0e), 0x0e)
        self.assertEqual(set_bits(0x00, 4, 2, 0x02), 0x20)
        self.assertEqual(set_bits(0x44, 0, 2, 0x03), 0x47)
        self.assertEqual(set_bits(0x77, 4, 4, 0x0f), 0xf7)
        self.assertEqual(set_bits(0xf0, 0, 5, 0x1f), 0xff)

    def test_clear_bit(self):
        self.assertEqual(clear_bit(0x01, 0), 0x00)
        self.assertEqual(clear_bit(0x80, 7), 0x00)
        self.assertEqual(clear_bit(0xff, 7), 0x7f)

    def test_clear_bits(self):
        self.assertEqual(clear_bits(0x03, 0, 2), 0x00)
        self.assertEqual(clear_bits(0x80, 0, 7), 0x80)
        self.assertEqual(clear_bits(0xff, 3, 2), 0xe7)

    def test_toggle_bit(self):
        self.assertEqual(toggle_bit(0x01, 0), 0x00)
        self.assertEqual(toggle_bit(0xff, 7), 0x7f)
        self.assertEqual(toggle_bit(0x03, 1), 0x01)
        self.assertEqual(toggle_bit(0xf7, 3), 0xff)

    def test_check_bit(self):
        self.assertEqual(check_bit(0x07, 2), True)
        self.assertEqual(check_bit(0x7f, 7), False)
        self.assertEqual(check_bit(0x11, 4), True)
        self.assertEqual(check_bit(0xe2, 5), True)

    def test_bytes_to_int16(self):
        self.assertEqual(bytes_to_int16(255, 255), -1)
        self.assertEqual(bytes_to_int16(255, 0), 255)
        self.assertEqual(bytes_to_int16(10, 48), 12298)
        self.assertEqual(bytes_to_int16(0, 255), -256)
        self.assertEqual(bytes_to_int16(0, 0), 0)
        self.assertEqual(bytes_to_int16(0, 164), -23552)

    def test_int16_to_bytes(self):
        self.assertEqual(int16_to_bytes(0), (0, 0))
        self.assertEqual(int16_to_bytes(-256), (0, 255))
        self.assertEqual(int16_to_bytes(12298), (10, 48))
        self.assertEqual(int16_to_bytes(255), (255, 0))
        self.assertEqual(int16_to_bytes(-1), (255, 255))

    def test_bytes_to_uint15(self):
        self.assertEqual(bytes_to_uint15(255, 255), 32767)
        self.assertEqual(bytes_to_uint15(0, 164), 0)
        self.assertEqual(bytes_to_uint15(0, 128), 0)
        self.assertEqual(bytes_to_uint15(10, 193), 0)
        self.assertEqual(bytes_to_uint15(255, 255), 0)
        
    def test_bytes_to_uint16(self):
        self.assertEqual(bytes_to_uint16(200, 0), 200)
        self.assertEqual(bytes_to_uint16(0, 255), 65280)
        self.assertEqual(bytes_to_uint16(0, 128), 32768)
        self.assertEqual(bytes_to_uint16(10, 193), 49418)
        self.assertEqual(bytes_to_uint16(255, 255), 65535)

if __name__ == "__main__":
    unittest.main()
