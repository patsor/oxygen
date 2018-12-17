import unittest
from mpu6500 import MPU6500

class TestMPU6500(unittest.TestCase):
    def setUp(self):
        self.mpu6500 = MPU6500()

    def test_get_set_fifo_en(self):
        self.mpu6500.set_fifo_en(False)
        self.assertEqual(self.mpu6500.get_fifo_en(), False)

        self.mpu6500.set_fifo_en(True)
        self.assertEqual(self.mpu6500.get_fifo_en(), True)

    def test_get_set_smplrt_div(self):
        self.mpu6500.set_smplrt_div(20)
        self.assertEqual(self.mpu6500.get_smplrt_div(), 20)
        self.mpu6500.set_smplrt_div(10)
        self.assertEqual(self.mpu6500.get_smplrt_div(), 10)


if __name__ == "__main__":
    unittest.main()
