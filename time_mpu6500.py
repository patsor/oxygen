#!/usr/bin/env python

import timeit

#from mpu6500 import MPU6500

def main():
    n = 10
    r = 1
    
    setup = "mpu6500 = MPU6500(0x68, False);"

#    py = timeit.repeat("mpu6500.calibrate_fifo()", setup = "from mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)
    # Calibration values: (-2278.703125, 2964.201171875, -2233.0859375, -42.568359375, 39.556640625, 7.669921875)
#    print("calibrate_fifo (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))

    py = timeit.repeat("mpu6500.read_gyro()", setup = "from lib.mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)

    print("read_gyro (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))

    py = timeit.repeat("mpu6500.read_accel()", setup = "from lib.mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)

    print("read_accel (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))

#    py = timeit.repeat("mpu6500.read_accel_gyro()", setup = "from mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)


#    print("read_accel_gyro (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))
    
if __name__ == "__main__":
    main()
