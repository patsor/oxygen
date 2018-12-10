#!/usr/bin/env python

import timeit

from mpu6500 import MPU6500

def main():
    n = 1000
    r = 3
    
    setup = "mpu6500 = MPU6500();"

    py = timeit.repeat("mpu6500.read_gyro()", setup = "from mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)

    print("read_gyro (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))

    py = timeit.repeat("mpu6500.read_accel()", setup = "from mpu6500 import MPU6500; {}".format(setup), repeat=r, number=n)

    print("read_accel (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))
    
if __name__ == "__main__":
    main()
