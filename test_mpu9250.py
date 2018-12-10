#!/usr/bin/env python
"""
Example usage of MPU9250 library
"""

from __future__ import print_function
import time
import sys

from mpu9250 import MPU9250
from ahrs import AHRS

def busy_wait(dt):
    current_time = time.time()
    while (time.time() < current_time + dt):
        pass

def main():
    mpu9250 = MPU9250()
    beta = 0.9067
    dt = 0.002
    t0 = time.time()
    t1 = 0.0
    delta_t = 0.0
    i = 0
    ahrs = AHRS(beta, dt)
    try:
        while True:

            (ax, ay, az) = mpu9250.read_accel()
#            print("Accel: {}, {}, {}".format(ax, ay, az))
                  
            (gx, gy, gz) = mpu9250.read_gyro()
#            print("Gyro: {}, {}, {}".format(gx, gy, gz))

            (mx, my, mz) = mpu9250.read_magnet()
#            print("Mag: {}, {}, {}".format(mx, my, mz))

#            print(dt, delta_t, dt - delta_t)
            t1 = time.time()
            delta_t = t1 - t0
#            print(delta_t)
            busy_wait(dt - delta_t)
            
            ahrs._update_quaternion_madgwick(ax, ay, az, gx, gy, gz, mx, my, mz)
#            t2 = time.time()


            if i % 1000 == 0:
                (yaw, pitch, roll) = ahrs._get_orientation()
                print(yaw, pitch, roll)
                
            i += 1
            t0 = time.time()



#            print()

#            time.sleep(dt - delta_t)

    except KeyboardInterrupt:
        sys.exit()

if __name__ == "__main__":
    main()
