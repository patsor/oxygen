#!/usr/bin/env python

import timeit
#from ahrs import AHRS

def main():
    n = 1000
    r = 3
    
    setup = "ah = AHRS(1, 0.5); accel = (3.54553222656, 0.582092285156, 3.30932617188); gyro = (498.825073242, 1.17492675781, 0.190734863281); mag = (27.7515911172, 9.31261446886, 11050.9357849);"

    py = timeit.repeat("ah.get_quaternion(accel, gyro, mag)", setup = "from ahrs_py import AHRS; {}".format(setup), repeat=r, number=n)
    cy = timeit.repeat("ah._get_quaternion(accel, gyro, mag)", setup = "from ahrs import AHRS; {}".format(setup), repeat=r, number=n)
    print("get_quaternion (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))
    print("get_quaternion (cy): best of {}; {} usec per loop".format(r, min(cy) / n * 1000000))


if __name__ == "__main__":
    main()
