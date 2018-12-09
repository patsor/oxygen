#!/usr/bin/env python

import timeit
#from ahrs import AHRS

def main():
    n = 1000
    r = 3
    
    setup = "ah = AHRS(0.9, 0.1); ax = 3.54553222656; ay = 0.582092285156; az = 3.30932617188; gx = 498.825073242; gy = 1.17492675781; gz = 0.190734863281; mx = 27.7515911172; my = 9.31261446886; mz = 11050.9357849;"

    py_mahony = timeit.repeat("ah.update_quaternion_mahony(ax, ay, az, gx, gy, gz, mx, my, mz)", setup = "from ahrs_py import AHRS; {}".format(setup), repeat=r, number=n)
    cy_mahony = timeit.repeat("ah._update_quaternion_mahony(ax, ay, az, gx, gy, gz, mx, my, mz)", setup = "from ahrs import AHRS; {}".format(setup), repeat=r, number=n)
    py_madgwick = timeit.repeat("ah.update_quaternion_madgwick(ax, ay, az, gx, gy, gz, mx, my, mz)", setup = "from ahrs_py import AHRS; {}".format(setup), repeat=r, number=n)
    cy_madgwick = timeit.repeat("ah._update_quaternion_madgwick(ax, ay, az, gx, gy, gz, mx, my, mz)", setup = "from ahrs import AHRS; {}".format(setup), repeat=r, number=n)
    print("get_quaternion_mahony (py): best of {}; {} usec per loop".format(r, min(py_mahony) / n * 1000000))
    print("get_quaternion_mahony (cy): best of {}; {} usec per loop".format(r, min(cy_mahony) / n * 1000000))
    print("get_quaternion_madgwick (py): best of {}; {} usec per loop".format(r, min(py_madgwick) / n * 1000000))
    print("get_quaternion_madgwick (cy): best of {}; {} usec per loop".format(r, min(cy_madgwick) / n * 1000000))


if __name__ == "__main__":
    main()
