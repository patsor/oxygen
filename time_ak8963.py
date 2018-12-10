#!/usr/bin/env python

import timeit
#from ahrs import AHRS

def main():
    n = 1000
    r = 3
    
    setup = "ak8963 = AK8963();"

    py = timeit.repeat("ak8963.read_magnet()", setup = "from ak8963 import AK8963; {}".format(setup), repeat=r, number=n)
    print("read_magnet (py): best of {}; {} usec per loop".format(r, min(py) / n * 1000000))

    
if __name__ == "__main__":
    main()
