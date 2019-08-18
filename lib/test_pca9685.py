#!/sr/bin/env python

import time

from pca9685 import PCA9685

def main():
    driver = PCA9685()
    driver.set_pwm_freq(60)
    time.sleep(3)
    for i in range(300, 500, 5):
        print(i)
        driver.set_pwm(0, 0, i)
        time.sleep(0.1)
    for i in range(500, 299, -5):
        print(i)
        driver.set_pwm(0, 0, i)
        time.sleep(0.1)
#    driver.set_pwm(0, 0, 0)

if __name__ == "__main__":
    main()
