"""
Library for sensor data fusion
The purpose of this library is to generate
usable values out of sensor data from
accelerometer, gyroscope and magnetometer
"""

import cython

from libc.math cimport sqrt, atan2, asin

cdef double radians(double val):
    """Converts degree value into radians"""
    return val * 0.0174533

cdef double degrees(double val):
    """Converts radian value into degrees"""
    return val * 57.29578

cdef class AHRS:
    cdef double q1, q2, q3, q4, beta, dt
    def __init__(self, double beta, double dt):
        self.q1 = 1.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.beta = beta
        self.dt = dt

    def _get_quaternion(self, (double, double, double) accel, (double, double, double) gyro, (double, double, double) mag):
        """Python wrapper method for testing and timing purposes"""
        return self.get_quaternion(accel, gyro, mag)

    # Remove zero division checks
    # as we catch the exception already in the code
    @cython.cdivision(True)        
    cdef (double, double, double) get_quaternion(self, (double, double, double) accel, (double, double, double) gyro, (double, double, double) mag):
        """Converts sensor data to quaternions using Madgwick algorithm"""
        # Todo: Test Mahony algorithm for sensor fusion
        cdef double mx, my, mz
        cdef double ax, ay, az
        cdef double gx, gy, gz
        cdef double _2q1, _2q2, _2q3, _2q4
        cdef double _2q1q3, _2q3q4
        cdef double q1q1, q1q2, q1q3, q1q4
        cdef double q2q2, q2q3, q2q4
        cdef double q3q3, q3q4
        cdef double q4q4
        cdef double norm
        cdef double _2q1mx, _2q1my, _2q1mz, _2qq2mx
        cdef double hx, hy
        cdef double _2bx, _2bz, _4bx, _4bz
        cdef double s1, s2, s3, s4
        cdef double qDot1, qDot2, qDot3, qDot4
        cdef double yaw, pitch, roll

        # Accelerometer and magnetometer units irrelevant
        # as they are normalised
        ax, ay, az = accel
        mx, my, mz = mag
        # Convert gyroscope values from degrees to radians
        gx = radians(gyro[0])
        gy = radians(gyro[1])
        gz = radians(gyro[2])
#        gx, gy, gz = gyro
        # Load quaternion values into local variable
        # to avoid too many self calls
        # TODO: Test if this is necessary
#        q1, q2, q3, q4 = self.q1, self.q2, self.q3, self.q4

        # Normalise accelerometer measurement
        # TODO: Test if values (double) can be 0, if this makes sense
        # TODO: Test reciprocal norm factor and sqrt approximation
        norm = sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return (0, 0, 0)
        
        ax /= norm
        ay /= norm
        az /= norm
        
        # Normalise magnetometer measurement
        # TODO: Test if values (double) can be 0, if this makes sense
        # TODO: Test reciprocal norm factor and sqrt approximation
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return (0, 0, 0)

        mx /= norm
        my /= norm
        mz /= norm

        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * self.q1
        _2q2 = 2 * self.q2
        _2q3 = 2 * self.q3
        _2q4 = 2 * self.q4
        _2q1q3 = 2 * self.q1 * self.q3
        _2q3q4 = 2 * self.q3 * self.q4
        q1q1 = self.q1 * self.q1
        q1q2 = self.q1 * self.q2
        q1q3 = self.q1 * self.q3
        q1q4 = self.q1 * self.q4
        q2q2 = self.q2 * self.q2
        q2q3 = self.q2 * self.q3
        q2q4 = self.q2 * self.q4
        q3q3 = self.q3 * self.q3
        q3q4 = self.q3 * self.q4
        q4q4 = self.q4 * self.q4
	
        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * self.q1 * mx
        _2q1my = 2 * self.q1 * my
        _2q1mz = 2 * self.q1 * mz
        _2q2mx = 2 * self.q2 * mx
        hx = mx*q1q1 - _2q1my*self.q4 + _2q1mz*self.q3 + mx*q2q2 + _2q2*my*self.q3 + _2q2*mz*self.q4 - mx*q3q3 - mx*q4q4
        hy = _2q1mx*self.q4 + my*q1q1 - _2q1mz*self.q2 + _2q2mx*self.q3 - my*q2q2 + my*q3q3 + _2q3*mz*self.q4 - my*q4q4
        _2bx = sqrt(hx*hx + hy*hy)
        _2bz = -_2q1mx*self.q3 + _2q1my*self.q2 + mz*q1q1 + _2q2mx*self.q4 - mz*q2q2 + _2q3*my*self.q4 - mz*q3q3 + mz*q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz
    
        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * self.q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * self.q4 + _2bz * self.q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * self.q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * self.q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * self.q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * self.q3 + _2bz * self.q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * self.q4 - _4bz * self.q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * self.q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * self.q3 - _2bz * self.q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * self.q2 + _2bz * self.q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * self.q1 - _4bz * self.q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * self.q4 + _2bz * self.q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * self.q1 + _2bz * self.q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * self.q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        # normalise step magnitude
        # TODO: Test if sqrt approximation or reciprocal norm factor is faster
        norm = sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        s1 /= norm
        s2 /= norm
        s3 /= norm
        s4 /= norm
        
        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-self.q2*gx - self.q3*gy - self.q4*gz) - self.beta*s1
        qDot2 = 0.5 * (self.q1*gx + self.q3*gz - self.q4*gy) - self.beta*s2
        qDot3 = 0.5 * (self.q1*gy - self.q2*gz + self.q4*gx) - self.beta*s3
        qDot4 = 0.5 * (self.q1*gz + self.q2*gy - self.q3*gx) - self.beta*s4
        
        # Integrate to yield quaternion
        self.q1 += qDot1 * self.dt
        self.q2 += qDot2 * self.dt
        self.q3 += qDot3 * self.dt
        self.q4 += qDot4 * self.dt
        
        # normalise quaternion
        # TODO: Test reciprocal norm factor and sqrt approximation
        norm = sqrt(self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3 + self.q4*self.q4)
    
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm
        self.q4 /= norm
        yaw = degrees(atan2(self.q2*self.q3 + self.q1*self.q4, 0.5 - self.q3*self.q3 - self.q4*self.q4))
        pitch = degrees(-asin(-2.0 * (self.q2*self.q4 - self.q1*self.q3)))
        roll = degrees(atan2(self.q1*self.q2 + self.q3*self.q4, 0.5 - self.q1*self.q1 - self.q2*self.q2))
        return (yaw, pitch, roll)
