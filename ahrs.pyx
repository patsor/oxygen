"""
Library for sensor data fusion
The purpose of this library is to generate
usable values out of sensor data from
accelerometer, gyroscope and magnetometer
"""

import cython

from libc.math cimport sqrt, atan2, asin

cdef float radians(float val):
    """Converts degree value into radians"""
    return val * 0.0174533

cdef float degrees(float val):
    """Converts radian value into degrees"""
    return val * 57.29578

cdef class AHRS:
    cdef float q1, q2, q3, q4, beta, dt
    def __init__(self, float beta, float dt):
        self.q1 = 1.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.beta = beta
        self.dt = dt

    def _update_quaternion_madgwick(self, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz):
        """Python wrapper method for testing and timing purposes"""
        return self.update_quaternion_madgwick(ax, ay, az, gx, gy, gz, mx, my, mz)

    # Remove zero division checks
    # as we catch the exception already in the code
    @cython.cdivision(True)
    cdef update_quaternion_madgwick(self, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz):
        """Converts sensor data to quaternions using Madgwick algorithm"""
        cdef float q1, q2, q3, q4

        cdef float norm
        cdef float hx, hy, _2bx, _2bz
        cdef float s1, s2, s3, s4
        cdef float qDot1, qDot2, qDot3, qDot4
        
        # Auxiliary variables to avoid repeated arithmetic
        cdef float _2q1mx
        cdef float _2q1my
        cdef float _2q1mz
        cdef float _2q2mx
        cdef float _4bx
        cdef float _4bz
        cdef float _2q1, _2q2, _2q3, _2q4
        cdef float _2q1q3, _2q3q4
        cdef float q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4
        
        gx = radians(gx)
        gy = radians(gy)
        gz = radians(gz)
        
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        q4 = self.q4
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4
        
        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0/norm
        ax *= norm
        ay *= norm
        az *= norm
        
        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0/norm
        mx *= norm
        my *= norm
        mz *= norm
        
        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        norm = 1.0/norm
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4
        
        # Integrate to yield quaternion
        q1 += qDot1 * self.dt
        q2 += qDot2 * self.dt
        q3 += qDot3 * self.dt
        q4 += qDot4 * self.dt
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        norm = 1.0/norm
        self.q1 = q1 * norm
        self.q2 = q2 * norm
        self.q3 = q3 * norm
        self.q4 = q4 * norm


    def _update_quaternion_mahony(self, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz):
        """Python wrapper method for testing and timing purposes"""
        return self.update_quaternion_mahony(ax, ay, az, gx, gy, gz, mx, my, mz)
        
    # Remove zero division checks
    # as we catch the exception already in the code
    @cython.cdivision(True)
    cdef update_quaternion_mahony(self, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz):
        cdef (float, float, float) eInt = (0.0, 0.0, 0.0)
        cdef float Kp = 0.4 # these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
        cdef float Ki = 0.0
        cdef float q1, q2, q3, q4

        cdef float norm
        cdef float hx, hy, bx, bz
        cdef float vx, vy, vz, wx, wy, wz
        cdef float ex, ey, ez
        cdef float pa, pb, pc
        
        # Auxiliary variables to avoid repeated arithmetic
        cdef float q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4

        gx = radians(gx)
        gy = radians(gy)
        gz = radians(gz)
        
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        q4 = self.q4
        
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4
        
        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm
        
        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm
        
        # Reference direction of Earth's magnetic field
        hx = 2.0 * mx * (0.5 - q3q3 - q4q4) + 2.0 * my * (q2q3 - q1q4) + 2.0 * mz * (q2q4 + q1q3)
        hy = 2.0 * mx * (q2q3 + q1q4) + 2.0 * my * (0.5 - q2q2 - q4q4) + 2.0 * mz * (q3q4 - q1q2)
        bx = sqrt((hx * hx) + (hy * hy))
        bz = 2.0 * mx * (q2q4 - q1q3) + 2.0 * my * (q3q4 + q1q2) + 2.0 * mz * (0.5 - q2q2 - q3q3)
        
            # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)  
        
        # Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy)
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx)
        if Ki > 0.0:
            eInt[0] += ex      # accumulate integral error
            eInt[1] += ey
            eInt[2] += ez
        else: 
            eInt[0] = 0.0     # prevent integral wind up
            eInt[1] = 0.0
            eInt[2] = 0.0
        
        # Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0]
        gy = gy + Kp * ey + Ki * eInt[1]
        gz = gz + Kp * ez + Ki * eInt[2]
        
        # Integrate rate of change of quaternion
        pa = q2
        pb = q3
        pc = q4
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * self.dt)
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * self.dt)
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * self.dt)
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * self.dt)
        
        # Normalise quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        self.q1 = q1 * norm
        self.q2 = q2 * norm
        self.q3 = q3 * norm
        self.q4 = q4 * norm

    def _get_orientation(self):
        return self.get_orientation()
        
    cdef (float, float, float) get_orientation(self):
        cdef float yaw, pitch, roll
        yaw = degrees(atan2(2 * (self.q2*self.q3 + self.q1*self.q4), self.q1*self.q1 + self.q2*self.q2 - self.q3*self.q3 - self.q4*self.q4))
        pitch = degrees(-asin(2 * (self.q2*self.q4 - self.q1*self.q3)))
        roll = degrees(atan2(2 * (self.q1*self.q2 + self.q3*self.q4), self.q1*self.q1 - self.q2*self.q2 - self.q3*self.q3 + self.q4*self.q4))
        return (yaw, pitch, roll)
