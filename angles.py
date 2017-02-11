import micropython
import math
import time

class Angles(object):
    def __init__(self, gyro_weight=0.95, accel_weight=0.05):
        self.gyro_weight = gyro_weight
        self.accel_weight = accel_weight

        self.reset()

    def reset(self):
        self.last = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def input(self, vals):
        ax, ay, az, temp, gx, gy, gz = vals
        adx, ady, adz = self.accel_degrees(ax, ay, az)
        now = time.ticks_ms()

        if self.last == 0:
            self.pitch, self.roll, self.yaw = [adx, ady, 0]
            self.last = now
            return

        dt = time.ticks_diff(self.last, now)/1000
        self.last = now

        dpitch = gx * dt
        droll = gy * dt

        self.pitch = (self.gyro_weight * (self.pitch + dpitch)
                      + self.accel_weight * ady)
        self.roll = (self.gyro_weight * (self.roll + droll)
                     + self.accel_weight * adx)

    def accel_degrees(self, x, y, z):
        x2 = (x*x);
        y2 = (y*y);
        z2 = (z*z);

        # this is equation 37 (aka 26) from AN3461
        pitch = math.atan2(-x, math.sqrt(y2 + z2))

        # this is equation 38 from AN3461
        roll = math.atan2(y, (
            math.copysign(1, z)
            * (math.sqrt(z2 + (0.001)*x2))))

        return [math.degrees(x) for x in [pitch, roll, 0]]

    def angles(self):
        return [self.pitch, self.roll, self.yaw]
