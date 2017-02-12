import micropython
import math
import time

class ComplementaryFilter(object):
    def __init__(self, gyro_weight=0.95):
        self.gyro_weight = gyro_weight
        self.reset()

    def reset(self):
        self.last = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def input(self, vals):
        now = time.ticks_ms()

        # unpack sensor readings
        ax, ay, az, temp, gx, gy, gz = vals

        # convert accelerometer reads to degrees
        adx, ady, adz = self.accel_degrees(ax, ay, az)

        # if this is our first chunk of data, simply accept
        # the accelerometer reads and move on.
        if self.last == 0:
            self.pitch, self.roll, self.yaw = [adx, ady, 0]
            self.last = now
            return

        # calculate the elapsed time (in seconds) since last data.
        # we need this because the gyroscope measures movement in
        # degrees/second.
        dt = time.ticks_diff(now, self.last)/1000
        self.last = now

        # calculate change in position from gyroscope readings
        dpitch = gx * dt
        droll = gy * dt

        self.pitch = (self.gyro_weight * (self.pitch + dpitch) +
                      (1-self.gyro_weight) * ady)
        self.roll = (self.gyro_weight * (self.roll + droll) +
                      (1-self.gyro_weight) * adx)

    def accel_degrees(self, x, y, z):
        x2 = (x*x);
        y2 = (y*y);
        z2 = (z*z);

        # roll = rotation about x axis = changes in y angle
        ady = math.atan2(y, z)

        # pitch = rotation about y axis = changes in x angle
        adx = math.atan2(-x, math.sqrt(y2 + z2))

        return [math.degrees(x) for x in [adx, ady, 0]]

    @property
    def position(self):
        return [self.pitch, self.roll, self.yaw]
