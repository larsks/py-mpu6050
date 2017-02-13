import micropython
import math
import time

class ComplementaryFilter(object):
    def __init__(self, gyro_weight=0.95):
        self.gyro_weight = gyro_weight
        self.reset()

    def reset(self):
        self.last = 0

        self.accel_pos = [0, 0, 0]
        self.gyro_pos = [0, 0, 0]
        self.filter_pos = [0, 0, 0]

    def reset_gyro(self):
        self.gyro_pos = self.filter_pos

    def input(self, vals):
        now = time.ticks_ms()

        # unpack sensor readings
        accel_data = vals[0:3]
        gyro_data = vals[4:7]

        # convert accelerometer reading to degrees
        self.accel_pos = self.calculate_accel_pos(*accel_data)

        # if this is our first chunk of data, simply accept
        # the accelerometer reads and move on.
        if self.last == 0:
            self.filter_pos = self.gyro_pos = self.accel_pos
            self.last = now
            return

        # calculate the elapsed time (in seconds) since last data.
        # we need this because the gyroscope measures movement in
        # degrees/second.
        dt = time.ticks_diff(now, self.last)/1000
        self.last = now

        # calculate change in position from gyroscope readings
        gyro_delta = [i * dt for i in gyro_data]
        self.gyro_pos = [i + j for i, j in zip(self.gyro_pos, gyro_delta)]

        # pitch
        self.filter_pos[0] = (
            self.gyro_weight * (self.filter_pos[0] + gyro_delta[0])
            + (1-self.gyro_weight) * self.accel_pos[0])

        # roll
        self.filter_pos[1] = (
            self.gyro_weight * (self.filter_pos[1] + gyro_delta[1])
            + (1-self.gyro_weight) * self.accel_pos[1])

    def calculate_accel_pos(self, x, y, z):
        x2 = (x*x);
        y2 = (y*y);
        z2 = (z*z);

        adx = math.atan2(y, z)
        ady = math.atan2(-x, math.sqrt(y2 + z2))

        return [math.degrees(x) for x in [adx, ady, 0]]
