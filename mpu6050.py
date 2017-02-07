#!/usr/bin/python

import machine
import math
import time
import ucollections as collections
import ustruct as struct

default_pin_scl = 2
default_pin_sda = 0

Register = collections.namedtuple('Register',
                                  ('address', 'size',
                                   'read', 'write',
                                   'txout', 'txin'))

def to_2c(val):
    return val - (1 << 16)

def to_2c_array(val):
    pass

def from_bytes(buf):
    val = (buf[0] << 8) + buf[1]
    return val

def from_2c(buf):
    val = (buf[0] << 8) + buf[1]

    if (val >= 0x8000):
        val = -(((2**16 - 1) - val) + 1)

    return val

def from_2c_array(buf):
    bufiter = iter(buf)
    res = []
    for vh, vl in zip(bufiter, bufiter):
        val = (vh << 8) + vl
        if (val >= 0x8000):
            val = -(((2**16 - 1) - val) + 1)

        res.append(val)

    return res

registers = {
    'power_mgmt_1': Register(0x6b, 1, True, True, None, None),
    'power_mgmt_2': Register(0x6c, 1, True, True, None, None),
    'accel_config': Register(0x1c, 1, True, True, None, None),
    'gyro_config': Register(0x18, 1, True, True, None, None),
    'int_enable': Register(0x38, 1, True, True, None, None),
    'int_status': Register(0x3A, 1, True, False, None, None),
    'user_ctrl': Register(0x6A, 1, True, True, None, None),
    'fifo_count': Register(0x72, 2, True, True, from_bytes, None),
    'fifo_rw': Register(0x74, 1, True, True, None, None),
    'sample_rate_div': Register(0x19, 1, True, True, None, None),
    'config': Register(0x1A, 1, True, True, None, None),

    'accel_x': Register(0x3b, 2, True, False, from_2c, None),
    'accel_y': Register(0x3d, 2, True, False, from_2c, None),
    'accel_z': Register(0x3f, 2, True, False, from_2c, None),

    'temp': Register(0x41, 2, True, False, from_2c, None),

    'gyro_x': Register(0x43, 2, True, False, from_2c, None),
    'gyro_y': Register(0x45, 2, True, False, from_2c, None),
    'gyro_z': Register(0x47, 2, True, False, from_2c, None),

    'sensors': Register(0x3b, 14, True, False, from_2c_array, None),
    'accel_all': Register(0x3b, 6, True, False, from_2c_array, None),
    'gyro_all': Register(0x43, 6, True, False, from_2c_array, None),
}

# scale in deg/s
gyro_scale = [250, 500, 1000, 2000]

# scale in g
accel_scale = [2, 4, 8, 16]

class MPU6050(dict):
    temp_divider = 340
    temp_offset = 35

    def __init__(self, scl=None, sda=None, address=0x68):
        scl = scl if scl is not None else default_pin_scl
        sda = sda if sda is not None else default_pin_sda

        self.bus = machine.I2C(scl=machine.Pin(scl),
                               sda=machine.Pin(sda))
        self.address = address
        self.buf = memoryview(bytearray(16))

        # used by the complementary filter
        self.lastsample = time.ticks_ms()

        self.init_device()
        self.init_filter()

    def __repr__(self):
        return '<mpu6050 @ %02X>' % self.address

    def __getitem__(self, k):
        reg = registers[k]
        if not reg.read:
            raise NotImplementedError('register %s cannot be read', k)

        data = self.read(reg.address, reg.size)
        if callable(reg.txout):
            data = reg.txout(data)
        elif reg.size == 1:
            data = data[0]

        return data

    def __setitem__(self, k, v):
        reg = registers[k]
        if not reg.write:
            raise NotImplementedError('register %s cannot be written', k)

        if callable(reg.txin):
            v = reg.txin(v)
        elif isinstance(v, int):
            v = bytearray([v])

        self.write(reg.address, v)

    def read(self, reg, length):
        if length > len(self.buf):
            raise ValueError('Length must be <= {}'.format(len(m)))

        self.bus.readfrom_mem_into(self.address, reg, self.buf[:length])
        return self.buf[:length]

    def write(self, reg, val):
        self.bus.writeto_mem(self.address, reg, val)

    def keys(self):
        return registers.keys()

    def init_device(self):
        self['power_mgmt_1'] = 0

    def set_dlpf(self, val):
        cfg = (self['config'] & 0b11111000) | val
        self['config'] = cfg

    def read_gyro_raw(self):
        return self['gyro_all']

    def read_gyro_scaled(self):
        val = self.read_gyro_raw()
        scalei = (self['gyro_config'] & 0b00011000) >> 3
        scalev = gyro_scale[scalei]

        return [x/(65536//scalev//2) for x in val]

    def set_gyro_scale(self, scale):
        if scale not in gyro_scale:
            raise ValueError(scale)

        i = gyro_scale.index(scale)
        cfg = (self['gyro_config'] & 0b11100111) | (i<<3)
        self['gyro_config'] = cfg

    def read_accel_raw(self):
        return self['accel_all']

    def read_accel_scaled(self):
        val = self.read_accel_raw()
        scalei = (self['accel_config'] & 0b00011000) >> 3
        scalev = accel_scale[scalei]

        return [x/(65536//scalev//2) for x in val]

    def set_accel_scale(self, scale):
        if scale not in accel_scale:
            raise ValueError(scale)

        i = accel_scale.index(scale)
        cfg = (self['accel_config'] & 0b11100111) | (i<<3)
        self['accel_config'] = cfg

    def read_temp_raw(self):
        val = self['temp']
        return val

    def read_temp_scaled(self):
        val = self.read_temp_raw()
        return (val/self.temp_divider) + self.temp_offset

    # from: http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
    # and: http://www.nxp.com/assets/documents/data/en/application-notes/AN3461.pdf
    def read_accel_rad(self):
        x, y, z = self.read_accel_scaled()

        x2 = (x*x);
        y2 = (y*y);
        z2 = (z*z);

        # this is equation 37 (aka 26) from AN3461
        pitch = math.atan2(-x, math.sqrt(y2 + z2))

        # this is equation 38 from AN3461
        roll = math.atan2(y, (
            math.copysign(1, z)
            * (math.sqrt(z2 + (0.001)*x2))))

        return [pitch, roll, 0]

    def read_accel_deg(self):
        return [math.degrees(v) for v in self.read_accel_rad()]

    def init_filter(self):
        self.lastsample = time.ticks_ms()
        self.pitch, self.roll, self.yaw = self.read_accel_deg()

    gyro_weight = 0.7
    accel_weight = 0.3

    def read_deg(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(self.lastsample, now)/1000
        self.lastsample = now

        gyro = self.read_gyro_scaled()
        accel = self.read_accel_deg()

        accel_pitch = accel[0]
        accel_roll = accel[1]

        dpitch = gyro[0] * dt
        droll = gyro[1] * dt

        self.pitch = (self.gyro_weight * (self.pitch + dpitch)
                      + self.accel_weight * accel_roll)
        self.roll = (self.gyro_weight * (self.roll + droll)
                     + self.accel_weight * accel_pitch)

        return [self.pitch, self.roll, self.yaw]
