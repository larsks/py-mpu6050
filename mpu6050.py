#!/usr/bin/python

import machine
import math
import time
import ucollections as collections
import ustruct as struct

default_pin_scl = 2
default_pin_sda = 0

Register = collections.namedtuple('Register', ('address', 'size', 'read', 'write'))

registers = {
    'power_mgmt_1': Register(0x6b, 1, True, True),
    'power_mgmt_2': Register(0x6c, 1, True, True),

    'gyro_config': Register(0x18, 1, True, True),
    'gyro_x': Register(0x43, 2, True, False),
    'gyro_y': Register(0x45, 2, True, False),
    'gyro_z': Register(0x47, 2, True, False),

    'accel_config': Register(0x1c, 1, True, True),
    'accel_x': Register(0x3b, 2, True, False),
    'accel_y': Register(0x3d, 2, True, False),
    'accel_z': Register(0x3f, 2, True, False),

    'temp': Register(0x41, 2, True, False),

    'int_enable': Register(0x38, 1, True, True),
    'int_status': Register(0x3A, 1, True, False),

    'user_ctrl': Register(0x6A, 1, True, True),
    'fifo_count': Register(0x72, 2, True, True),
    'fifo_rw': Register(0x74, 1, True, True),

    'sample_rate_div': Register(0x19, 1, True, True),
    'config': Register(0x1A, 1, True, True),
}

# scale in deg/s
gyro_scale = [250, 500, 1000, 2000]

# scale in g
accel_scale = [2, 4, 8, 16]

def to_2c(val):
    return val - (1 << 16)

def from_2c(val):
    if (val >= 0x8000):
        val = -(((2**16 - 1) - val) + 1)

    return val

class MPU6050(dict):
    temp_divider = 340
    temp_offset = 35

    def __init__(self, scl=None, sda=None, address=0x68, init=True):
        scl = scl if scl is not None else default_pin_scl
        sda = sda if sda is not None else default_pin_sda

        self.bus = machine.I2C(scl=machine.Pin(scl),
                               sda=machine.Pin(sda))
        self.address = address
        self.buf1 = bytearray([0])
        self.buf2 = bytearray([0, 0])

        if init:
            self.init_device()

    def __repr__(self):
        return '<mpu6050 @ %02X>' % self.address

    def __getitem__(self, k):
        reg = registers[k]
        if not reg.read:
            raise NotImplementedError('register %s cannot be read', k)

        if reg.size == 2:
            return self.read_word(reg.address)
        else:
            return self.read_byte(reg.address)

    def __setitem__(self, k, v):
        reg = registers[k]
        if not reg.write:
            raise NotImplementedError('register %s cannot be written', k)

        if reg.size == 2:
            self.write_word(reg.address, v)
        else:
            self.write_byte(reg.address, v)

    def keys(self):
        return registers.keys()

    def init_device(self):
        self['power_mgmt_1'] = 0

    def write_byte(self, reg, val):
        self.buf1[0] = val
        self.bus.writeto_mem(self.address, reg, self.buf1)

    def write_word(self, reg, val):
        val = to_2c(val)
        struct.pack_into('!h', self.buf2, 0, val)
        self.bus.writeto_mem(self.address, reg, self.buf2)

    def read_byte(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.buf1)
        return self.buf1[0]

    def read_word(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.buf2)
        val = (self.buf2[0] << 8) + self.buf2[1]
        return from_2c(val)

    def read_gyro_raw(self):
        val = [
            self['gyro_x'], self['gyro_y'], self['gyro_z']
        ]

        return val

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
        val = [
            self['accel_x'], self['accel_y'], self['accel_z']
        ]

        return val

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

    # from: http://www.hobbytronics.co.uk/accelerometer-info
    def read_accel_rad(self):
       x, y, z = self.read_accel_scaled()

       x2 = (x*x);
       y2 = (y*y);
       z2 = (z*z);

       ax = math.atan(x/(math.sqrt(y2 + z2)));
       ay = math.atan(y/(math.sqrt(x2 + z2)));
       az = math.atan(z/(math.sqrt(x2 + y2)));

       return [ax, ay, az]

    def read_accel_deg(self):
        return [math.degrees(v) for v in self.read_accel_rad()]

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

