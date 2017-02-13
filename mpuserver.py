import micropython
micropython.alloc_emergency_exception_buf(100)

from machine import Pin, reset, disable_irq, enable_irq
import gc

from mpu6050 import MPU

import socket
import select
import time

default_port = 8000
default_irq_pin = 4
default_write_interval = 10
default_gc_interval = 1000

def tojson(values):
    inner = []
    for item in values:
        msg = ('['
               + (', '.join(str(x) for x in item))
               + ']')
        inner.append(msg)

    return ('[' + ','.join(inner) + ']\n')


class MPUServer(object):
    def __init__(self, mpu,
                 port=default_port,
                 write_interval=default_write_interval,
                 gc_interval=default_gc_interval,
                 irq_pin=default_irq_pin):
        self.mpu = mpu
        self.port = port
        self.write_interval = write_interval
        self.gc_interval = gc_interval
        self.irq_pin = irq_pin
        self.last_isr = 0
        self.flag_reset_gyro = False
        self.init_pins()
        self.init_socket()

        self.mpu.calibrate()

    def __repr__(self):
        return '<{} @ {}>'.format(self.__class__.__name__, self.port)

    def init_pins(self):
        self.pin_irq = Pin(self.irq_pin, Pin.IN, Pin.PULL_UP)
        self.pin_irq.irq(handler=self.isr, trigger=Pin.IRQ_FALLING)

    def init_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def isr(self, pin):
        # debounce
        if time.ticks_diff(time.ticks_ms(), self.last_isr) < 10:
            return

        print('! reset gyro request')
        self.flag_reset_gyro = True
        self.last_isr = time.ticks_ms()

    def serve(self):
        print('starting mpu server on port {}'.format(self.port))

        lastgc = lastsent = lastread = time.ticks_ms()

        while True:
            now = time.ticks_ms()
            write_dt = time.ticks_diff(now, lastsent)
            read_dt = time.ticks_diff(now, lastread)
            gc_dt = time.ticks_diff(now, lastgc)

            time.sleep_ms(max(0, 1-read_dt))

            if self.flag_reset_gyro:
                self.mpu.filter.reset_gyro()
                self.flag_reset_gyro = False

            values = self.mpu.read_position()
            lastread = now

            if write_dt >= self.write_interval:
                lastsent = time.ticks_ms()
                self.sock.sendto(tojson(values), ('192.168.4.2', 8000))

            if gc_dt >= self.gc_interval:
                gc.collect()
