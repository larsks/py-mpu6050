import gc
from machine import Pin, I2C, PWM
import time
import micropython
from ustruct import unpack

from constants import *
import cfilter

micropython.alloc_emergency_exception_buf(100)

default_pin_scl = 13
default_pin_sda = 12
default_pin_intr = 14
default_pin_led = 5
default_sample_rate = 0x20

default_calibration_numsamples = 200
default_calibration_accel_deadzone = 15
default_calibration_gyro_deadzone = 5

accel_range = [2, 4, 8, 16]
gyro_range = [250, 500, 1000, 2000]

# These are what the sensors ought to read at rest
# on a level surface
expected = [0, 0, 16384, None, 0, 0, 0]

class CalibrationFailure(Exception):
    pass

class MPU(object):
    def __init__(self, scl=None, sda=None,
                 intr=None, led=None, rate=None,
                 address=None):

        self.scl = scl if scl is not None else default_pin_scl
        self.sda = sda if sda is not None else default_pin_sda
        self.intr = intr if intr is not None else default_pin_intr
        self.led = led if led is not None else default_pin_led
        self.rate = rate if rate is not None else default_sample_rate

        self.address = address if address else MPU6050_DEFAULT_ADDRESS

        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(14)

        self.calibration = [0] * 7

        self.filter = cfilter.ComplementaryFilter()

        self.init_pins()
        self.init_led()
        self.init_i2c()
        self.init_device()

    def write_byte(self, reg, val):
        self.bytebuf[0] = val
        self.bus.writeto_mem(self.address, reg, self.bytebuf)

    def read_byte(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.bytebuf)
        return self.bytebuf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2**length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def read_word(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>H', self.wordbuf)[0]

    def read_word2(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>h', self.wordbuf)[0]

    def init_i2c(self):
        print('* initializing i2c')
        self.bus = I2C(scl=self.pin_scl,
                       sda=self.pin_sda)

    def init_pins(self):
        print('* initializing pins')
        self.pin_sda = Pin(self.sda)
        self.pin_scl = Pin(self.scl)
        self.pin_intr = Pin(self.intr, mode=Pin.IN)
        self.pin_led = PWM(Pin(self.led, mode=Pin.OUT))

    def set_state_uncalibrated(self):
        self.pin_led.freq(1)
        self.pin_led.duty(500)

    def set_state_calibrating(self):
        self.pin_led.freq(10)
        self.pin_led.duty(500)

    def set_state_calibrated(self):
        self.pin_led.freq(1000)
        self.pin_led.duty(500)

    def set_state_disabled(self):
        self.pin_led.duty(0)

    def init_led(self):
        self.set_state_uncalibrated()

    def identify(self):
        print('* identifying i2c device')
        val = self.read_byte(MPU6050_RA_WHO_AM_I)
        if val != MPU6050_ADDRESS_AD0_LOW:
            raise OSError("No mpu6050 at address {}".format(self.address))

    def reset(self):
        print('* reset')
        self.write_byte(MPU6050_RA_PWR_MGMT_1, (
            (1 << MPU6050_PWR1_DEVICE_RESET_BIT)
        ))
        time.sleep_ms(100)

        self.write_byte(MPU6050_RA_SIGNAL_PATH_RESET, (
            (1 << MPU6050_PATHRESET_GYRO_RESET_BIT) |
            (1 << MPU6050_PATHRESET_ACCEL_RESET_BIT) |
            (1 << MPU6050_PATHRESET_TEMP_RESET_BIT)
        ))
        time.sleep_ms(100)

    def init_device(self):
        print('* initializing mpu')

        self.identify()

        # disable sleep mode and select clock source
        self.write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO)

        # enable all sensors
        self.write_byte(MPU6050_RA_PWR_MGMT_2, 0)

        # set sampling rate
        self.write_byte(MPU6050_RA_SMPLRT_DIV, self.rate)

        # enable dlpf
        self.write_byte(MPU6050_RA_CONFIG, 1)

        # explicitly set accel/gyro range
        self.set_accel_range(MPU6050_ACCEL_FS_2)
        self.set_gyro_range(MPU6050_GYRO_FS_250)

    def set_gyro_range(self, fsr):
        self.gyro_range = gyro_range[fsr]
        self.set_bitfield(MPU6050_RA_GYRO_CONFIG,
                          MPU6050_GCONFIG_FS_SEL_BIT,
                          MPU6050_GCONFIG_FS_SEL_LENGTH,
                          fsr)

    def set_accel_range(self, fsr):
        self.accel_range = accel_range[fsr]
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_AFS_SEL_BIT,
                          MPU6050_ACONFIG_AFS_SEL_LENGTH,
                          fsr)

    def read_sensors(self):
        self.bus.readfrom_mem_into(self.address,
                                   MPU6050_RA_ACCEL_XOUT_H,
                                   self.sensors)

        data = unpack('>hhhhhhh', self.sensors)

        # apply calibration values
        return [data[i] + self.calibration[i] for i in range(7)]

    def read_sensors_scaled(self):
        data = self.read_sensors()
        data[0:3] = [x/(65536//self.accel_range//2) for x in data[0:3]]
        data[4:7] = [x/(65536//self.gyro_range//2) for x in data[4:7]]
        return data

    def read_position(self):
        self.filter.input(self.read_sensors_scaled())
        return [
            self.filter.filter_pos,
            self.filter.accel_pos,
            self.filter.gyro_pos,
        ]

    def set_dhpf_mode(self, bandwidth):
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_ACCEL_HPF_BIT,
                          MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
                          bandwidth)

    def set_motion_detection_threshold(self, threshold):
        self.write_byte(MPU6050_RA_MOT_THR, threshold)

    def set_motion_detection_duration(self, duration):
        self.write_byte(MPU6050_RA_MOT_DUR, duration)

    def set_int_motion_enabled(self, enabled):
        self.set_bitfield(MPU6050_RA_INT_ENABLE,
                          MPU6050_INTERRUPT_MOT_BIT,
                          1,
                          enabled)

    def get_sensor_avg(self, samples, softstart=100):
        '''Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle.'''
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return [x//samples for x in counters]

    stable_reading_timeout = 10
    max_gyro_variance = 5

    def wait_for_stable(self, numsamples=10):
        print('* waiting for gyros to stabilize')

        gc.collect()
        time_start = time.time()
        samples = []

        while True:
            now = time.time()
            if now - time_start > self.stable_reading_timeout:
                raise CalibrationFailure()

            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            samples.append(sample[4:7])
            if len(samples) < numsamples:
                continue

            samples = samples[-numsamples:]

            totals = [0] * 3
            for cola, colb in zip(samples, samples[1:]):
                deltas = [abs(a-b) for a,b in zip(cola, colb)]
                totals = [a+b for a,b in zip(deltas, totals)]

            avg = [a/numsamples for a in totals]

            if all(x < self.max_gyro_variance for x in avg):
                break

        now = time.time()
        print('* gyros stable after {:0.2f} seconds'.format(now-time_start))

    def calibrate(self,
                  numsamples=None,
                  accel_deadzone=None,
                  gyro_deadzone=None):

        old_calibration = self.calibration
        self.calibration = [0] * 7

        numsamples = (numsamples if numsamples is not None
                   else default_calibration_numsamples)
        accel_deadzone = (accel_deadzone if accel_deadzone is not None
                          else default_calibration_accel_deadzone)
        gyro_deadzone = (gyro_deadzone if gyro_deadzone is not None
                         else default_calibration_gyro_deadzone)

        print('* start calibration')
        self.set_state_calibrating()

        try:
            self.wait_for_stable()
            gc.collect()

            # calculate offsets between the expected values and
            # the average value for each sensor reading
            avg = self.get_sensor_avg(numsamples)
            off = [0 if expected[i] is None else expected[i] - avg[i]
                   for i in range(7)]

            accel_ready = False
            gyro_read = False
            for passno in range(20):
                self.calibration = off
                avg = self.get_sensor_avg(numsamples)

                check = [0 if expected[i] is None else expected[i] - avg[i]
                       for i in range(7)]
                print('- pass {}: {}'.format(passno, check))

                # check if current values are within acceptable offsets
                # from the expected values
                accel_ready = all(abs(x) < accel_deadzone
                                  for x in check[0:3])
                gyro_ready = all(abs(x) < gyro_deadzone
                                 for x in check[4:7])

                if accel_ready and gyro_ready:
                    break

                if not accel_ready:
                    off[0:3] = [off[i] + check[i]//accel_deadzone
                                for i in range(3)]

                if not gyro_ready:
                    off[4:7] = [off[i] + check[i]//gyro_deadzone
                                for i in range(4, 7)]
            else:
                raise CalibrationFailure()
        except CalibrationFailure:
            self.calibration = old_calibration
            print('! calibration failed')
            self.set_state_uncalibrated()
            return

        print('* calibrated!')
        self.set_state_calibrated()
