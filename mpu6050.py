from machine import Pin, I2C, disable_irq, enable_irq
import time
import micropython
from ustruct import unpack

from constants import *
import angles

micropython.alloc_emergency_exception_buf(100)

default_pin_scl = 13
default_pin_sda = 12
default_pin_intr = 14
default_sample_rate = 0x20

default_calibration_samples = 100
default_calibration_accel_deadzone = 15
default_calibration_gyro_deadzone = 5

class MPU(object):
    def __init__(self, scl=None, sda=None,
                 intr=None, rate=None,
                 address=None):

        self.scl = scl if scl is not None else default_pin_scl
        self.sda = sda if sda is not None else default_pin_sda
        self.intr = intr if intr is not None else default_pin_intr
        self.rate = rate if rate is not None else default_sample_rate

        self.address = address if address else MPU6050_DEFAULT_ADDRESS
        self.bytebuf = bytearray(1)
        self.wordbuf = bytearray(2)
        self.sensors = bytearray(14)
        self.use_fifo = False
        self.calibration = [0] * 7

        self.angles = angles.Angles()

        self.init_pins()
        self.init_i2c()
        self.init_device()

    def init_i2c(self):
        print('* initializing i2c')
        self.bus = I2C(scl=self.pin_scl,
                       sda=self.pin_sda)

    def init_pins(self):
        print('* initializing pins')
        self.pin_sda = Pin(self.sda)
        self.pin_scl = Pin(self.scl)
        self.pin_intr = Pin(self.intr, mode=Pin.IN)

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
        shift = (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)
        val = self.read_byte(MPU6050_RA_GYRO_CONFIG)
        val &= ~(0b11 << shift)
        val |= fsr << shift
        self.write_byte(MPU6050_RA_GYRO_CONFIG, val)

    def set_accel_range(self, fsr):
        shift = (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)
        val = self.read_byte(MPU6050_RA_ACCEL_CONFIG)
        val &= ~(0b11 << shift)
        val |= fsr << shift
        self.write_byte(MPU6050_RA_ACCEL_CONFIG, val)

    def write_byte(self, reg, val):
        self.bytebuf[0] = val
        self.bus.writeto_mem(self.address, reg, self.bytebuf)

    def read_byte(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.bytebuf)
        return self.bytebuf[0]

    def read_word(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>H', self.wordbuf)[0]

    def read_word2(self, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>h', self.wordbuf)[0]

    def read_sensors(self):
        self.bus.readfrom_mem_into(self.address,
                                   MPU6050_RA_ACCEL_XOUT_H,
                                   self.sensors)

        data = unpack('>hhhhhhh', self.sensors)

        # apply calibration values
        return [data[i] + self.calibration[i] for i in range(7)]

    def read_sensors_scaled(self):
        data = self.read_sensors()
        data[0:3] = [x/(65536//2//2) for x in data[0:3]]
        data[4:7] = [x/(65536//250//2) for x in data[4:7]]
        return data

    def read_angles(self):
        self.angles.input(self.read_sensors_scaled())
        return self.angles.angles()

    def get_sensor_avg(self, samples, softstart=100):
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            time.sleep_ms(2)
            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return [x//samples for x in counters]

    def calibrate(self, samples=None, accel_deadzone=None, gyro_deadzone=None):
        print('* start calibration')

        self.calibration = [0] * 7

        samples = (samples if samples is not None
                   else default_calibration_samples)
        accel_deadzone = (accel_deadzone if accel_deadzone is not None
                          else default_calibration_accel_deadzone)
        gyro_deadzone = (gyro_deadzone if gyro_deadzone is not None
                         else default_calibration_gyro_deadzone)

        # These are what the sensors ought to read at rest
        # on a level surface
        expected = [0, 0, 16384, None, 0, 0, 0]

        # calculate offsets between the expected values and
        # the average value for each sensor reading
        avg = self.get_sensor_avg(samples)
        off = [0 if expected[i] is None else expected[i] - avg[i]
               for i in range(7)]

        accel_ready = False
        gyro_read = False
        for passno in range(20):
            self.calibration[:] = off
            avg = self.get_sensor_avg(samples)

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

            if not gyro_read:
                off[4:7] = [off[i] + check[i]//gyro_deadzone
                            for i in range(4, 7)]

        print('* calibrated!')
        self.set_state_calibrated()
