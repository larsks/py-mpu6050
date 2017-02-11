from machine import Pin, I2C, disable_irq, enable_irq
import time
import micropython
from ustruct import unpack

from constants import *

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

    def enable_fifo(self):
        print('* enable fifo')
        # enable writing values to fifo
        self.write_byte(MPU6050_RA_FIFO_EN, (
            (1 << MPU6050_TEMP_FIFO_EN_BIT) |
            (1 << MPU6050_XG_FIFO_EN_BIT) |
            (1 << MPU6050_YG_FIFO_EN_BIT) |
            (1 << MPU6050_ZG_FIFO_EN_BIT) |
            (1 << MPU6050_ACCEL_FIFO_EN_BIT)
        ))

        val = self.read_byte(MPU6050_RA_USER_CTRL)
        val |= (1 << MPU6050_USERCTRL_FIFO_EN_BIT)
        self.write_byte(MPU6050_RA_USER_CTRL, val)

        self.use_fifo = True

    def disable_fifo(self):
        print('* disable fifo')
        val = self.read_byte(MPU6050_RA_USER_CTRL)
        val &= ~(1 << MPU6050_USERCTRL_FIFO_EN_BIT)
        self.write_byte(MPU6050_RA_USER_CTRL, val)

        self.write_byte(MPU6050_RA_FIFO_EN, 0)

        self.use_fifo = False

    def reset_fifo(self):
        print('* reset fifo')
        self.disable_fifo()
        val = self.read_byte(MPU6050_RA_USER_CTRL)
        val &= ~(1 << MPU6050_USERCTRL_FIFO_RESET_BIT)
        self.write_byte(MPU6050_RA_USER_CTRL, val)

    def fifo_count(self):
        count = self.read_word(MPU6050_RA_FIFO_COUNTH)
        return count

    def disable_interrupt(self):
        print('* disabling data ready interrupt')
        val = self.read_byte(MPU6050_RA_INT_ENABLE)
        self.write_byte(MPU6050_RA_INT_ENABLE, (
            val & ~(
                (1 << MPU6050_INTERRUPT_DATA_RDY_BIT) |
                (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)
            )
        ));

        self.pin_intr.irq(handler=None)

    def enable_interrupt(self):
        print('* enabling data ready interrupt')
        self.pin_intr.irq(handler=self.isr, trigger=Pin.IRQ_RISING)

        val = self.read_byte(MPU6050_RA_INT_ENABLE)
        self.write_byte(MPU6050_RA_INT_ENABLE, (
            val | (
                (1 << MPU6050_INTERRUPT_DATA_RDY_BIT) |
                (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)
            )
        ));

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

    def isr(self, pin):
        irqs = self.read_byte(MPU6050_RA_INT_STATUS)

        if irqs & (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT):
            print('overflow!')

        if irqs & (1 << MPU6050_INTERRUPT_DATA_RDY_BIT):
            self._read_sensor_fifo()

    def read_sensors(self):
        if not self.use_fifo:
            self._read_sensor_regs()

        irq_state = disable_irq()
        data = bytearray(self.sensors)
        enable_irq(irq_state)

        data = unpack('>hhhhhhh', data)
        return [data[i] + self.calibration[i] for i in range(7)]

    def _read_sensor_regs(self):
        self.bus.readfrom_mem_into(self.address,
                                   MPU6050_RA_ACCEL_XOUT_H,
                                   self.sensors)

        return self.sensors

    def _read_sensor_fifo(self):
        self.bus.readfrom_mem_into(self.address,
                                   MPU6050_RA_FIFO_R_W,
                                   self.sensors)

        return self.sensors

    def _get_sensor_avg(self, samples, softstart=100):
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

        expected = [0, 0, 16384, None, 0, 0, 0]

        avg = self._get_sensor_avg(samples)
        off = [0 if expected[i] is None else expected[i] - avg[i]
               for i in range(7)]

        accel_ready = False
        gyro_read = False
        for passno in range(20):
            self.calibration[:] = off
            avg = self._get_sensor_avg(samples)

            check = [0 if expected[i] is None else expected[i] - avg[i]
                   for i in range(7)]
            print('- pass {}: {}'.format(passno, check))

            accel_ready = all(abs(x) < accel_deadzone
                              for x in check[0:3])
            gyro_ready = all(abs(x) < gyro_deadzone
                             for x in check[4:7])

            if accel_ready and gyro_ready:
                break

            if not accel_ready:
                off[0:3] = [off[i] + check[i]//accel_deadzone for i in range(3)]

            if not gyro_read:
                off[4:7] = [off[i] + check[i]//gyro_deadzone for i in range(4, 7)]

        print('* calibrated!')
