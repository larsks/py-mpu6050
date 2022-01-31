import collections
import time
from ustruct import unpack

import constants as C

DEFAULT_SAMPLE_RATE = 0x20

ACCEL_RANGE = [2, 4, 8, 16]
GYRO_RANGE = [250, 500, 1000, 2000]


class SensorReadings(
    collections.namedtuple(
        "SensorReadings", ["AccX", "AccY", "AccZ", "Temp", "GyroX", "GyroY", "GyroZ"]
    )
):
    pass


class MPU(object):
    stable_reading_timeout = 10
    max_gyro_variance = 5

    def __init__(self, i2c, rate=None, address=None):

        self.rate = rate if rate is not None else DEFAULT_SAMPLE_RATE
        self.address = address if address else C.MPU6050_DEFAULT_ADDRESS
        self.i2c = i2c

        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(14)

        self.init_device()

    def write_byte(self, reg, val):
        self.bytebuf[0] = val
        self.i2c.writeto_mem(self.address, reg, self.bytebuf)

    def read_byte(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.bytebuf)
        return self.bytebuf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2 ** length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def read_word(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack(">H", self.wordbuf)[0]

    def read_word2(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack(">h", self.wordbuf)[0]

    def identify(self):
        val = self.read_byte(C.MPU6050_RA_WHO_AM_I)
        if val != C.MPU6050_ADDRESS_AD0_LOW:
            raise OSError("No mpu6050 at address {}".format(self.address))

    def reset(self):
        self.write_byte(
            C.MPU6050_RA_PWR_MGMT_1, ((1 << C.MPU6050_PWR1_DEVICE_RESET_BIT))
        )
        time.sleep_ms(100)  # type: ignore[attr-defined]

        self.write_byte(
            C.MPU6050_RA_SIGNAL_PATH_RESET,
            (
                (1 << C.MPU6050_PATHRESET_GYRO_RESET_BIT)
                | (1 << C.MPU6050_PATHRESET_ACCEL_RESET_BIT)
                | (1 << C.MPU6050_PATHRESET_TEMP_RESET_BIT)
            ),
        )
        time.sleep_ms(100)  # type: ignore[attr-defined]

    def init_device(self):
        self.identify()

        # disable sleep mode and select clock source
        self.write_byte(C.MPU6050_RA_PWR_MGMT_1, C.MPU6050_CLOCK_PLL_XGYRO)

        # enable all sensors
        self.write_byte(C.MPU6050_RA_PWR_MGMT_2, 0)

        # set sampling rate
        self.write_byte(C.MPU6050_RA_SMPLRT_DIV, self.rate)

        # enable dlpf
        self.write_byte(C.MPU6050_RA_CONFIG, 1)

        # explicitly set accel/gyro range
        self.set_accel_range(C.MPU6050_ACCEL_FS_2)
        self.set_gyro_range(C.MPU6050_GYRO_FS_250)

    def set_gyro_range(self, fsr):
        self.gyro_range = GYRO_RANGE[fsr]
        self.set_bitfield(
            C.MPU6050_RA_GYRO_CONFIG,
            C.MPU6050_GCONFIG_FS_SEL_BIT,
            C.MPU6050_GCONFIG_FS_SEL_LENGTH,
            fsr,
        )

    def set_accel_range(self, fsr):
        self.accel_range = ACCEL_RANGE[fsr]
        self.set_bitfield(
            C.MPU6050_RA_ACCEL_CONFIG,
            C.MPU6050_ACONFIG_AFS_SEL_BIT,
            C.MPU6050_ACONFIG_AFS_SEL_LENGTH,
            fsr,
        )

    def read_sensors(self):
        self.i2c.readfrom_mem_into(
            self.address, C.MPU6050_RA_ACCEL_XOUT_H, self.sensors
        )

        data = unpack(">hhhhhhh", self.sensors)
        return SensorReadings(*data)

    def read_sensors_scaled(self):
        data = list(self.read_sensors())
        data[0:3] = [x / (65536 // self.accel_range // 2) for x in data[0:3]]
        data[4:7] = [x / (65536 // self.gyro_range // 2) for x in data[4:7]]
        return SensorReadings(*data)

    def set_dhpf_mode(self, bandwidth):
        self.set_bitfield(
            C.MPU6050_RA_ACCEL_CONFIG,
            C.MPU6050_ACONFIG_ACCEL_HPF_BIT,
            C.MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
            bandwidth,
        )

    # 0 Reset
    # 1 On @ 5 Hz
    # 2 On @ 2.5 Hz
    # 3 On @ 1.25 Hz
    # 4 On @ 0.63 Hz
    # 7 Hold
    def get_dhpf_mode(self):
        return self.read_byte(C.MPU6050_RA_ACCEL_CONFIG)

    def set_motion_detection_threshold(self, threshold):
        self.write_byte(C.MPU6050_RA_MOT_THR, threshold)

    def set_motion_detection_duration(self, duration):
        self.write_byte(C.MPU6050_RA_MOT_DUR, duration)

    def enable_motion_interrupt(self, enabled):
        self.set_bitfield(C.MPU6050_RA_INT_ENABLE, C.MPU6050_INTERRUPT_MOT_BIT, 1, 1)

    def disable_motion_interrupt(self, enabled):
        self.set_bitfield(C.MPU6050_RA_INT_ENABLE, C.MPU6050_INTERRUPT_MOT_BIT, 1, 0)

    def get_sensor_avg(self, samples, softstart=100):
        """Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle."""
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)  # type: ignore[attr-defined]

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return SensorReadings(*[x // samples for x in counters])
