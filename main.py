import time
import mpu6050
import mpuserver

def showpos():
    last = time.ticks_ms()
    while True:
        p = mpu.read_position()
        if time.ticks_diff(time.ticks_ms(), last) >= 100:
            last = time.ticks_ms()
            print(p)

        time.sleep_ms(1)


mpu = mpu6050.MPU()
server = mpuserver.MPUServer(mpu)
server.serve()

