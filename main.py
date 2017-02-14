import time
import mpu6050
import mpuserver
import micropython
from constants import *
micropython.alloc_emergency_exception_buf(100)

def isr(pin):
    print("Interrupt!")

mpu = mpu6050.MPU()
server = mpuserver.MPUServer(mpu)
server.serve()
