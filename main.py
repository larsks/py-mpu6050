import mpu6050
import mpuserver

mpu = mpu6050.MPU()
server = mpuserver.MPUServer(mpu)
server.serve()
