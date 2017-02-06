import mpu6050

#mpu = mpu6050.MPU6050()
try:
    import mpuserver
    mpuserver.serve()
except Exception as exc:
    print('mpuserver failed: {}'.format(exc))
