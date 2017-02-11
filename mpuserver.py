import os
import gc

from mpu6050 import MPU

import socket
import select
import time

mpu = MPU()

def serve(port=8000, interval=10):
    print('starting mpu server on port {}'.format(port))

    mpu.calibrate()

    server = socket.socket()
    server.bind(('0.0.0.0', port))
    server.listen(2)

    poll = select.poll()
    poll.register(server, select.POLLIN)
    clients = {}
    lastsample = 0
    while True:
        now = time.ticks_ms()
        delta = time.ticks_diff(now, lastsample)
        ready = poll.poll(max(0, interval-delta))

        if delta >= interval:
            values = mpu.read_angles()
            lastsample = time.ticks_ms()

            for c in clients.values():
                poll.register(c[0])

        for obj, eventmask in ready:
            if obj is server:
                if eventmask & select.POLLIN:
                    cl, addr = server.accept()
                    print('new connection from {}'.format(addr))
                    clients[id(cl)] = (cl, addr)
                    poll.register(cl, select.POLLOUT|select.POLLHUP)
                else:
                    print('connection says what?')

            elif eventmask & select.POLLHUP:
                client = clients[id(obj)]
                print('client {} has disconnected'.format(client[1]))
                obj.close()
                del clients[id(obj)]
                poll.unregister(obj)
                gc.collect()

            elif eventmask & select.POLLOUT:
                client = clients[id(obj)]

                try:
                    obj.write('[')
                    obj.write(', '.join('{:f}'.format(x) for x in values))
                    obj.write(']\n')
                except OSError:
                    print('lost connection from {}'.format(client[1]))
                    obj.close()
                    del clients[id(obj)]
                    gc.collect()

                poll.unregister(obj)
            else:
                print('client says what?')
