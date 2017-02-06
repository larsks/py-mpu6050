import os

from mpu6050 import MPU6050

import socket
import select

mpu = MPU6050()

def serve(port=8000, interval=500):
    server = socket.socket()
    server.bind(('0.0.0.0', port))
    server.listen(5)

    poll = select.poll()
    poll.register(server, select.POLLIN)
    clients = {}
    lastsample = 0
    while True:
        now = time.ticks_ms()
        delta = time.ticks_diff(now, lastsample)
        ready = poll.poll(max(0, interval-delta))

        if delta >= interval:
            print('taking sample')
            values = mpu.read_accel_deg()
            lastsample = time.ticks_ms()

            for c in clients.values():
                poll.register(c[1])

        for obj, eventmask in ready:
            if obj is server:
                if eventmask & select.POLLIN:
                    cl, addr = server.accept()
                    print('new connection from {}'.format(addr[0]))
                    clients[id(cl)] = (c, addr)
                    poll.register(cl, select.POLLOUT|select.POLLHUP)
                else:
                    print('connection says what?')

            elif eventmask & select.POLLHUP:
                obj.close()
                poll.unregister(obj)
                del clients[id(obj)]

            elif eventmask & select.POLLOUT:
                client = clients[id(obj)]

                try:
                    obj.write('''[%0.2f, %0.2f, %0.2f]\n''' % tuple(values))
                except OSError:
                    print('lost connection from {}'.format(client[0][0]))
                    del clients[id(obj)]
                    poll.unregister(obj)
                else:
                    poll.unregister(client[1])
            else:
                print('client says what?')
