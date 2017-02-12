# py-mpu6050

This is [micropython][] code that runs on an [ESP8266][].  It reads
data from an [MPU6050][] 6DOF IMU and provides it to clients via a
simple TCP server.

[micropython]: https://micropython.org/
[esp8266]: https://en.wikipedia.org/wiki/ESP8266
[mpu6050]: https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/

## Requirements

You will need a device running Micropython.

In order to use the `Makefile`, you will need [mpy-cross][] and
[ampy][].

## Installing

Running `make install` will first byte-compile (most of) the source
files using [mpy-cross][], and will then use [ampy][] to install the
files onto your Micropython device.

This project includes a `boot.py` and `main.py` that will cause your
Micropython board to run the `mpuserver` automatically at boot.  If
you do not want to install these files, edit the `Makefile` and
comment out the `STATIC = ...` line.

If your device is not connected to `/dev/ttyUSB0`, you will need to
either edit the `Makefile` to change the value of the `PORT` setting,
or provide it on the command line:

    make PORT=/dev/someotherdevice install

[ampy]: https://github.com/adafruit/ampy
[mpy-cross]: https://github.com/micropython/micropython/tree/master/mpy-cross

## License

py-mpu6050 -- visualization for py-mpu6050  
Copyright (C) 2017 Lars Kellogg-Stedman <lars@oddbit.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

