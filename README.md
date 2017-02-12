# py-mpu6050

This is [micropython][] code that runs on an [ESP8266][].  It reads
data from an [MPU6050][] 6DOF IMU and provides it to clients via a
simple TCP server.

[micropython]: https://micropython.org/
[esp8266]: https://en.wikipedia.org/wiki/ESP8266
[mpu6050]: https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/

The code uses a complementary filter to combine readings from the
gyroscope and accelerometer to provide stable and responsive position
information.

## References

- [MPU 6050 Register Map and Descriptions][1]
- [MPU 6050 Data Sheet][2]
- [Tilt Sensing Using a Three-Axis Accelerometer][3]
- [Quaternions & IMU Sensor Fusion with Complementary Filtering][4]

[1]: http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
[2]: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
[3]: http://www.nxp.com/assets/documents/data/en/application-notes/AN3461.pdf
[4]: https://stanford.edu/class/ee267/lectures/lecture10.pdf

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

