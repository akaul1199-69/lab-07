# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 19:48:28 2026

@author: dejam
"""

"""bno055 - IMU driver, IMUPLUS mode.
Wiring (record actual): Red=VIN, Black=GND, Yellow=SDA, Blue=SCL, Green=RST. Pin 2 (3Vo) unconnected.
Mounting (record actual): board flat on Romi, z up, x forward.
"""
from micropython import const
from struct import calcsize, unpack_from
from time import sleep_ms
from math import pi

from micropython import const
from struct import calcsize, unpack_from
from time import sleep_ms
from math import pi


class BNO055:
    DEV_ADDR = const(0x28)
    CHIP_ID_VALUE = const(0xA0)
    MODE_CONFIG = const(0x00)
    MODE_IMUPLUS = const(0x08)
    UNIT_SEL_DEG = const(0x00)    # Degrees and deg/s
    PROFILE_LEN = const(22)
    TO_RAD = pi / 180 / 16    # 16 counts per degree (or deg/s), then to radians

    class reg:
        CHIP_ID = (const(0x00), b"<B")
        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")
        GYR_DATA_Z = (const(0x18), b"<h")
        GYR_Z_EUL_HEADING = (const(0x18), b"<hh")    # Gyro Z then heading, one read
        CALIB_STAT = (const(0x35), b"<B")
        UNIT_SEL = (const(0x3B), b"<B")
        OPR_MODE = (const(0x3D), b"<B")
        CALIB_PROFILE = (const(0x55), b"<22B")

    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22)))

    def _read_reg(self, reg):
        # Determine number of bytes to read
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)

    def _write_reg(self, reg, value):
        self._i2c.mem_write(bytes([value]), self.DEV_ADDR, reg[0])

    def chip_id(self):
        return self._read_reg(BNO055.reg.CHIP_ID)[0]

    def set_mode(self, mode):
        self._write_reg(BNO055.reg.OPR_MODE, mode)
        sleep_ms(25 if mode == BNO055.MODE_CONFIG else 10)    # Datasheet: 19 ms / 7 ms

    def configure(self):
        self.set_mode(BNO055.MODE_CONFIG)
        self._write_reg(BNO055.reg.UNIT_SEL, BNO055.UNIT_SEL_DEG)
        self.set_mode(BNO055.MODE_IMUPLUS)

    def get_calibration(self):
        """Returns (system, gyro, accel, mag), each 0 (uncalibrated) to 3 (fully calibrated)"""
        stat = self._read_reg(BNO055.reg.CALIB_STAT)[0]
        return (stat >> 6) & 3, (stat >> 4) & 3, (stat >> 2) & 3, stat & 3

    def read(self):
        """Returns (yaw rate [rad/s], heading [rad]), both positive-left"""
        gyro_z, head = self._read_reg(BNO055.reg.GYR_Z_EUL_HEADING)
        return gyro_z * self.TO_RAD, -head * self.TO_RAD    # BNO heading is clockwise, so negate

    def read_profile(self):
        self.set_mode(BNO055.MODE_CONFIG)
        data = list(self._read_reg(BNO055.reg.CALIB_PROFILE))
        self.set_mode(BNO055.MODE_IMUPLUS)
        return data

    def write_profile(self, data):
        self.set_mode(BNO055.MODE_CONFIG)
        self._i2c.mem_write(bytes(data), self.DEV_ADDR, BNO055.reg.CALIB_PROFILE[0])
        self.set_mode(BNO055.MODE_IMUPLUS)