"""imu_startup.py: this has the IMU wiring constants and startup calibration (for Lab 0x07). Call get_imu() BEFORE the
scheduler starts, with the motors disabled. It blocks while it waits for you (allowed at startup only).

WIRING: so I2C is bit banged (SoftI2C) because every hardware I2C bus on this
Nucleo shares pins with our motors or line sensors:
   red    VIN  -> 3V3   CN7 inner column, row 8 (pin 16)
   black  GND  -> GND   CN7 inner column, row 10 (pin 20)
   yellow SDA  -> PB14  CN10 outer column, row 14 (pin 28)
   blue   SCL  -> PB13  CN10 outer column, row 15 (pin 30)
   green  RST  -> PC10  CN7 outer column, row 1 (pin 1)
   connector position 2 (3Vo) is left unconnected.
"""
from machine import SoftI2C, Pin
from time import sleep_ms
from bno055 import BNO055
from imu_calibration import save_profile, load_profile, CAL_FILE

SCL_PIN = "B13"
SDA_PIN = "B14"
RST_PIN = "C10"


def make_imu():
    i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000, timeout=20000)
    rst = Pin(RST_PIN, Pin.OUT, value=1)
    imu = BNO055(i2c, reset_pin=rst)
    imu.begin()
    return imu


def run_calibration(imu):
    """Gyro: keep still. Accelerometer: hold in six axis-aligned orientations (each ~3 s)."""
    input("Keep Romi STILL, then slowly hold it in 6 positions (flat, upside down, on each side, "
          "nose up, nose down), pausing about 3 s in each. Press Enter to start...")
    while True:
        s, g, a, m = imu.calibration_status()
        print("calibration  sys:{} gyro:{} accel:{}   (need gyro 3 and accel 3)".format(s, g, a))
        if g == 3 and a == 3:
            return
        sleep_ms(500)


def get_imu(path=CAL_FILE, force_calibrate=False):
    imu = make_imu()
    print("BNO055 found (chip id 0xA0), IMUPLUS mode.")
    if not force_calibrate:
        profile = load_profile(path)
        if profile is not None:
            if input("Saved IMU calibration found. Use it? (y/n): ").strip().lower().startswith("y"):
                imu.write_profile(profile)
                print("Restored. Status (sys, gyro, accel, mag):", imu.calibration_status())
                return imu
        else:
            print("No valid saved IMU calibration; calibrating now.")
    run_calibration(imu)
    save_profile(imu.read_profile(), path)
    print("IMU calibration saved to", path)
    return imu
