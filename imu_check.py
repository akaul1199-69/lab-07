"""imu_check.py: Okay, so this is the Step 3 verification. Motors disabled, do NOT run main. In PuTTY:
    exec(open('imu_check.py').read())
   """
import sys
from pyb import USB_VCP
from time import sleep_ms
from math import pi
from imu_startup import get_imu

imu = get_imu()
vcp = USB_VCP()
h0, _ = imu.read()
print("Turn Romi LEFT by hand: relative yaw should go positive. q to stop.")
while True:
    h, rate = imu.read()
    psi = (h - h0 + pi) % (2 * pi) - pi
    sys.stdout.write("\rheading {:6.1f} deg   relative yaw {:+7.1f} deg   yaw rate {:+6.1f} deg/s   cal {}   \x1b[K".format(
        h * 180 / pi, psi * 180 / pi, rate * 180 / pi, imu.calibration_status()))
    if vcp.any() and vcp.read(1) in (b"q", b"Q"):
        print("\nStopped.")
        break
    sleep_ms(100)
