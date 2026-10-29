"""thresholds_check.py - read the line sensor to pick WHITE_TOTAL / LINE_TOTAL (motors off, do NOT run main).
exec(open('thresholds_check.py').read())   Put the array over WHITE, then a LINE, then the STOP BAR; note the numbers. q = stop."""
import sys
from pyb import USB_VCP
from time import sleep_ms
from line_startup import get_line_sensor

s = get_line_sensor()
vcp = USB_VCP()
print("total weight: WHITE should be small, a LINE larger. Stop bar: lowest channel should be high. q = stop.")
while True:
    s.read()
    sys.stdout.write("\r total weight {:5.2f}   lowest channel weight {:4.2f}   \x1b[K".format(s.total, min(s.w)))
    if vcp.any() and vcp.read(1) in (b"q", b"Q"):
        print("\nStopped.")
        break
    sleep_ms(100)
