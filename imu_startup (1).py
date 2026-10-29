# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 19:59:17 2026

@author: dejam
"""

"""IMU_ startup"""
from time import sleep_ms
from bno055 import BNO055
from imu_calibration import save_calibration, load_calibration, IMU_CAL_FILE

CAL_WAIT_MS = 3000    # Time allowed to calibrate


def display_cal_status(imu):
    """Get the four calibration values (0..3), 
    Any sensor data reporting 0 should be ignored,
    3 means 'fully calibrated'  """
    system, gyro, accel, mag = imu.get_calibration()

    # The data should be ignored until the system calibration is > 0
    print("\t", end="")
    if not system:
        print("! ", end="")

    # Display the individual values
    print("Sys:", system, " G:", gyro, " A:", accel, " M:", mag)
    return system, gyro, accel, mag


def wait_for_status(imu, timeout_ms=None):
    waited = 0
    while timeout_ms is None or waited < timeout_ms:
        system, gyro, accel, mag = display_cal_status(imu)
        if gyro == 3 and accel == 3:
            return True
        sleep_ms(200)
        waited += 200
    return False


def run_calibration(imu):
    input("Set the IMU down and don't move it for the gyro, then press Enter...")
    print("Now hold it still on each side: +X, -X, +Y, -Y, +Z, -Z")
    wait_for_status(imu)


def get_imu(i2c, path=IMU_CAL_FILE, force_calibrate=False):
    sleep_ms(700)    # BNO055 boot time
    imu = BNO055(i2c)
    if imu.chip_id() != BNO055.CHIP_ID_VALUE:
        raise RuntimeError("BNO055 not found")
    imu.configure()

    if not force_calibrate:
        data = load_calibration(path, BNO055.PROFILE_LEN)
        if data is not None:
            answer = input("Found saved IMU calibration...Use it? (y/n): ")
            if answer.strip().lower().startswith("y"):
                imu.write_profile(data["profile"])
                if wait_for_status(imu, CAL_WAIT_MS):
                    return imu
                print("Status 3 not reached...Recalibrating.")
        else:
            print("IMU calibration invalid... Recalibrating.")

    run_calibration(imu)
    save_calibration(imu.read_profile(), path)
    print("IMU calibration saved to", path)
    return imu