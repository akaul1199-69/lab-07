# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 19:53:32 2026

@author: dejam
"""

""" IMU Calibration"""
import json

IMU_CAL_FILE = "imu_calibration.json"    # Separate from line_calibration.json
IMU_MODE = "IMUPLUS"    # Operating mode the profile was saved in
IMU_MOUNTING = "flat, z up, x forward"    # Mounting the profile was saved with (match your actual mounting)


def save_calibration(profile, path=IMU_CAL_FILE):
    data = {"mode": IMU_MODE,
            "mounting": IMU_MOUNTING,
            "profile": list(profile)}
    with open(path, "w") as f:
        json.dump(data, f)


def load_calibration(path=IMU_CAL_FILE, n=22):
    """Return the dict, or None if the file is missing or invalid."""
    try:
        with open(path) as f:
            data = json.load(f)
        if data["mode"] != IMU_MODE or data["mounting"] != IMU_MOUNTING:
            return None
        if len(data["profile"]) != n:
            return None
        for b in data["profile"]:
            if not isinstance(b, int) or b < 0 or b > 255:
                return None
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return None