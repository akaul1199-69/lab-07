"""imu_calibration.py - save/load the BNO055 22-byte calibration profile as JSON (separate from the QTR file).

{"mode": "IMUPLUS", "units": "deg, deg/s (16 LSB per unit)", "mounting": "...", "profile": [22 ints 0..255]}
All 22 bytes are saved even though IMUPLUS ignores the magnetometer entries.
"""
import json

CAL_FILE = "imu_calibration.json"
MODE_NAME = "IMUPLUS"
UNITS = "deg, deg/s (16 LSB per unit)"
MOUNTING = "flat on Romi top plate, z up, heading sign -1 (clockwise sensor -> left-positive Romi)"


def save_profile(profile, path=CAL_FILE):
    with open(path, "w") as f:
        json.dump({"mode": MODE_NAME, "units": UNITS, "mounting": MOUNTING, "profile": list(profile)}, f)


def load_profile(path=CAL_FILE):
    """Return the 22-int list, or None if missing, invalid, or saved for a different configuration."""
    try:
        with open(path) as f:
            data = json.load(f)
        p = data["profile"]
        if data["mode"] != MODE_NAME or data["units"] != UNITS:
            return None
        if len(p) != 22 or any((not isinstance(b, int)) or b < 0 or b > 255 for b in p):
            return None
        return p
    except (OSError, ValueError, KeyError, TypeError):
        return None
