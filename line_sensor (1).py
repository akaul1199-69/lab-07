# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 22:16:49 2026

@author: dejam
"""

""" (Lab 0x06).
Line sensor

Hardware objects (the Timer) are created by the caller and passed in. This module
does not touch the motors.

NEW WIRING TABLE in memo document
  Battery sensing is reserved for pin B1 (not used here).
"""
from array import array
from pyb import ADC, Pin

CHANNELS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13)           # printed channel numbers
PIN_NAMES = ("A0", "A1", "A4", "B0", "C1", "C0", "C2", "C3", "A6", "A7", "C5", "C4", "A5")
CH1_ON_RIGHT = True
SPACING_MM = 4.0

ADC_TIMER_ID = 7    # a free timer (2, 3, 4 are used by the motors and encoders)
TIMER_FREQ = 5000    # Hz. 13 channels x 5 kHz = 65,000 readings/s = tested maximum
N_SAMPLES = 10    # samples per channel per batch (10 samples at 5 kHz = about 2 ms)
MIN_SPAN = 200    # smallest acceptable black - white difference [ADC counts]
MIN_TOTAL = 0.1    # total weight below this = no line seen (centroid undefined).
                        # Noise on white leaves a tiny nonzero total, so test against a
                        # small threshold, not exactly zero.
COMPLETELY_BLACK = 0.8    # Any reading at or above this counts as Black

def lateral_positions(ch1_on_right=CH1_ON_RIGHT):
    """Lateral position of each channel in mm (-24 ... +24), + toward Romi's left."""
    half = SPACING_MM * (len(CHANNELS) - 1) / 2
    if ch1_on_right:
        return [-half + SPACING_MM * (ch - 1) for ch in CHANNELS]
    return [half - SPACING_MM * (ch - 1) for ch in CHANNELS]


def check_calibration(black, white, min_span=MIN_SPAN, n=len(CHANNELS)):
    """Return a list of problems (empty list = calibration is usable)."""
    problems = []
    if black is None or white is None or len(black) != n or len(white) != n:
        return ["black/white lists must each contain " + str(n) + " values"]
    for i in range(n):
        span = black[i] - white[i]
        if span < 0:
            problems.append("channel " + str(i + 1) + ": black is LOWER than white")
        elif span < min_span:
            problems.append("channel " + str(i + 1) + ": span " + str(int(span)) +
                            " is below min_span " + str(min_span))
    return problems


class LineSensor:
    def __init__(self, timer, pin_names=PIN_NAMES, positions_mm=None, n_samples=N_SAMPLES):
        self.n = len(pin_names)
        self.n_samples = n_samples
        self.timer = timer                                   # already running at TIMER_FREQ
        self.adcs = tuple([ADC(getattr(Pin.cpu, p)) for p in pin_names])
        # Allocated once, reused on every acquisition
        self.bufs = tuple([array('H', [0] * n_samples) for _ in range(self.n)])
        self.pos = list(positions_mm) if positions_mm else lateral_positions()
        self.raw = [0.0] * self.n        # channel averages, ADC counts (overwritten each read)
        self.w = [0.0] * self.n          # normalized weights 0..1 (overwritten each read)
        self.total = 0.0                 # sum of weights from the last read
        self.black = None
        self.white = None
        self.span = None

    # ---- raw readings ----
    def read_raw(self):
        """One short timed batch on all channels; returns the per-channel averages."""
        if not ADC.read_timed_multi(self.adcs, self.bufs, self.timer):
            raise RuntimeError("ADC acquisition overrun")
        for i in range(self.n):
            self.raw[i] = sum(self.bufs[i]) / self.n_samples
        return self.raw

    # ---- calibration ----
    def set_calibration(self, black, white, min_span=MIN_SPAN):
        problems = check_calibration(black, white, min_span, self.n)
        if problems:
            raise ValueError("; ".join(problems))
        self.black = list(black)
        self.white = list(white)
        self.span = [b - w for b, w in zip(self.black, self.white)]

    # ---- normalization and centroid ----
    def weights(self, raw=None):
        """Per-channel (raw - white) / (black - white), clipped to 0..1."""
        if raw is None:
            raw = self.raw
        total = 0.0
        for i in range(self.n):
            x = (raw[i] - self.white[i]) / self.span[i]
            if x < 0.0:
                x = 0.0
            elif x > 1.0:
                x = 1.0
            self.w[i] = x
            total += x
        self.total = total
        return self.w

    def centroid(self):
        """Weighted line position in mm (+ = left), or None if no weight (undefined)."""
        if self.total < MIN_TOTAL:
            return None
        s = 0.0
        for i in range(self.n):
            s += self.pos[i] * self.w[i]
        return s / self.total

    def read(self):
        """Acquire, normalize, and return the centroid in mm (or None). Weights in self.w,
        total weight in self.total."""
        self.read_raw()
        self.weights()
        return self.centroid()

    def line_lost(self, low, high):
        """Is true if total weight is outside [low, high]. 
        Thresholds chosen from the normal line profile."""
        return self.total < low or self.total > high
    
    def completely_black(self, threshold=COMPLETELY_BLACK):
        """ Is true if all sensor channels read at or above threshold"""
        for i in range(self.n):
            if self.w[i] < threshold:
                return False
            return True