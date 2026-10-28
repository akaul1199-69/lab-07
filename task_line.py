"""task_line.py - outer-loop task: line following (Lab 0x06) + IMU lane change (Lab 0x07).

Owns the line sensor AND the IMU driver (one owner each). Only this task writes the wheel setpoints,
so exactly one steering law is active at a time:
  S3_FOLLOW      line controller on the starting line          -> trigger: distance travelled
  S5_DEPART      IMU yaw-rate feedback, reference  +s*RATE0     -> lateral estimate reaches half a lane
  S6_TURNBACK    IMU yaw-rate feedback, reference  -s*RATE0     -> target line seen after a white gap
  S7_FOLLOW_NEW  line controller on the neighbouring line       -> all 13 channels black (stop bar)
  S4_FINISHED    both motors disabled
s = +1 for a LEFT lane change, -1 for RIGHT (Romi positive-left convention).
"""
from time import ticks_ms, ticks_us, ticks_diff
from math import sin, pi

S1_INIT = 1
S2_IDLE = 2
S3_FOLLOW = 3
S4_FINISHED = 4
S5_DEPART = 5
S6_TURNBACK = 6
S7_FOLLOW_NEW = 7

DONE_OK = 1
DONE_FAILED = 2          # maneuver timeout (target missed)

MAX_SP = 50              # wheel setpoint limit [rad/s]
NAN = float("nan")

# ---------------- maneuver configuration (fixed constants; tune on the track) ----------------
WHEEL_R = 0.035          # wheel radius [m]
LANE_D = 0.200           # distance between neighbouring line centres [m]
Y_SWITCH = LANE_D / 2    # depart until the lateral estimate reaches half a lane, then turn back
RATE0 = 0.15             # yaw-rate reference magnitude [rad/s]. Keep it gentle: a sharp turn arrives at the new line at a steep angle
K_YAW = 3.0              # yaw-rate error -> wheel-speed correction [(rad/s)/(rad/s)]
DELTA_MAX = 2.0          # limit on the wheel-speed correction [rad/s]
PSI_HOLD = 0.05          # turn-back stops turning once heading is within this of the departure line [rad]
TRIGGER_DIST = 0.30      # follow the first line this far (encoder distance) before departing [m]
MAN_TIMEOUT_MS = 12000   # depart + turn-back must finish within this, else stop
NEW_TIMEOUT_MS = 12000   # new-line following must reach the stop bar within this, else stop
WHITE_MAX_TOTAL = 0.2    # total weight below this = white background
WHITE_COUNT = 3          # consecutive white samples needed to say the original line is cleared
LINE_MIN_TOTAL = 0.6     # total weight above this (after clearing) = a line is under the array
ALL_BLACK_W = 0.6        # every channel's normalized weight must exceed this for the stop bar
LOST_LIMIT = 10          # passes without a line before the wheels stop while plain following


class TaskLine:
    def __init__(self, sensor, go, done, base, kp, left_sp, right_sp,
                 left_go, right_go, time_q, cen_q, imu=None, mode=None,
                 left_meas=None, right_meas=None):
        self.sensor = sensor
        self.imu = imu
        self.go = go            # Share user -> line (1 = run, 0 = stop)
        self.done = done        # Share line -> user (0 idle, 1 finished, 2 maneuver timeout)
        self.base = base        # Share user -> line, base wheel speed [rad/s]
        self.kp = kp            # Share user -> line, line gain [(rad/s)/mm]
        self.mode = mode        # Share user -> line: 0 plain follow, 1 lane change LEFT, 2 lane change RIGHT
        self.left_sp = left_sp  # Shares line -> motor tasks [rad/s]
        self.right_sp = right_sp
        self.left_go = left_go
        self.right_go = right_go
        self.lm = left_meas     # Shares motor -> line: measured wheel speed [rad/s], + = forward
        self.rm = right_meas
        self.time_q = time_q
        self.cen_q = cen_q

        self.state = S1_INIT
        self.base_val = 0.0
        self.kp_val = 0.0
        self.lane = False
        self.s = 1
        self.start = 0
        self.man_start = 0
        self.last_us = 0
        self.dist = 0.0         # forward distance travelled since the run began [m]
        self.y = 0.0            # lateral displacement estimate from the departure line [m]
        self.h0 = 0.0
        self.white_run = 0
        self.cleared = False
        self.lost = 0

    # ---------- helpers ----------
    def _clamp(self, v):
        return max(-MAX_SP, min(MAX_SP, v))

    def _stop_wheels(self):
        self.left_sp.put(0)
        self.right_sp.put(0)

    def _finish(self, status):
        self._stop_wheels()
        self.left_go.put(0)      # motors disabled by their tasks
        self.right_go.put(0)
        self.done.put(status)
        self.state = S4_FINISHED

    def _steer_line(self, centroid):
        """Lab 0x06 law: correction = kp * centroid (+ centroid = line on the left -> turn left)."""
        if centroid is None:
            self.lost += 1
            if self.lost >= LOST_LIMIT:
                self._stop_wheels()
            return
        self.lost = 0
        corr = self.kp_val * centroid
        self.left_sp.put(self._clamp(self.base_val - corr))
        self.right_sp.put(self._clamp(self.base_val + corr))

    def _steer_yaw(self, ref, rate):
        """IMU yaw-rate feedback: delta = K_YAW * (ref - measured); positive delta turns left."""
        d = K_YAW * (ref - rate)
        d = max(-DELTA_MAX, min(DELTA_MAX, d))
        self.left_sp.put(self._clamp(self.base_val - d))
        self.right_sp.put(self._clamp(self.base_val + d))

    # ---------- state machine ----------
    def run(self):
        while True:
            st = self.state
            if st == S1_INIT:
                self.go.put(0)
                self.done.put(0)
                self.state = S2_IDLE

            elif st == S2_IDLE:
                if self.go.get():
                    self.base_val = self.base.get()
                    self.kp_val = self.kp.get()
                    m = self.mode.get() if self.mode is not None else 0
                    self.lane = (m != 0 and self.imu is not None and self.lm is not None)
                    self.s = 1 if m == 1 else -1
                    self.left_sp.put(self._clamp(self.base_val))     # start straight at base speed
                    self.right_sp.put(self._clamp(self.base_val))
                    self.time_q.clear()
                    self.cen_q.clear()
                    self.dist = 0.0
                    self.y = 0.0
                    self.lost = 0
                    self.white_run = 0
                    self.cleared = False
                    self.start = ticks_ms()
                    self.last_us = ticks_us()
                    self.state = S3_FOLLOW

            elif st == S4_FINISHED:
                if not self.go.get():            # wait until the user has printed the data
                    self.done.put(0)
                    self.state = S2_IDLE

            else:
                self._active()

            yield self.state

    def _active(self):
        if not self.go.get():                    # user stop
            self._stop_wheels()
            self.state = S2_IDLE
            return
        if self.time_q.full():                   # dataset full
            self._finish(DONE_OK)
            return

        now = ticks_us()
        dt = ticks_diff(now, self.last_us) / 1.0e6
        self.last_us = now
        v = 0.0                                  # forward speed [m/s] from the wheel encoders
        if self.lm is not None:
            v = WHEEL_R * (self.lm.get() + self.rm.get()) / 2
        st = self.state
        c = None

        if st == S3_FOLLOW:
            c = self.sensor.read()
            self._steer_line(c)
            self.dist += v * dt
            if self.lane and self.dist >= TRIGGER_DIST:        # automatic trigger: begin the lane change
                self.h0, _ = self.imu.read()                   # heading while aligned with the line
                self.y = 0.0
                self.white_run = 0
                self.cleared = False
                self.man_start = ticks_ms()
                self.state = S5_DEPART

        elif st == S5_DEPART or st == S6_TURNBACK:
            h, rate = self.imu.read()
            psi = (h - self.h0 + pi) % (2 * pi) - pi           # relative yaw, wrapped to -pi..pi
            self.y += v * sin(psi) * dt                        # lateral displacement along the fixed reference
            c = self.sensor.read()
            if self.sensor.total < WHITE_MAX_TOTAL:            # count white background after leaving the line
                self.white_run += 1
                if self.white_run >= WHITE_COUNT:
                    self.cleared = True
            else:
                self.white_run = 0
            if st == S5_DEPART:
                ref = self.s * RATE0
                if self.s * self.y >= Y_SWITCH:
                    self.state = S6_TURNBACK
            else:
                ref = -self.s * RATE0 if self.s * psi > PSI_HOLD else 0.0
                if self.cleared and c is not None and self.sensor.total >= LINE_MIN_TOTAL:
                    self.lost = 0                              # target line found: hand over to the line controller
                    self.state = S7_FOLLOW_NEW
                    self.man_start = ticks_ms()
            self._steer_yaw(ref, rate)
            if ticks_diff(ticks_ms(), self.man_start) > MAN_TIMEOUT_MS:
                self._finish(DONE_FAILED)                      # missed the target line
                return

        elif st == S7_FOLLOW_NEW:
            self.sensor.read_raw()
            self.sensor.weights()
            if min(self.sensor.w) >= ALL_BLACK_W:              # all 13 channels black = stop bar (check BEFORE centroid)
                self._finish(DONE_OK)
                return
            c = self.sensor.centroid()
            self._steer_line(c)
            if ticks_diff(ticks_ms(), self.man_start) > NEW_TIMEOUT_MS:
                self._finish(DONE_FAILED)
                return

        self.time_q.put(ticks_diff(ticks_ms(), self.start))
        self.cen_q.put(NAN if c is None else c)
