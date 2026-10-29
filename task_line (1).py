# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 23:46:16 2026

@author: dejam
"""

"""taskline"""
from time import ticks_ms, ticks_diff, ticks_us
from math import pi

S1_INIT = 1
S2_IDLE = 2
S3_FOLLOW = 3
S4_FINISHED = 4
S5_LEAVE = 5
S6_RETURN = 6
S7_NEWLINE = 7

DONE_SUCCEEDED = 1
DONE_FAILED = 2    

MAX_SP = 50    # Same limit as the user interface [rad/s]
NAN = float("nan")    # Marks an undefined centroid in the log

# CHECK ALL OF THESE NUMBERS
WHEEL_R = 0.035    # Wheel radius [m] 
D_LANE = 0.200    # Neighboring line spacing [m]
SIGMA = 1    # +1 = left lane change, -1 = right
W0 = 0.5    # Yaw-rate request [rad/s]  TUNE
KP_W = 2.0    # Yaw-rate gain [(rad/s)/(rad/s)]  TUNE
TRIGGER_MS = 2000    # Line following time before the lane change  TUNE
TIMEOUT_MS = 5000    # Max time leaving + turning back  TUNE
WHITE_TOTAL = 0.3    # Total weight below this = white background  TUNE from readings
LINE_TOTAL = 1.5    # Total weight above this = a line is under the array  TUNE from readings

class TaskLine:
    def __init__(self, sensor, go, done, base, kp, left_sp, right_sp,
                 left_go, right_go, time_q, cen_q, imu, omega_L, omega_R):
        self.sensor = sensor
        self.go = go    # Share user -> line (1 = follow, 0 = stop)
        self.done = done    # Share line -> user (1 = data ready)
        self.base = base    # Share user -> line, base speed [rad/s]
        self.kp = kp    # Share user -> line, line gain [(rad/s)/mm]
        self.left_sp = left_sp    # Shares line -> motor tasks
        self.right_sp = right_sp
        self.left_go = left_go    # Motor go shares, used to stop the motors
        self.right_go = right_go
        self.time_q = time_q    # Queues line -> user
        self.cen_q = cen_q    # Centroid [mm], nan if undefined

        self.state = S1_INIT
        self.start = 0
        self.base_val = 0
        self.kp_val = 0
        
        self.imu = imu
        self.omega_L = omega_L    # Shares motor -> line, measured wheel speed [rad/s]
        self.omega_R = omega_R
        self.h0 = 0    # IMU heading after leaving [rad]
        self.d_hat = 0    # Lateral displacement estimate [m]
        self.last_us = 0
        self.man_start = 0
        self.seen_white = False


    def _clamp(self, value):
        return max(-MAX_SP, min(MAX_SP, value))
    
    def _stop(self, result):
        self.left_sp.put(0)
        self.right_sp.put(0)
        self.left_go.put(0)
        self.right_go.put(0)
        self.done.put(result)
        self.state = S4_FINISHED

    def _yaw_steer(self, w_sp, rate):
        d_omega = KP_W * (w_sp - rate)    # Positive correction turns left
        self.left_sp.put(self._clamp(self.base_val - d_omega))
        self.right_sp.put(self._clamp(self.base_val + d_omega))

    def run(self):
        """Generator: each next() runs ONE pass of the state machine."""
        while True:
            if self.state == S1_INIT:
                self.go.put(0)
                self.done.put(0)
                self.state = S2_IDLE

            elif self.state == S2_IDLE:
                if self.go.get():
                    self.base_val = self.base.get()
                    self.kp_val = self.kp.get()
                    self.time_q.clear()
                    self.cen_q.clear()
                    self.start = ticks_ms()
                    self.state = S3_FOLLOW

            elif self.state == S3_FOLLOW:
                if not self.go.get():    # User stopped, stop the wheels
                    self.left_sp.put(0)
                    self.right_sp.put(0)
                    self.state = S2_IDLE
                elif self.time_q.full():    # Dataset finished
                    self.left_sp.put(0)
                    self.right_sp.put(0)
                    self.left_go.put(0)
                    self.right_go.put(0)
                    self.done.put(DONE_SUCCEEDED)
                    self.state = S4_FINISHED
                elif ticks_diff(ticks_ms(), self.start) >= TRIGGER_MS:    # Start lleaving lane
                    rate, self.h0 = self.imu.read()
                    self.d_hat = 0
                    self.last_us = ticks_us()
                    self.man_start = ticks_ms()
                    self.state = S5_LEAVE
                
                else:
                    t_ms = ticks_diff(ticks_ms(), self.start)
                    centroid = self.sensor.read()    # mm, + = line on left
                    if centroid is None:    # Line lost, keep last setpoints
                        self.cen_q.put(NAN)
                    else:
                        corr = self.kp_val * centroid
                        self.left_sp.put(self._clamp(self.base_val - corr))    # Swap - and + if turns backwards
                        self.right_sp.put(self._clamp(self.base_val + corr))
                        self.cen_q.put(centroid)
                    self.time_q.put(t_ms)
                    
            elif self.state == S5_LEAVE:
                if not self.go.get():    # Stop the wheels
                    self.left_sp.put(0)
                    self.right_sp.put(0)
                    self.state = S2_IDLE
                elif ticks_diff(ticks_ms(), self.man_start) >= TIMEOUT_MS:
                    self._stop(DONE_FAILED)
                else:
                    rate, heading = self.imu.read()
                    self._yaw_steer(SIGMA * W0, rate)
                    now = ticks_us()
                    dt = ticks_diff(now, self.last_us) / 1_000_000    # Actual elapsed time [s]
                    self.last_us = now
                    psi = (heading - self.h0 + pi) % (2 * pi) - pi
                    v_hat = WHEEL_R / 2 * (self.omega_L.get() + self.omega_R.get())
                    self.d_hat += v_hat * psi * dt
                    if SIGMA * self.d_hat >= D_LANE / 2:
                        self.seen_white = False
                        self.state = S6_RETURN

            elif self.state == S6_RETURN:
                if not self.go.get():
                    self.left_sp.put(0)
                    self.right_sp.put(0)
                    self.state = S2_IDLE
                elif ticks_diff(ticks_ms(), self.man_start) >= TIMEOUT_MS:
                    self._stop(DONE_FAILED)
                else:
                    rate, heading = self.imu.read()
                    self._yaw_steer(-SIGMA * W0, rate)
                    self.sensor.read_raw()
                    self.sensor.weights()
                    if self.sensor.total < WHITE_TOTAL:
                        self.seen_white = True
                    elif self.seen_white and self.sensor.total >= LINE_TOTAL:
                        self.state = S7_NEWLINE

            elif self.state == S7_NEWLINE:
                if not self.go.get():
                    self.left_sp.put(0)
                    self.right_sp.put(0)
                    self.state = S2_IDLE
                elif self.time_q.full():    # Backstop
                    self._stop(DONE_SUCCEEDED)
                else:
                    t_ms = ticks_diff(ticks_ms(), self.start)
                    self.sensor.read_raw()
                    self.sensor.weights()
                    if self.sensor.completely_black():    # Stop bar before the centroid
                        self._stop(DONE_SUCCEEDED)
                    else:
                        centroid = self.sensor.centroid()
                        if centroid is None:
                            self.cen_q.put(NAN)
                        else:
                            corr = self.kp_val * centroid
                            self.left_sp.put(self._clamp(self.base_val - corr))
                            self.right_sp.put(self._clamp(self.base_val + corr))
                            self.cen_q.put(centroid)
                        self.time_q.put(t_ms)
           
            elif self.state == S4_FINISHED:
                if not self.go.get():    # Wait until the user has printed the data
                    self.done.put(0)
                    self.state = S2_IDLE

            else:
                raise ValueError('Invalid state')

            yield self.state