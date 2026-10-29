# -*- coding: utf-8 -*-
"""
Created on Thu Oct  8 01:26:19 2026

@author: dejam
"""
"""taskMotor"""
from time import ticks_ms, ticks_diff
from math import pi
from ClosedLoop import volts_to_duty

S1_INIT = 1
S2_IDLE = 2
S3_COLLECT = 3
S4_FINISHED = 4

DONE_SUCCEEDED = 1    # Full trial ran successfully
DONE_FAILED = 2    # Trial failed


class TaskMotor:
    def __init__(self, motor, encoder, go, done, time_q, pos_q, vel_q, volt_q,
                 setpoint, kp, ki, controller, V_SUPPLY, COUNTS_PER_REV,
                 TRIAL_DURATION_MS=1000, motor_dir=1, enc_sign=1, follow=None, omega_out=None):
        self.motor = motor
        self.encoder = encoder
        self.go = go    # Share user -> motor (1 = run, 0 = stop)
        self.done = done    # Share motor -> user (1 = data ready)

        self.time_q = time_q    # Queues motor -> user
        self.pos_q = pos_q    # Wheel position [rad]
        self.vel_q = vel_q    # Wheel velocity [rad/s]
        self.volt_q = volt_q    # Voltage command [V]
        self.setpoint = setpoint    # Shares user -> motor
        self.kp = kp
        self.ki = ki
        self.follow = follow    # Share, 1 = line following (same for both motors)
        self.following = 0    # Copy of follow, read once at the start of a run

        self.contr = controller
        self.V_SUPPLY = V_SUPPLY    # Measured from battery
        self.TRIAL_DURATION_MS = TRIAL_DURATION_MS

        self.motor_dir = motor_dir
        self.enc_sign = enc_sign
        self.counts_to_rad = enc_sign * 2 * pi / COUNTS_PER_REV

        self.state = S1_INIT
        self.trial_start = 0
        self.sp = 0
        self.kp_val = 0
        self.ki_val = 0
        
        self.omega_out = omega_out    # Share motor to line; measured wheel speed [rad/s]

    def run(self):
        """Generator: each next() runs ONE pass of the state machine."""
        while True:
            if self.state == S1_INIT:
                self.motor.disable()
                if self.omega_out is not None:    # Stop Sharing when motors are disabled
                    self.omega_out.put(0)
                    self.go.put(0)
                    self.done.put(0)
                    self.state = S2_IDLE

            elif self.state == S2_IDLE:
                if self.go.get():    # Wait without blocking for the user
                    self.sp = self.setpoint.get()
                    self.kp_val = self.kp.get()
                    self.ki_val = self.ki.get()
                    self.following = self.follow.get() if self.follow is not None else 0
                    self.contr.reset()
                    self.encoder.zero()
                    self.time_q.clear()
                    self.pos_q.clear()
                    self.vel_q.clear()
                    self.volt_q.clear()
                    self.motor.enable()
                    self.trial_start = ticks_ms()
                    self.state = S3_COLLECT

            elif self.state == S3_COLLECT:
                t_ms = ticks_diff(ticks_ms(), self.trial_start)
                if not self.go.get():    # User aborted, disable motors
                    self.motor.disable()
                    if self.omega_out is not None:
                        self.omega_out.put(0)
                    self.state = S2_IDLE
                elif not self.following and t_ms >= self.TRIAL_DURATION_MS:
                    self.motor.disable()
                    if self.omega_out is not None:
                        self.omega_out.put(0)
                    self.done.put(DONE_SUCCEEDED)
                    self.state = S4_FINISHED
                elif not self.following and self.time_q.full():    # No more space
                    self.motor.disable()
                    if self.omega_out is not None:
                        self.omega_out.put(0)
                    self.done.put(DONE_FAILED)
                    self.state = S4_FINISHED
                else:
                    if self.following:
                        self.sp = self.setpoint.get()    # Line task updates this live
                    self.encoder.update()
                    dt = self.encoder.dt / 1_000_000    # Change in time [s]
                    omega = self.encoder.get_velocity() * self.counts_to_rad    # Velocity [rad/s]
                    if self.omega_out is not None:    # Copies measured wheels speed into Share
                        self.omega_out.put(omega)
                    theta = self.encoder.get_position() * self.counts_to_rad    # Position [rad]
                    volts = self.contr.update(self.sp, omega, self.kp_val,
                                              self.ki_val, dt, -self.V_SUPPLY, self.V_SUPPLY)
                    duty = self.motor_dir * volts_to_duty(volts, self.V_SUPPLY)    # Volts to duty cycle
                    self.motor.set_effort(duty)
                    if not self.following:    # Only log step tests
                        self.time_q.put(t_ms)
                        self.pos_q.put(theta)
                        self.vel_q.put(omega)
                        self.volt_q.put(volts)

            elif self.state == S4_FINISHED:
                if not self.go.get():    # Wait until the user prints the data
                    self.done.put(0)
                    self.state = S2_IDLE

            else:
                raise ValueError('Invalid state')

            yield self.state