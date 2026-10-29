""""main"""
import cotask
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, USB_VCP, I2C
from task_share import Share, Queue
from motor_AK import Motor
from encoder import Encoder
from taskmotor_0x07 import TaskMotor
from taskuser_0x07 import TaskUser, S6_EXIT
from task_line import TaskLine
from ClosedLoop import ClosedLoop
from line_sensor import N_SAMPLES, TIMER_FREQ
from line_startup import get_line_sensor
from imu_startup import get_imu
from machine import SoftI2C, Pin as MPin

Pin(Pin.cpu.C8, Pin.OUT_PP).low()    # Left nSLP
Pin(Pin.cpu.B9, Pin.OUT_PP).low()    # Right nSLP

line_sensor = get_line_sensor()    # Calibrate before the scheduler starts

# ADD WIRE PLACEMENT
MPin("C10", MPin.OUT, value=1)     # IMU reset (green wire) held high
i2c = SoftI2C(scl=MPin("B13"), sda=MPin("B14"), freq=400_000, timeout=20000)
imu = get_imu(i2c)

class Profiled:
    def __init__(self, name, gen, priority, period_ms):
        self.name, self.gen = name, gen
        self.priority, self.period_ms = priority, period_ms
        self.runs = 0
        self.run_sum = 0
        self.run_max = 0
        self.gap_sum = 0
        self.gap_max = 0
        self.last = None

    def run(self):
        while True:
            t0 = ticks_us()
            if self.last is not None:
                gap = ticks_diff(t0, self.last)
                self.gap_sum += gap
                if gap > self.gap_max:
                    self.gap_max = gap
            self.last = t0
            state = next(self.gen)
            dt = ticks_diff(ticks_us(), t0)
            self.runs += 1
            self.run_sum += dt
            if dt > self.run_max:
                self.run_max = dt
            yield state

    def row(self):
        n = max(self.runs, 1)
        m = max(self.runs - 1, 1)
        return "{:<9}{:>4}{:>9}{:>7}{:>11.0f}{:>10}{:>12.2f}{:>11.2f}".format(
            self.name, self.priority, self.period_ms, self.runs,
            self.run_sum / n, self.run_max,
            self.gap_sum / m / 1000, self.gap_max / 1000)


def print_profile(plist):
    print("Task     Pri  Per(ms)   Runs  AveRun(us)  MaxRun(us)  AveGap(ms)  MaxGap(ms)")
    for p in plist:
        print(p.row())


pwm_tim = Timer(2, freq=20_000)    # Motor PWM timer
motor_L = Motor(Pin.cpu.B10, Pin.cpu.C9, Pin.cpu.C8, pwm_tim, 3)
motor_R = Motor(Pin.cpu.B3, Pin.cpu.C6, Pin.cpu.B9, pwm_tim, 2)

enc_tim_L = Timer(3, period=0xFFFF, prescaler=0)
encoder_L = Encoder(enc_tim_L, Pin.cpu.B4, Pin.cpu.B5)
enc_tim_R = Timer(4, period=0xFFFF, prescaler=0)
encoder_R = Encoder(enc_tim_R, Pin.cpu.B6, Pin.cpu.B7)

ser = USB_VCP()

V_SUPPLY = 8.06    # Measured battery voltage on 10/22
COUNTS_PER_REV = 1440    # Encoder counts per revolution per wheel
SAMPLE_INTERVAL_MS = 10    # Motor task period
USER_INTERVAL_MS = 10    # User task period
LINE_INTERVAL_MS = 20    # Line task period
MOTOR_PRIORITY = 2    # Highest priority runs first
LINE_PRIORITY = 1    # Between motors and user
USER_PRIORITY = 0    # Lowest priority
TRIAL_DURATION_MS = 4000    # Step test length
LINE_DURATION_MS = 15000    # Line following dataset length
N = TRIAL_DURATION_MS // SAMPLE_INTERVAL_MS + 5    # Step test queue size
N_LINE = LINE_DURATION_MS // LINE_INTERVAL_MS + 5    # Line queue size

MOTOR_DIR_L = 1
ENC_SIGN_L = 1
MOTOR_DIR_R = 1
ENC_SIGN_R = -1

def make_shares(tag):
    return (Share("B", name=tag + "_go"),
            Share("B", name=tag + "_done"),
            Queue("L", N, name=tag + "_t"),
            Queue("f", N, name=tag + "_pos"),
            Queue("f", N, name=tag + "_vel"),
            Queue("f", N, name=tag + "_volt"),
            Share("f", name=tag + "_setpoint"),
            Share("f", name=tag + "_kp"),
            Share("f", name=tag + "_ki"))


left, right = make_shares("L"), make_shares("R")
follow_mode = Share("B", name="follow")    # 0 = step test, 1 = line following
line_go = Share("B", name="line_go")
line_done = Share("B", name="line_done")
line_base = Share("f", name="line_base")
line_kp = Share("f", name="line_kp")
line_t = Queue("L", N_LINE, name="line_t")
line_cen = Queue("f", N_LINE, name="line_cen")

omega_L = Share("f", name="omega_L")    # Measure wheel speed [rad/s]
omega_R = Share("f", name="omega_R")

task_left = TaskMotor(motor_L, encoder_L, *left, ClosedLoop(), V_SUPPLY,
                      COUNTS_PER_REV, TRIAL_DURATION_MS, MOTOR_DIR_L, ENC_SIGN_L,
                      follow=follow_mode, omega_out=omega_L)
task_right = TaskMotor(motor_R, encoder_R, *right, ClosedLoop(), V_SUPPLY,
                       COUNTS_PER_REV, TRIAL_DURATION_MS, MOTOR_DIR_R, ENC_SIGN_R,
                       follow=follow_mode, omega_out=omega_R)
task_line = TaskLine(line_sensor, line_go, line_done, line_base, line_kp,
                     left[6], right[6], left[0], right[0], line_t, line_cen, 
                     imu, omega_L, omega_R)
config = {"motor_ms": SAMPLE_INTERVAL_MS, "user_ms": USER_INTERVAL_MS,
          "duration_ms": TRIAL_DURATION_MS, "v_supply": V_SUPPLY,
          "line_ms": LINE_INTERVAL_MS, "line_duration_ms": LINE_DURATION_MS,
          "n_samples": N_SAMPLES, "timer_freq": TIMER_FREQ}
line = (line_go, line_done, line_base, line_kp, line_t, line_cen, follow_mode)
task_user = TaskUser(ser, left, right, config, line)

prof_L = Profiled("MOTOR L", task_left.run(), MOTOR_PRIORITY, SAMPLE_INTERVAL_MS)
prof_R = Profiled("MOTOR R", task_right.run(), MOTOR_PRIORITY, SAMPLE_INTERVAL_MS)
prof_Line = Profiled("LINE", task_line.run(), LINE_PRIORITY, LINE_INTERVAL_MS)
prof_U = Profiled("USER", task_user.run(), USER_PRIORITY, USER_INTERVAL_MS)
profiles = [prof_L, prof_R, prof_Line, prof_U]

for prof in profiles:
    cotask.task_list.append(
        cotask.Task(prof.run(), name=prof.name,
                    priority=prof.priority, period=prof.period_ms, profile=True))

try:
    while task_user.state != S6_EXIT:    # Run until the user exits
        cotask.task_list.pri_sched()
except KeyboardInterrupt:
    pass
finally:
    motor_L.disable()
    motor_R.disable()
    motor_L.set_effort(0)
    motor_R.set_effort(0)
    print("Motors stopped.")

    print_profile(profiles)
