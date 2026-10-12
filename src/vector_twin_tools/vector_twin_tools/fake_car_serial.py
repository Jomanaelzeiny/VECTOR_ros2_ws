#!/usr/bin/env python3
"""
Fake F1R3 car - emulates the Arduino Nano RP2040 low-level-control firmware
(see "Vector Low Level Control User Guide").

It creates a VIRTUAL SERIAL PORT and behaves like the real car's USB serial:
  - telemetry line every 200 ms (TL:R:<ms> changes it):   DC,Servo,Speed,Yaw\n
        DC    : motor throttle        0..100 (%)
        Servo : servo angle           65..135 (100 = straight)
        Speed : linear speed          mm/s
        Yaw   : heading from the IMU  deg (0 = heading at calibration, NOT wrapped)   # <<< CHANGED
  - first seconds after start (and after IMU:CAL): -1,-1,-1,-1  (IMU calibrating)
  - accepts the firmware commands:
        DC:F:<0-255>  DC:R:<0-255>  DC:S  SER:<65-135>  TL:R:<ms>  IMU:CAL  SCAL

Vehicle behaviour MEASURED on the real car (2026-09-28, wheels lifted):              # <<< CHANGED
  servo > 100 steers RIGHT; motor only buzzes below ~40 % power;                    # <<< CHANGED
  150 -> 0.29 m/s, 170 -> 0.39, 185 -> 0.455, 200 -> 0.495, 230 -> 0.73, 255 -> 0.905  # <<< CHANGED
  (on the floor the real car is a bit slower - re-measure and update SPEED_TABLE)    # <<< CHANGED

Until the first movement command arrives it drives a built-in scenario:
  oval (default) : closed oval that fits inside the 3.80 x 4.75 m city
  test           : original test (straight, left, right, stop, reverse)
The virtual port is linked at /tmp/f1r3_fake_serial  (real car: /dev/ttyACM0).

Usage:  ros2 run vector_twin_tools fake_car_serial [--scenario oval|test] [--noise] [--quiet]
"""
import argparse
import math
import os
import pty
import random
import select
import sys
import time
import tty

LINK_PATH = '/tmp/f1r3_fake_serial'

# ---------------- Firmware facts (from the Firmware Guide) ----------------
BAUD = 115200                  # a virtual port ignores baud; kept for documentation
DEFAULT_TL_RATE_MS = 200       # telemetry period
SERVO_MIN, SERVO_NEUTRAL, SERVO_MAX = 65, 100, 135
PWM_MAX = 255                  # DC:F / DC:R value range 0..255
CAL_SECONDS = 3.0              # "-1,-1,-1,-1" while the IMU calibrates

# ---------------- Vehicle model ----------------
# Measured PWM -> steady speed (m/s). Below ~105 the motor only buzzes (speed 0).    # <<< CHANGED
SPEED_TABLE = [(0, 0.0), (105, 0.0), (150, 0.29), (170, 0.39), (185, 0.455),          # <<< CHANGED
               (200, 0.495), (230, 0.73), (255, 0.905)]                               # <<< CHANGED
SPEED_TAU_S = 0.3              # how fast speed follows throttle (from the coast-down)  # <<< CHANGED
SERVO_UP_IS_LEFT = False       # MEASURED: servo > 100 steers RIGHT                     # <<< CHANGED
YAW_CCW_POSITIVE = True        # yaw grows when turning LEFT (confirm with the IMU)
YAW_WRAP_180 = False           # MEASURED: the firmware does NOT wrap yaw (-845 seen)   # <<< CHANGED
REVERSE_SPEED_NEGATIVE = True  # Speed is negative when reversing (confirm)
# Still ASSUMED - measure on the real car:
WHEELBASE_M = 0.25             # front-to-rear axle distance
STEER_RATIO = 0.6              # front-wheel angle / servo angle

# ---------------- Oval scenario settings (fits inside the city) ----------------
OVAL_PWM = 160                 # above the buzz zone -> ~0.34 m/s                       # <<< CHANGED (was 85)
OVAL_SERVO_OFFSET = 25         # 25 deg servo -> 15 deg wheels -> radius ~0.93 m        # <<< CHANGED
OVAL_STRAIGHT_M = 1.5          # length of each straight
OVAL_STOP_S = 2.0              # pause at the start point every lap


def speed_from_pwm(pwm):                                                              # <<< CHANGED
    """Steady-state speed (m/s, signed) for a signed PWM, from the measured table."""  # <<< CHANGED
    p = min(abs(pwm), PWM_MAX)                                                        # <<< CHANGED
    for (p0, v0), (p1, v1) in zip(SPEED_TABLE, SPEED_TABLE[1:]):                      # <<< CHANGED
        if p <= p1:                                                                   # <<< CHANGED
            v = v0 + (v1 - v0) * (p - p0) / (p1 - p0)                                 # <<< CHANGED
            return math.copysign(v, pwm) if pwm != 0 else 0.0                         # <<< CHANGED
    return math.copysign(SPEED_TABLE[-1][1], pwm)                                     # <<< CHANGED


def build_oval():
    """Closed oval, timing computed from the vehicle model so the loop closes:
    half straight -> left U-turn -> straight -> left U-turn -> half straight -> stop"""
    v = speed_from_pwm(OVAL_PWM)                                                      # <<< CHANGED
    wheel = math.radians(OVAL_SERVO_OFFSET * STEER_RATIO)                             # <<< CHANGED
    radius = WHEELBASE_M / math.tan(wheel)
    t_turn = math.pi * radius / v            # half circle
    t_straight = OVAL_STRAIGHT_M / v
    left = SERVO_NEUTRAL + OVAL_SERVO_OFFSET if SERVO_UP_IS_LEFT else SERVO_NEUTRAL - OVAL_SERVO_OFFSET  # <<< CHANGED
    return [
        (t_straight / 2, OVAL_PWM, SERVO_NEUTRAL),
        (t_turn,         OVAL_PWM, left),
        (t_straight,     OVAL_PWM, SERVO_NEUTRAL),
        (t_turn,         OVAL_PWM, left),
        (t_straight / 2, OVAL_PWM, SERVO_NEUTRAL),
        (OVAL_STOP_S,    0,        SERVO_NEUTRAL),
    ], v, radius


# Original test scenario: (duration_s, signed_pwm -255..255 (negative = reverse), servo_angle)
TEST_SCENARIO = [
    (2.0,    0, 100),   # stand still
    (3.0,  180, 100),   # straight                                                    # <<< CHANGED (160 -> 180)
    (3.0,  180, 125),   # turn (right on the real car)                                # <<< CHANGED
    (2.0,  180, 100),   # straight                                                    # <<< CHANGED
    (3.0,  180,  75),   # turn (left on the real car)                                 # <<< CHANGED
    (2.0,    0, 100),   # stop
    (2.0, -160, 100),   # reverse                                                     # <<< CHANGED (-120 -> -160)
    (2.0,    0, 100),   # stop
]


def log(msg):
    print(f'[fake_car] {msg}', flush=True)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FakeCar:
    def __init__(self, noise, scenario):
        self.noise = noise
        self.scenario = scenario
        self.pwm = 0                  # signed -255..255
        self.servo = SERVO_NEUTRAL
        self.speed = 0.0              # m/s, signed
        self.yaw = 0.0                # rad, + = left (CCW)
        self.tl_rate_ms = DEFAULT_TL_RATE_MS
        self.scenario_active = True
        self.scenario_t0 = None
        self.start_calibration()

    # ---------- firmware behaviour ----------
    def start_calibration(self):
        """Like the real firmware: car held still, yaw reset to 0."""
        self.cal_until = time.monotonic() + CAL_SECONDS
        self.pwm = 0
        self.speed = 0.0
        self.yaw = 0.0
        self.scenario_t0 = None

    @property
    def calibrating(self):
        return time.monotonic() < self.cal_until

    def stop_scenario(self):
        if self.scenario_active:
            self.scenario_active = False
            log('scenario stopped - now following serial commands')

    def handle_command(self, raw):
        cmd = raw.strip().upper()
        if not cmd:
            return
        try:
            if cmd.startswith('DC:F:'):
                self.pwm = clamp(int(cmd[5:]), 0, PWM_MAX)
                self.stop_scenario()
            elif cmd.startswith('DC:R:'):
                self.pwm = -clamp(int(cmd[5:]), 0, PWM_MAX)
                self.stop_scenario()
            elif cmd == 'DC:S':
                self.pwm = 0
                self.stop_scenario()
            elif cmd.startswith('SER:'):
                self.servo = clamp(int(cmd[4:]), SERVO_MIN, SERVO_MAX)
                self.stop_scenario()
            elif cmd.startswith('TL:R:'):
                self.tl_rate_ms = max(10, int(cmd[5:]))
            elif cmd == 'IMU:CAL':
                self.start_calibration()
            elif cmd == 'SCAL':
                pass   # stores servo neutral in EEPROM on the real car; nothing to simulate
            else:
                log(f'unknown command ignored: {cmd}')
                return
            log(f'command received: {cmd}')
        except ValueError:
            log(f'bad command value ignored: {cmd}')

    # ---------- scenario ----------
    def update_scenario(self, now):
        if not self.scenario_active or self.calibrating:
            return
        if self.scenario_t0 is None:
            self.scenario_t0 = now
        total = sum(step[0] for step in self.scenario)
        t = (now - self.scenario_t0) % total
        for duration, pwm, servo in self.scenario:
            if t < duration:
                self.pwm, self.servo = pwm, servo
                return
            t -= duration

    # ---------- physics (kinematic bicycle model) ----------
    def step(self, dt):
        if self.calibrating:
            return   # the car must stay still while the IMU calibrates
        target = speed_from_pwm(self.pwm)                                             # <<< CHANGED
        self.speed += (target - self.speed) * (1.0 - math.exp(-dt / SPEED_TAU_S))

        servo_offset = self.servo - SERVO_NEUTRAL
        if not SERVO_UP_IS_LEFT:
            servo_offset = -servo_offset
        wheel_angle = math.radians(servo_offset * STEER_RATIO)       # + = left
        self.yaw += self.speed / WHEELBASE_M * math.tan(wheel_angle) * dt

    # ---------- telemetry line, same format as the firmware ----------
    def telemetry_line(self):
        if self.calibrating:
            return '-1,-1,-1,-1'
        throttle_pct = round(abs(self.pwm) / PWM_MAX * 100)
        speed_mm = self.speed * 1000.0
        if not REVERSE_SPEED_NEGATIVE:
            speed_mm = abs(speed_mm)
        yaw_deg = math.degrees(self.yaw)
        if not YAW_CCW_POSITIVE:
            yaw_deg = -yaw_deg
        if self.noise:
            if abs(speed_mm) > 1.0:
                speed_mm += random.gauss(0.0, 10.0)
            yaw_deg += random.gauss(0.0, 0.5)
        if YAW_WRAP_180:                                                              # <<< CHANGED
            yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0                               # <<< CHANGED
        return f'{throttle_pct},{self.servo},{round(speed_mm)},{round(yaw_deg)}'


def describe(line):
    """Human-readable version of one telemetry line (for the terminal only)."""
    if line == '-1,-1,-1,-1':
        return '(IMU calibrating - car must stay still)'
    dc, servo, speed, yaw = (int(v) for v in line.split(','))
    off = servo - SERVO_NEUTRAL
    steer = 'STRAIGHT' if off == 0 else f'{abs(off)} deg {"LEFT" if (off > 0) == SERVO_UP_IS_LEFT else "RIGHT"}'
    return f'(throttle {dc:3d}% | servo {servo} = {steer} | {speed / 1000:+.2f} m/s | yaw {yaw:+d} deg)'


def main():
    parser = argparse.ArgumentParser(description='Fake F1R3 car on a virtual serial port')
    parser.add_argument('--scenario', choices=['oval', 'test'], default='oval',
                        help='built-in drive scenario (default: oval)')
    parser.add_argument('--noise', action='store_true', help='add small sensor noise')
    parser.add_argument('--quiet', action='store_true', help='do not print every telemetry line')
    args, _ = parser.parse_known_args()

    # Virtual serial port: we hold the "Arduino" end (master),
    # the reader (serial monitor / bridge) opens the other end.
    master, slave = pty.openpty()
    tty.setraw(slave)                  # raw bytes, no echo - like a real USB serial port
    os.set_blocking(master, False)
    slave_path = os.ttyname(slave)
    if os.path.lexists(LINK_PATH):
        os.remove(LINK_PATH)
    os.symlink(slave_path, LINK_PATH)

    if args.scenario == 'oval':
        scenario, v, radius = build_oval()
        log(f'scenario OVAL: {v:.2f} m/s, turn radius {radius:.2f} m, '
            f'size {2 * radius:.2f} x {OVAL_STRAIGHT_M + 2 * radius:.2f} m, '
            f'lap {sum(s[0] for s in scenario):.1f} s')
    else:
        scenario = TEST_SCENARIO
        log('scenario TEST: straight, turns, stop, reverse')

    log('vehicle model: MEASURED real-car speed table, servo > 100 = RIGHT, yaw not wrapped')  # <<< CHANGED
    log(f'virtual serial port: {LINK_PATH} -> {slave_path}   (real car: /dev/ttyACM0 @ {BAUD})')
    log(f'IMU calibrating for {CAL_SECONDS:.0f} s, then running the scenario')
    log('send commands from the reader side, e.g. DC:F:180  SER:120  DC:S  IMU:CAL')

    car = FakeCar(args.noise, scenario)
    rx_buf = b''
    last = time.monotonic()
    next_tx = last

    try:
        while True:
            # wake up at the next telemetry time, or every 20 ms for the physics
            timeout = max(0.0, min(next_tx, last + 0.02) - time.monotonic())
            ready, _, _ = select.select([master], [], [], timeout)
            if ready:
                try:
                    rx_buf += os.read(master, 1024)
                except (BlockingIOError, OSError):
                    pass
                while b'\n' in rx_buf:
                    line, rx_buf = rx_buf.split(b'\n', 1)
                    car.handle_command(line.decode(errors='ignore'))

            now = time.monotonic()
            car.update_scenario(now)
            car.step(now - last)
            last = now

            if now >= next_tx:
                line = car.telemetry_line()
                try:
                    os.write(master, (line + '\n').encode())
                except BlockingIOError:
                    pass   # nobody is reading: drop the line, like a real serial port
                if not args.quiet:
                    print(f'TX-SERIAL {line:<18} {describe(line)}', flush=True)
                next_tx = now + car.tl_rate_ms / 1000.0
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.lexists(LINK_PATH):
            os.remove(LINK_PATH)
        os.close(master)
        os.close(slave)
        print('\n[fake_car] stopped, virtual port removed.')


if __name__ == '__main__':
    main()
