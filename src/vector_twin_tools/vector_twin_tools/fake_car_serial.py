#!/usr/bin/env python3
"""
Fake F1R3 car - emulates the Arduino Nano RP2040 low-level-control firmware
(see "Vector Low Level Control User Guide").

It creates a VIRTUAL SERIAL PORT and behaves like the real car's USB serial:
  - telemetry line every 200 ms (TL:R:<ms> changes it):   DC,Servo,Speed,Yaw\n
        DC    : motor throttle        0..100 (%)
        Servo : servo angle           65..135 (100 = straight)
        Speed : linear speed          mm/s
        Yaw   : heading from the IMU  deg (0 = heading at calibration)
  - first seconds after start (and after IMU:CAL): -1,-1,-1,-1  (IMU calibrating)
  - accepts the firmware commands:
        DC:F:<0-255>  DC:R:<0-255>  DC:S  SER:<65-135>  TL:R:<ms>  IMU:CAL  SCAL

Until the first movement command arrives it drives a built-in test scenario.
The virtual port is linked at /tmp/f1r3_fake_serial  (real car: /dev/ttyACM0).

Usage:  ros2 run vector_twin_tools fake_car_serial [--noise] [--quiet]
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

# ---------------- Vehicle model: ASSUMPTIONS - confirm with the embedded team ----------------
WHEELBASE_M = 0.25             # front-to-rear axle distance
MAX_SPEED_MPS = 1.5            # speed at 100 % throttle
DEADBAND_PCT = 15.0            # below this throttle the motor does not turn
SPEED_TAU_S = 0.4              # how fast speed follows throttle (1st-order lag)
STEER_RATIO = 0.6              # front-wheel angle / servo angle
SERVO_UP_IS_LEFT = True        # servo > 100 steers LEFT
YAW_CCW_POSITIVE = True        # yaw grows when turning LEFT
REVERSE_SPEED_NEGATIVE = True  # Speed is negative when reversing

# Built-in drive scenario (used until the first movement command arrives):
# (duration_s, signed_pwm -255..255 (negative = reverse), servo_angle)
SCENARIO = [
    (2.0,    0, 100),   # stand still
    (3.0,  160, 100),   # straight
    (3.0,  160, 125),   # turn left
    (2.0,  160, 100),   # straight
    (3.0,  160,  75),   # turn right
    (2.0,    0, 100),   # stop
    (2.0, -120, 100),   # reverse
    (2.0,    0, 100),   # stop
]


def log(msg):
    print(f'[fake_car] {msg}', flush=True)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FakeCar:
    def __init__(self, noise):
        self.noise = noise
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
        total = sum(step[0] for step in SCENARIO)
        t = (now - self.scenario_t0) % total
        for duration, pwm, servo in SCENARIO:
            if t < duration:
                self.pwm, self.servo = pwm, servo
                return
            t -= duration

    # ---------- physics (kinematic bicycle model) ----------
    def step(self, dt):
        if self.calibrating:
            return   # the car must stay still while the IMU calibrates
        throttle_pct = abs(self.pwm) / PWM_MAX * 100.0
        if throttle_pct < DEADBAND_PCT:
            target = 0.0
        else:
            target = math.copysign(MAX_SPEED_MPS * throttle_pct / 100.0, self.pwm)
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
        yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0                 # wrap to [-180, 180)
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

    log(f'virtual serial port: {LINK_PATH} -> {slave_path}   (real car: /dev/ttyACM0 @ {BAUD})')
    log(f'IMU calibrating for {CAL_SECONDS:.0f} s, then running the test scenario')
    log('send commands from the reader side, e.g. DC:F:150  SER:120  DC:S  IMU:CAL')

    car = FakeCar(args.noise)
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
