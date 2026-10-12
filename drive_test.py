#!/usr/bin/env python3
"""
VECTOR twin check: drive the REAL car straight for a set distance, then compare
  (a) the tape measure on the floor,
  (b) Unity's  Car_01 > Ros Car Telemetry Subscriber > Distance Travelled Cm,
  (c) what this script counted from the car's own (already scaled) telemetry.

Runs on the LAPTOP, plain python3, no ROS needed. Talks to the car's Pi exactly like
the base station does:
  commands  -> <pi-ip>:8888   (bridge_reciever.py, started with --car <name>)
  telemetry <- <pi-ip>:9000   (TRUE speed in mm/s, speed_scale already applied on the Pi)

The steering wheel / base station must NOT be connected at the same time
(the Pi accepts one command connection).

Safety: Ctrl+C or any error -> DC:S. If this script dies, the Pi stops the car by
itself when the connection drops. Hard time limit: --max-time.

Usage:
  python3 drive_test.py --pi 192.168.1.148                  # 200 cm at full power
  python3 drive_test.py --pi 192.168.1.148 --dist-cm 150 --pwm 220
"""
import argparse
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser(description='Drive the real car a set distance')
    ap.add_argument('--pi', required=True, help="car's Pi IP, e.g. 192.168.1.148")
    ap.add_argument('--dist-cm', type=float, default=200.0, help='distance to drive (cm)')
    ap.add_argument('--pwm', type=int, default=255, help='motor power 0..255 (255 = full)')
    ap.add_argument('--steer', type=int, default=100, help='SER value sent (100 = straight)')
    ap.add_argument('--max-time', type=float, default=30.0, help='safety limit (s)')
    a = ap.parse_args()

    # 1) telemetry first, so nothing is missed
    tel = socket.create_connection((a.pi, 9000), timeout=5)
    tel.settimeout(0.05)
    print(f'telemetry connected ({a.pi}:9000)')
    # 2) command connection (the Pi's receiver)
    cmd = socket.create_connection((a.pi, 8888), timeout=5)
    print(f'command connected ({a.pi}:8888)')

    def send(c):
        cmd.sendall((c + '\n').encode())
        print(f'  -> {c}')

    buf = b''
    last_t = None
    dist_mm = 0.0
    lines = 0
    target_mm = a.dist_cm * 10.0

    def read_lines():
        nonlocal buf
        try:
            data = tel.recv(4096)
        except socket.timeout:
            return []
        if not data:
            raise ConnectionError('telemetry connection closed by the Pi')
        buf += data
        out = []
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            out.append(line.decode(errors='ignore').strip())
        return out

    def integrate(line):
        """Add this line's speed x time since the previous line. Returns speed mm/s or None."""
        nonlocal last_t, dist_mm, lines
        parts = line.split(',')
        if len(parts) != 4 or line == '-1,-1,-1,-1':
            return None
        try:
            speed = float(parts[2])        # TRUE mm/s from the Pi (signed)
        except ValueError:
            return None
        now = time.monotonic()
        if last_t is not None:
            dist_mm += abs(speed) * (now - last_t)
        last_t = now
        lines += 1
        return speed

    try:
        send('DC:S')
        send(f'SER:{a.steer}')
        print('waiting 2 s for telemetry and straight wheels ...')
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            for ln in read_lines():
                integrate(ln)
        if lines == 0:
            sys.exit('no telemetry from the car - is bridge_reciever.py running on the Pi?')

        dist_mm = 0.0
        last_t = None
        print(f'\nDRIVING {a.dist_cm:.0f} cm at PWM {a.pwm}  (Ctrl+C = stop)')
        send(f'DC:F:{a.pwm}')
        t0 = time.monotonic()
        stop_reason = 'time limit'
        while time.monotonic() - t0 < a.max_time:
            for ln in read_lines():
                v = integrate(ln)
                if v is not None:
                    print(f'  t={time.monotonic() - t0:5.1f}s  speed {v:6.0f} mm/s  '
                          f'distance {dist_mm / 10:6.1f} cm', end='\r', flush=True)
            if dist_mm >= target_mm:
                stop_reason = 'distance reached'
                break
        send('DC:S')
        driven_mm = dist_mm
        drive_s = time.monotonic() - t0
        print(f'\nstopped: {stop_reason} after {drive_s:.1f} s')

        print('coasting ...')
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end:
            for ln in read_lines():
                integrate(ln)

        print('\n================ RESULT ================')
        print(f'  under power        : {driven_mm / 10:6.1f} cm  ({drive_s:.1f} s, '
              f'avg {driven_mm / max(drive_s, 0.01):.0f} mm/s)')
        print(f'  incl. coasting     : {dist_mm / 10:6.1f} cm   <- compare THIS one')
        print('  Now measure the real car with the tape (rear axle mark to rear axle)')
        print('  and read Unity: Car_01 > Ros Car Telemetry Subscriber > Distance Travelled Cm')
        print('  twin  ~ script  : Unity follows the car correctly')
        print('  tape  ~ script  : the car\'s speed_scale (car.yaml) is correct')
    except KeyboardInterrupt:
        print('\nCtrl+C')
    finally:
        try:
            send('DC:S')
            time.sleep(0.2)
        except OSError:
            pass
        cmd.close()
        tel.close()


if __name__ == '__main__':
    main()
