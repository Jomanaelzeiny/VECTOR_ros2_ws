#!/usr/bin/env python3
"""
VECTOR Mixed Reality - direct link from Unity's path follower to the real car
============================================================================

No lab-PC / wheel software involved. This node runs on YOUR laptop:

    Unity (UnityPathFollower)
        /car_01/mr/cmd_vel   geometry_msgs/Twist   (linear.x = m/s, angular.z = rad/s, + = LEFT)
              |
              v
    mr_car_bridge.py  (this file)
        speed   -> motor PWM   (your measured speed table)
        turn    -> servo value (bicycle model)
              |
              |  TCP  <car_ip>:8888   text lines  "DC:F:<pwm>\n"  "SER:<servo>\n"
              v
    Pi  bridge_reciever.py  -> USB -> Arduino -> motor + servo

The Pi's bridge_reciever applies the car's own corrections (--invert-motor,
--ser-center, --pwm-min ...), so this node sends NOMINAL values
(servo center 100, range +-35), the same as the wheel always did.

Vehicle values = vehicle_config.py (VehicleConfig), same as Unity's PathPlanner:
    wheelbase 0.26 m, max steering 24.5 deg (config.yaml: the car WHILE DRIVING),   # <<< CHANGED
    servo full LEFT = 65, full RIGHT = 135  -> center 100, span 35
    (the servo center/span are now DERIVED from servo_left_deg/servo_right_deg)

Telemetry back to Unity is unchanged (serial_bridge reads <car_ip>:9000).

Safety:
    - no Unity command for timeout_s  -> STOP  (DC:F:0, servo center)
    - PWM never above pwm_limit
    - Ctrl+C sends STOP before exiting
    - lost connection -> reconnects automatically every 2 s

Run (laptop):
    cd ~/VECTOR/mixed_reality
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=42
    python3 mr_car_bridge.py --ros-args -p car_ip:=192.168.1.154
"""
import math
import socket
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Measured on the real car (wheels lifted): motor PWM -> speed (m/s)
SPEED_TABLE = [  # (pwm, m/s)  car .115 (vector115), measured ON THE MAT 10 Nov 2026   # <<< CHANGED
    (160, 0.033),
    (180, 0.043),
    (200, 0.061),
    (220, 0.081),
    (240, 0.108),
    (255, 0.111),
]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def speed_to_pwm(v):
    """Inverse of the speed table (linear interpolation). v below the slowest entry -> slowest PWM."""
    if v <= SPEED_TABLE[0][1]:
        return SPEED_TABLE[0][0]
    for (p0, v0), (p1, v1) in zip(SPEED_TABLE, SPEED_TABLE[1:]):
        if v <= v1:
            t = (v - v0) / (v1 - v0)
            return p0 + t * (p1 - p0)
    return SPEED_TABLE[-1][0]


class MrCarBridge(Node):
    def __init__(self):
        super().__init__('mr_car_bridge')

        # ---------- parameters ----------
        self.declare_parameter('car_ip', '192.168.1.154')
        self.declare_parameter('car_port', 8888)
        self.declare_parameter('in_topic', '/car_01/mr/cmd_vel')
        self.declare_parameter('wheelbase_m', 0.26)
        self.declare_parameter('max_steer_deg', 24.5)   # <<< CHANGED (was 30): config.yaml max_steering_angle_deg - full lock = 24.5 deg while driving
        self.declare_parameter('servo_left_deg', 65)
        self.declare_parameter('servo_right_deg', 135)
        self.declare_parameter('invert_steer', False)   # flip if the wheels steer the wrong way
        self.declare_parameter('pwm_min', 150)
        self.declare_parameter('pwm_limit', 185)        # hard cap (~0.45 m/s)
        self.declare_parameter('timeout_s', 0.3)        # no Unity command for this long -> STOP
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('keepalive_s', 0.25)     # resend unchanged commands this often

        p = lambda n: self.get_parameter(n).value
        self.car_ip = p('car_ip')
        self.car_port = int(p('car_port'))
        self.in_topic = p('in_topic')
        self.L = float(p('wheelbase_m'))
        self.max_steer = math.radians(float(p('max_steer_deg')))
        sl, sr = int(p('servo_left_deg')), int(p('servo_right_deg'))
        self.ser_center = int(round((sl + sr) / 2.0))
        self.ser_span = int(round(abs(sr - sl) / 2.0))
        self.left_is_low = sl < sr   # True: servo BELOW center = LEFT
        self.invert_steer = bool(p('invert_steer'))
        self.pwm_min = int(p('pwm_min'))
        self.pwm_limit = int(p('pwm_limit'))
        if self.pwm_limit < self.pwm_min:
            self.pwm_limit = self.pwm_min
        self.timeout_s = float(p('timeout_s'))
        self.keepalive_s = float(p('keepalive_s'))
        rate_hz = float(p('rate_hz'))

        # ---------- state ----------
        self.sock = None
        self.last_connect_try = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_rx = 0.0
        self.driving = False
        self.last_sent = (None, None)
        self.last_send_time = 0.0
        self.last_log = 0.0

        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f'{self.in_topic} -> TCP {self.car_ip}:{self.car_port} | L={self.L} m, '
            f'max_steer={math.degrees(self.max_steer):.0f} deg, servo L{sl}/C{self.ser_center}/R{sr}, '
            f'pwm {self.pwm_min}..{self.pwm_limit}, invert_steer={self.invert_steer}, timeout={self.timeout_s} s')

    # ---------------------------------------------------------
    # connection
    # ---------------------------------------------------------
    def ensure_connected(self):
        if self.sock is not None:
            return True
        now = time.monotonic()
        if now - self.last_connect_try < 2.0:
            return False
        self.last_connect_try = now
        try:
            s = socket.create_connection((self.car_ip, self.car_port), timeout=1.5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(0.5)
            self.sock = s
            self.last_sent = (None, None)
            self.get_logger().info(f'CONNECTED to car {self.car_ip}:{self.car_port}')
            self.write_line(f'SER:{self.ser_center}')
            self.write_line('DC:F:0')
            return True
        except OSError as e:
            self.get_logger().warn(f'Cannot connect to {self.car_ip}:{self.car_port} ({e}) - is bridge_reciever running on the Pi? Retrying...')
            self.sock = None
            return False

    def write_line(self, text):
        if self.sock is None:
            return False
        try:
            self.sock.sendall((text + '\n').encode())
            return True
        except OSError as e:
            self.get_logger().warn(f'Connection lost ({e}) - will reconnect')
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            return False

    # ---------------------------------------------------------
    # ROS input
    # ---------------------------------------------------------
    def on_cmd(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last_rx = time.monotonic()

    # ---------------------------------------------------------
    # main loop
    # ---------------------------------------------------------
    def tick(self):
        if not self.ensure_connected():
            return

        now = time.monotonic()
        stale = (now - self.last_rx) > self.timeout_s
        v, w = (0.0, 0.0) if stale else (self.v, self.w)

        if v <= 0.02:
            pwm, ser, steer_norm = 0, self.ser_center, 0.0
        else:
            # inverse bicycle model: omega = v / L * tan(delta)  ->  delta = atan(omega * L / v)
            delta = math.atan(w * self.L / v)
            steer_norm = clamp(delta / self.max_steer, -1.0, 1.0)   # + = LEFT
            if self.invert_steer:
                steer_norm = -steer_norm
            side = -1 if self.left_is_low else 1
            ser = int(round(self.ser_center + side * steer_norm * self.ser_span))
            pwm = int(round(clamp(speed_to_pwm(v), self.pwm_min, self.pwm_limit)))

        driving = pwm > 0
        if driving != self.driving:
            self.get_logger().info('Unity is DRIVING the car' if driving else
                                   ('STOP (no command from Unity)' if stale else 'STOP (Unity asked to stop)'))
            self.driving = driving

        changed = (pwm, ser) != self.last_sent
        if changed or (now - self.last_send_time) > self.keepalive_s:
            ok = self.write_line(f'SER:{ser}') and self.write_line(f'DC:F:{pwm}')
            if ok:
                self.last_sent = (pwm, ser)
                self.last_send_time = now

        if driving and now - self.last_log > 0.5:
            self.last_log = now
            self.get_logger().info(
                f'Unity v={v:.2f} m/s w={w:+.2f} rad/s -> DC:F:{pwm}  SER:{ser}  '
                f'(steer {steer_norm * math.degrees(self.max_steer):+.1f} deg)')

    # ---------------------------------------------------------
    def stop_car(self):
        for _ in range(3):
            if self.sock is None:
                return
            self.write_line(f'SER:{self.ser_center}')
            self.write_line('DC:F:0')
            time.sleep(0.05)
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    rclpy.init()
    node = MrCarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        print('[mr_car_bridge] sent STOP to the car, exiting')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()