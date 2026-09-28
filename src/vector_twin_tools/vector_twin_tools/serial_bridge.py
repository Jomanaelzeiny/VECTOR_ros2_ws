#!/usr/bin/env python3
"""
F1R3 serial bridge: car (USB serial) <-> ROS2.

Reads the low-level-control firmware telemetry line  DC,Servo,Speed,Yaw\n
(see "Vector Low Level Control User Guide") and publishes it to ROS2.
Works the same with the fake car (/tmp/f1r3_fake_serial) and the real car (/dev/ttyACM0).

Publishes:
  /f1r3/car/telemetry       std_msgs/Float32MultiArray
                            data = [throttle_pct, servo_deg, speed_mps, yaw_deg]
                            (only when the IMU is NOT calibrating)
  /f1r3/car/telemetry_raw   std_msgs/String   exact line received from the car
  /f1r3/car/calibrating     std_msgs/Bool     true while the car sends -1,-1,-1,-1
Subscribes:
  /f1r3/car/cmd             std_msgs/String   firmware command, forwarded to the car
                            e.g. DC:F:150  DC:R:100  DC:S  SER:120  TL:R:100  IMU:CAL

Parameters:
  port         default /tmp/f1r3_fake_serial   (real car: /dev/ttyACM0)
  baud         default 115200
  print_every  print every Nth telemetry line (0 = never)

Robustness: reconnects automatically if the port disappears (car unplugged /
fake car stopped), discards stale buffered data on connect, skips malformed lines.
"""
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

STALE_WARN_S = 1.0        # warn if connected but no line for this long
RECONNECT_PERIOD_S = 1.0  # how often to retry opening the port
MAX_BUF = 4096            # guard against garbage without newlines


class SerialBridge(Node):
    def __init__(self):
        super().__init__('f1r3_serial_bridge')
        self.declare_parameter('port', '/tmp/f1r3_fake_serial')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('print_every', 1)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.print_every = int(self.get_parameter('print_every').value)

        self.pub_tel = self.create_publisher(Float32MultiArray, '/f1r3/car/telemetry', 10)
        self.pub_raw = self.create_publisher(String, '/f1r3/car/telemetry_raw', 10)
        self.pub_cal = self.create_publisher(Bool, '/f1r3/car/calibrating', 10)
        self.create_subscription(String, '/f1r3/car/cmd', self.on_cmd, 10)

        self.ser = None
        self.buf = b''
        self.next_open_try = 0.0
        self.waiting_logged = False
        self.last_line_time = None
        self.stale_warned = False
        self.calibrating = None
        self.count = 0

        self.create_timer(0.02, self.tick)   # 50 Hz: read serial, check health
        self.get_logger().info(f'Serial bridge for port {self.port} @ {self.baud} baud')

    # ---------------- connection ----------------
    def try_open(self):
        now = time.monotonic()
        if now < self.next_open_try:
            return
        self.next_open_try = now + RECONNECT_PERIOD_S
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.ser.reset_input_buffer()          # drop stale lines from before we connected
            self.buf = b''
            self.last_line_time = time.monotonic()
            self.stale_warned = False
            self.waiting_logged = False
            self.get_logger().info(f'CONNECTED to {self.port}')
        except (serial.SerialException, OSError):
            self.ser = None
            if not self.waiting_logged:
                self.get_logger().warn(f'cannot open {self.port} - waiting for the car (retrying every {RECONNECT_PERIOD_S:.0f} s)')
                self.waiting_logged = True

    def disconnect(self, reason):
        self.get_logger().warn(f'DISCONNECTED from {self.port}: {reason}')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.calibrating = None

    # ---------------- main loop ----------------
    def tick(self):
        if self.ser is None:
            self.try_open()
            return

        try:
            data = self.ser.read(self.ser.in_waiting or 1)   # non-blocking (timeout=0)
        except (serial.SerialException, OSError) as e:
            self.disconnect(str(e))
            return

        if data:
            self.buf += data
            if len(self.buf) > MAX_BUF:
                self.buf = b''
            while b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                self.handle_line(line.decode(errors='ignore').strip())

        if self.last_line_time is not None:
            silent = time.monotonic() - self.last_line_time
            if silent > STALE_WARN_S and not self.stale_warned:
                self.get_logger().warn(f'connected but no telemetry for {silent:.1f} s')
                self.stale_warned = True

    # ---------------- parsing ----------------
    def handle_line(self, line):
        if not line:
            return
        parts = line.split(',')
        if len(parts) != 4:
            self.get_logger().debug(f'skipping malformed line: {line!r}')
            return
        try:
            dc, servo, speed_mm, yaw = (float(p) for p in parts)
        except ValueError:
            self.get_logger().debug(f'skipping non-numeric line: {line!r}')
            return

        self.last_line_time = time.monotonic()
        if self.stale_warned:
            self.get_logger().info('telemetry is back')
            self.stale_warned = False

        self.pub_raw.publish(String(data=line))

        calibrating = (dc == -1 and servo == -1 and speed_mm == -1 and yaw == -1)
        self.pub_cal.publish(Bool(data=calibrating))
        if calibrating != self.calibrating:
            self.get_logger().info('IMU CALIBRATING - car must stay still' if calibrating else 'IMU ready - telemetry valid')
            self.calibrating = calibrating
        if calibrating:
            return

        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(
            label='throttle_pct,servo_deg,speed_mps,yaw_deg', size=4, stride=4)]
        msg.data = [dc, servo, speed_mm / 1000.0, yaw]
        self.pub_tel.publish(msg)

        self.count += 1
        if self.print_every > 0 and self.count % self.print_every == 0:
            print(f'RX-SERIAL {line:<18} -> throttle {dc:5.1f} %  servo {servo:5.1f}  '
                  f'speed {speed_mm / 1000.0:+.3f} m/s  yaw {yaw:+6.1f} deg', flush=True)

    # ---------------- commands ----------------
    def on_cmd(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if self.ser is None:
            self.get_logger().warn(f'not connected - command dropped: {cmd}')
            return
        try:
            self.ser.write((cmd + '\n').encode())
            self.get_logger().info(f'command sent to car: {cmd}')
        except (serial.SerialException, OSError) as e:
            self.disconnect(str(e))


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
