#!/usr/bin/env python3
"""
latency_probe.py  --  VECTOR digital twin, latency test, STEP 1

Measures how long a ROS2 message takes to reach the Unity twin and be
handled in a Unity frame, using a ping-pong:

  this node --/twin/latency/ping--> ros_tcp_endpoint --> Unity (LatencyEcho.cs)
  this node <--/twin/latency/pong-- ros_tcp_endpoint <-- Unity

The ping carries its send time in header.stamp. Unity sends the same message
back (it only appends its frame time to frame_id), so the round trip is
measured on ONE clock (this laptop's). No clock sync is needed.

    one-way latency  ~=  round trip / 2

Run (after ros_tcp_endpoint is running and Unity is in Play mode):
    python3 latency_probe.py --ros-args -p duration_s:=60.0 -p csv_path:=$HOME/latency_step1.csv

Parameters:
    rate_hz       pings per second                     (default 10.0)
    duration_s    measuring time, counted from the
                  first answer from Unity              (default 60.0)
    report_every  print live stats every N answers     (default 50)
    csv_path      save every sample to this CSV file   (default: no file)
    ping_topic    (default /twin/latency/ping)
    pong_topic    (default /twin/latency/pong)
"""

import csv
import math
import os
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


def percentile(values, p):
    """Linear-interpolated percentile, p in [0, 100]."""
    if not values:
        return float('nan')
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('latency_probe')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('duration_s', 60.0)
        self.declare_parameter('report_every', 50)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('ping_topic', '/twin/latency/ping')
        self.declare_parameter('pong_topic', '/twin/latency/pong')

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.duration_s = float(self.get_parameter('duration_s').value)
        self.report_every = int(self.get_parameter('report_every').value)
        self.csv_path = os.path.expanduser(str(self.get_parameter('csv_path').value))
        ping_topic = str(self.get_parameter('ping_topic').value)
        pong_topic = str(self.get_parameter('pong_topic').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Header, ping_topic, qos)
        self.sub = self.create_subscription(Header, pong_topic, self.on_pong, qos)

        self.seq = 0                  # next ping number to send
        self.last_sent_seq = -1
        self.first_pong_seq = None    # pings sent before Unity was ready don't count as lost
        self.t_first_pong = None
        self.received = set()
        self.rtts_ms = []
        self.unity_frame_ms = []
        self.rows = []                # (seq, rtt_ms, one_way_ms, unity_frame_ms)
        self.done = False
        self.finished = False
        self.finish_timer = None

        self.ping_timer = self.create_timer(1.0 / self.rate_hz, self.send_ping)

        print(f'[latency_probe] Pinging {ping_topic} at {self.rate_hz:.1f} Hz, '
              f'listening on {pong_topic}.', flush=True)
        print('[latency_probe] Waiting for Unity to answer '
              '(ros_tcp_endpoint running + Unity in Play mode with LatencyEcho) ...', flush=True)

    # ---------------------------------------------------------------- ping
    def send_ping(self):
        if self.t_first_pong is not None and time.monotonic() - self.t_first_pong >= self.duration_s:
            self.ping_timer.cancel()
            # wait 1 s so pongs still in flight can come back
            self.finish_timer = self.create_timer(1.0, self.finish)
            return

        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.seq)
        self.pub.publish(msg)
        self.last_sent_seq = self.seq
        self.seq += 1

    # ---------------------------------------------------------------- pong
    def on_pong(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        rtt_ms = (now_ns - sent_ns) / 1e6

        parts = msg.frame_id.split('|')
        try:
            seq = int(parts[0])
        except ValueError:
            return
        try:
            frame_ms = float(parts[1]) if len(parts) > 1 else float('nan')
        except ValueError:
            frame_ms = float('nan')

        if seq in self.received:      # ignore duplicates
            return
        self.received.add(seq)

        if self.first_pong_seq is None:
            self.first_pong_seq = seq
            self.t_first_pong = time.monotonic()
            print(f'[latency_probe] Unity is answering. Measuring for {self.duration_s:.0f} s ...',
                  flush=True)

        one_way_ms = rtt_ms / 2.0
        self.rtts_ms.append(rtt_ms)
        if not math.isnan(frame_ms):
            self.unity_frame_ms.append(frame_ms)
        self.rows.append((seq, rtt_ms, one_way_ms, frame_ms))

        if self.report_every > 0 and len(self.rtts_ms) % self.report_every == 0:
            self.print_stats(f'live, {len(self.rtts_ms)} samples')

    # ---------------------------------------------------------------- stats
    def print_stats(self, title, final=False):
        if not self.rtts_ms:
            print('[latency_probe] No answers from Unity. Check: ros_tcp_endpoint running, '
                  'Unity in Play mode, LatencyEcho on an active object, same topic names.',
                  flush=True)
            return

        rtt = self.rtts_ms
        one = [r / 2.0 for r in rtt]
        jitter = statistics.pstdev(one) if len(one) > 1 else 0.0
        frame = statistics.mean(self.unity_frame_ms) if self.unity_frame_ms else float('nan')

        lines = [
            '',
            f'========== LATENCY ({title}) ==========',
            f' samples              : {len(rtt)}',
            f' round trip  mean     : {statistics.mean(rtt):7.2f} ms',
            f' ONE-WAY     mean     : {statistics.mean(one):7.2f} ms',
            f' ONE-WAY     median   : {statistics.median(one):7.2f} ms',
            f' ONE-WAY     min      : {min(one):7.2f} ms',
            f' ONE-WAY     p95      : {percentile(one, 95):7.2f} ms',
            f' ONE-WAY     max      : {max(one):7.2f} ms',
            f' jitter (std one-way) : {jitter:7.2f} ms',
            f' Unity frame time     : {frame:7.2f} ms  (~{1000.0 / frame if frame > 0 else float("nan"):.0f} FPS)',
        ]
        if final and self.first_pong_seq is not None:
            expected = self.last_sent_seq - self.first_pong_seq + 1
            lost = max(0, expected - len(self.received))
            pct = 100.0 * lost / expected if expected > 0 else 0.0
            lines.append(f' lost pings           : {lost} / {expected} ({pct:.1f} %)')
        lines.append('=' * 46)
        print('\n'.join(lines), flush=True)

    def write_csv(self):
        if not self.csv_path or not self.rows:
            return
        folder = os.path.dirname(self.csv_path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['seq', 'round_trip_ms', 'one_way_ms', 'unity_frame_ms'])
            w.writerows(self.rows)
        print(f'[latency_probe] Saved {len(self.rows)} samples to {self.csv_path}', flush=True)

    def finish(self):
        if self.finished:
            return
        self.finished = True
        if self.finish_timer is not None:
            self.finish_timer.cancel()
        self.print_stats('FINAL', final=True)
        self.write_csv()
        self.done = True


def main():
    rclpy.init()
    node = LatencyProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\n[latency_probe] Stopped with Ctrl+C.', flush=True)
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()