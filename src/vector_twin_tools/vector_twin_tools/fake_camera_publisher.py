#!/usr/bin/env python3
"""
Fake camera-tracking publisher for the VECTOR digital twin.
Stands in for the real camera system until it is ready.

Publishes the car's position in the REAL-LIFE frame, in METERS (ROS standard):
  origin (0,0) = city top-left corner
  +x = along the city's 475 cm side (0 -> 4.75 m)
  +y = along the city's 380 cm side (0 -> 3.80 m)
Topic: /f1r3/pose   Type: geometry_msgs/msg/PoseStamped

PATTERN = 'figure8' draws a path Unity's TelemetrySimulator can NOT make,
so if the car drives a figure-8, the motion is definitely coming from ROS2.

Every printed line carries t = the message timestamp (last 3 digits of the
seconds + milliseconds). Unity shows the SAME t, so lines can be matched 1:1.
The MESSAGE is in meters; the PRINTED values are in CENTIMETERS for reading.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

PATTERN = 'figure8'   # 'figure8' or 'circle'
PRINT_EVERY = 1       # print every Nth message (1 = every message, 20 Hz)
M_TO_CM = 100.0


def stamp_to_t(stamp):
    """Short readable ID for a message: seconds mod 1000 + fraction."""
    return (stamp.sec % 1000) + stamp.nanosec * 1e-9


class FakeCameraPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_publisher')
        self.pub = self.create_publisher(PoseStamped, '/f1r3/pose', 10)

        # Path center in the REAL frame (meters) = city center
        self.center_x = 2.375   # middle of the 4.75 m side
        self.center_y = 1.90    # middle of the 3.80 m side
        self.radius = 0.5     # circle radius / figure-8 half-length along x
        self.fig8_width = 1.00  # figure-8 half-width along y (meters)
        self.speed_mps = 0.5

        self.rate_hz = 20.0     # cameras usually send 10-30 updates per second
        self.theta = 0.0
        self.count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(f'Publishing fake camera pose on /f1r3/pose (pattern: {PATTERN})')

    def tick(self):
        omega = self.speed_mps / self.radius
        self.theta += omega / self.rate_hz

        if PATTERN == 'figure8':
            # Figure-8 lying along x: crosses itself at the city center
            x = self.center_x + self.radius * math.sin(self.theta)
            y = self.center_y + self.fig8_width * math.sin(self.theta) * math.cos(self.theta)
        else:
            x = self.center_x + self.radius * math.cos(self.theta)
            y = self.center_y + self.radius * math.sin(self.theta)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'city'
        msg.pose.position.x = x          # meters (ROS standard)
        msg.pose.position.y = y          # meters (ROS standard)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0   # heading not used yet

        self.pub.publish(msg)

        # Print with the SAME format Unity uses: t, x, y in CENTIMETERS, 1 decimal
        self.count += 1
        if self.count % PRINT_EVERY == 0:
            t = stamp_to_t(msg.header.stamp)
            print(f'TX t={t:8.3f}  x={x * M_TO_CM:6.1f}  y={y * M_TO_CM:6.1f} cm', flush=True)


def main():
    rclpy.init()
    node = FakeCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():          # avoids the 'rcl_shutdown already called' error on Ctrl+C
            rclpy.shutdown()


if __name__ == '__main__':
    main()
