#!/usr/bin/env python3
"""
Fake overhead-camera publisher for a WHOLE FLEET (multi-car test of the twin).

Same data format as fake_camera_publisher, but one topic per car:
  /car_01/pose, /car_02/pose, ...   geometry_msgs/msg/PoseStamped, METERS
  origin (0,0) = city top-left corner
  +x = along the city's 475 cm side (0 -> 4.75 m)
  +y = along the city's 380 cm side (0 -> 3.80 m)

All cars drive the same figure-8, spread out evenly along it (different phase),
so you can see every car move at once without them sitting on top of each other.

Parameters (change without editing the file):
  ros2 run vector_twin_tools fake_fleet_publisher --ros-args -p num_cars:=5
  -p first_id:=1  -p namespace_format:='/car_{:02d}'  -p speed_mps:=0.4
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

M_TO_CM = 100.0
PRINT_EVERY = 20   # print one summary line every N ticks (20 = once per second)


class FakeFleetPublisher(Node):
    def __init__(self):
        super().__init__('fake_fleet_publisher')

        self.declare_parameter('num_cars', 3)
        self.declare_parameter('first_id', 1)
        self.declare_parameter('namespace_format', '/car_{:02d}')   # must match Unity's FleetManager
        self.declare_parameter('speed_mps', 0.4)
        self.declare_parameter('rate_hz', 20.0)

        n = int(self.get_parameter('num_cars').value)
        first = int(self.get_parameter('first_id').value)
        fmt = str(self.get_parameter('namespace_format').value)
        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # Figure-8 in the REAL frame (meters), centred on the city
        self.center_x = 2.375
        self.center_y = 1.90
        self.half_len = 1.20     # along x
        self.half_wid = 1.00     # along y

        self.ids = list(range(first, first + n))
        self.pubs = {}
        self.phase = {}
        for k, car_id in enumerate(self.ids):
            topic = fmt.format(car_id) + '/pose'
            self.pubs[car_id] = self.create_publisher(PoseStamped, topic, 10)
            self.phase[car_id] = 2.0 * math.pi * k / n      # spread evenly along the path
            self.get_logger().info(f'Car {car_id:02d} -> {topic}')

        self.theta = 0.0
        self.count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        omega = self.speed_mps / self.half_len
        self.theta += omega / self.rate_hz
        stamp = self.get_clock().now().to_msg()

        line = []
        for car_id in self.ids:
            th = self.theta + self.phase[car_id]
            x = self.center_x + self.half_len * math.sin(th)
            y = self.center_y + self.half_wid * math.sin(th) * math.cos(th)

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'city'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0   # heading not used yet
            self.pubs[car_id].publish(msg)

            line.append(f'{car_id:02d}:({x * M_TO_CM:5.1f},{y * M_TO_CM:5.1f})')

        self.count += 1
        if self.count % PRINT_EVERY == 0:
            print('TX cm  ' + '  '.join(line), flush=True)


def main():
    rclpy.init()
    node = FakeFleetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()