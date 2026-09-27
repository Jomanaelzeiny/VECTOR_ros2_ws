#!/usr/bin/env python3
"""
Prints the DIGITAL TWIN car's current position (published by Unity).

Topic: /f1r3/twin_pose   Type: geometry_msgs/msg/PoseStamped
The message is in METERS (real-life frame, origin = city top-left).
Printed values are in CENTIMETERS, same format as the fake camera publisher:
  TWIN t=<timestamp>  x=<cm>  y=<cm> cm
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

M_TO_CM = 100.0


def stamp_to_t(stamp):
    """Short readable ID: seconds mod 1000 + fraction (same as the publisher)."""
    return (stamp.sec % 1000) + stamp.nanosec * 1e-9


class TwinPoseListener(Node):
    def __init__(self):
        super().__init__('twin_pose_listener')
        self.sub = self.create_subscription(PoseStamped, '/f1r3/twin_pose', self.on_pose, 10)
        self.get_logger().info('Listening for the twin car position on /f1r3/twin_pose')

    def on_pose(self, msg):
        t = stamp_to_t(msg.header.stamp)
        x_cm = msg.pose.position.x * M_TO_CM
        y_cm = msg.pose.position.y * M_TO_CM
        print(f'TWIN t={t:8.3f}  x={x_cm:6.1f}  y={y_cm:6.1f} cm', flush=True)


def main():
    rclpy.init()
    node = TwinPoseListener()
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
