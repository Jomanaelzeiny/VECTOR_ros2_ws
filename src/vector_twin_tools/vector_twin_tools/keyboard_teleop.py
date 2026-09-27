#!/usr/bin/env python3
"""
Keyboard teleop for the VECTOR digital twin car (F1R3).
Runs on ANY lab PC with ROS2 (Humble or Jazzy) on the same network and ROS_DOMAIN_ID.

Publishes:
  /f1r3/cmd_vel  geometry_msgs/Twist   linear.x = speed (m/s), angular.z = turn (rad/s, + = LEFT)
  /f1r3/mode     std_msgs/String       "manual" or "camera"

The current command is re-sent 10x per second, so Unity's dead-man rule
(stop if no command for 0.5 s) only triggers if this program stops.
The chosen mode is re-sent every second, so a missed message or a Unity   # <<< CHANGED
restart is corrected automatically.                                          # <<< CHANGED
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ---- Tuning (the car is a scaled model, so keep these modest) ----
SPEED_STEP = 0.05     # m/s added per W/S press
TURN_STEP = 0.25      # rad/s added per A/D press
MAX_FWD = 1.0         # m/s
MAX_REV = 0.5         # m/s (reverse)
MAX_TURN = 1.5        # rad/s (Unity also limits this to its steering speed)
PUBLISH_HZ = 10.0     # how often the command is re-sent
MODE_RESEND_S = 1.0   # how often the chosen mode is re-sent (seconds)              # <<< CHANGED

HELP = """
==================== F1R3 KEYBOARD TELEOP ====================
  W / S : speed up / slow down (below 0 = reverse)
  A / D : turn more left / more right
  X     : straighten (turn = 0)
  SPACE : STOP (speed = 0, turn = 0)
  M     : switch car to MANUAL mode   (keyboard drives)
  C     : switch car to CAMERA mode   (camera tracking drives)
  Q     : quit (sends a stop command first)
===============================================================
Press M first, then W to start driving.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('f1r3_keyboard_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/f1r3/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/f1r3/mode', 10)
        self.speed = 0.0   # m/s
        self.turn = 0.0    # rad/s, + = left
        self.mode = '-'    # last mode we sent ('-' = none yet)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.turn
        self.cmd_pub.publish(msg)

    def publish_mode(self):                                                          # <<< CHANGED
        """(Re-)send the chosen mode; does nothing until M or C was pressed."""      # <<< CHANGED
        if self.mode == '-':                                                         # <<< CHANGED
            return                                                                   # <<< CHANGED
        msg = String()                                                               # <<< CHANGED
        msg.data = self.mode.lower()                                                 # <<< CHANGED
        self.mode_pub.publish(msg)                                                   # <<< CHANGED

    def send_mode(self, mode):
        self.mode = mode.upper()
        self.publish_mode()                                                          # <<< CHANGED
        # Always start a new mode from standstill
        self.speed = 0.0
        self.turn = 0.0

    def handle_key(self, key):
        """Update the command from one key press. Returns False to quit."""
        if key == 'w':
            self.speed = min(self.speed + SPEED_STEP, MAX_FWD)
        elif key == 's':
            self.speed = max(self.speed - SPEED_STEP, -MAX_REV)
        elif key == 'a':
            self.turn = min(self.turn + TURN_STEP, MAX_TURN)
        elif key == 'd':
            self.turn = max(self.turn - TURN_STEP, -MAX_TURN)
        elif key == 'x':
            self.turn = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.turn = 0.0
        elif key == 'm':
            self.send_mode('manual')
        elif key == 'c':
            self.send_mode('camera')
        elif key == 'q':
            return False

        # Avoid float drift like 0.30000000004
        self.speed = round(self.speed, 3)
        self.turn = round(self.turn, 3)
        return True

    def print_status(self):
        direction = 'LEFT ' if self.turn > 0 else ('RIGHT' if self.turn < 0 else 'STRAIGHT')
        hint = ''                                                                    # <<< CHANGED
        if self.mode != 'MANUAL' and (self.speed != 0.0 or self.turn != 0.0):        # <<< CHANGED
            hint = '  <-- car ignores this: press M for MANUAL'                      # <<< CHANGED
        sys.stdout.write(
            f'\rMODE: {self.mode:<7} | speed: {self.speed:+.2f} m/s | '
            f'turn: {self.turn:+.2f} rad/s ({direction}){hint}      '                # <<< CHANGED
        )
        sys.stdout.flush()


def read_key(timeout):
    """Return one key if pressed within timeout seconds, else None."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def main():
    if not sys.stdin.isatty():
        print('This program needs a real terminal (keyboard input).')
        return

    rclpy.init()
    node = KeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.print_status()

    period = 1.0 / PUBLISH_HZ
    next_publish = time.monotonic()
    next_mode_publish = time.monotonic() + MODE_RESEND_S                             # <<< CHANGED

    try:
        tty.setcbreak(sys.stdin.fileno())   # read keys one by one, no Enter needed
        running = True
        while running and rclpy.ok():
            key = read_key(max(0.0, next_publish - time.monotonic()))
            if key is not None:
                running = node.handle_key(key.lower())
                node.print_status()

            now = time.monotonic()
            if now >= next_publish:
                node.publish_cmd()
                next_publish = now + period

            if now >= next_mode_publish:                                             # <<< CHANGED
                node.publish_mode()                                                  # <<< CHANGED
                next_mode_publish = now + MODE_RESEND_S                              # <<< CHANGED

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car stopped (only possible if ROS is still up - after Ctrl+C   # <<< CHANGED
        # it may already be shut down; Unity's dead-man rule stops the car then)   # <<< CHANGED
        node.speed = 0.0
        node.turn = 0.0
        try:                                                                         # <<< CHANGED
            if rclpy.ok():                                                           # <<< CHANGED
                node.publish_cmd()                                                   # <<< CHANGED
        except Exception:                                                            # <<< CHANGED
            pass                                                                     # <<< CHANGED
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('\nTeleop closed - car stopped.')                                      # <<< CHANGED
        try:                                                                         # <<< CHANGED
            node.destroy_node()                                                      # <<< CHANGED
        except Exception:                                                            # <<< CHANGED
            pass                                                                     # <<< CHANGED
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
