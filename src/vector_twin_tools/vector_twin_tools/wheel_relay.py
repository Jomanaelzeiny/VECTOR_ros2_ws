#!/usr/bin/env python3
"""
Wheel relay: Logitech G920 stack (/control_commands) -> car firmware commands for the twin.

Reproduces what the base station's vector_bridge sends to the REAL car, but publishes
the commands on /f1r3/car/cmd instead of the Pi's TCP socket. serial_bridge forwards them
to the (fake or real) car; its telemetry moves the twin in CAR mode.

Input  /control_commands  ackermann_msgs/AckermannDriveStamped   (from logitech_controller)
    drive.speed          = gas - brake        in [-1, 1]
    drive.steering_angle = wheel axis         in [-1, 1]   (NOT radians, +1 = full left)
Output /f1r3/car/cmd      std_msgs/String   e.g. DC:F:180  DC:R:150  DC:S  SER:100

servo_mapping (parameter):                                                           # <<< CHANGED
    real     (default) : SAME as the fixed vector_bridge on the real car             # <<< CHANGED
                         SER = 100 - steering * 35   (servo > 100 steers RIGHT)      # <<< CHANGED
                         pedal: released -> DC:S, pressed -> min_pwm .. max_pwm      # <<< CHANGED
    original           : Adham's original bridge (SER = steering*180/3.14, pwm = speed*255)  # <<< CHANGED
Every 0.2 s, only when a value changed (same as vector_bridge); the current commands are
also re-sent every resend_period seconds so a late (re)connect still gets the state.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

SERVO_MIN, SERVO_NEUTRAL, SERVO_MAX = 65, 100, 135
PWM_MAX = 255
TIMER_SEC = 0.2          # same as vector_bridge


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WheelRelay(Node):
    def __init__(self):
        super().__init__('f1r3_wheel_relay')
        self.declare_parameter('servo_mapping', 'real')                              # <<< CHANGED
        self.declare_parameter('input_topic', '/control_commands')
        self.declare_parameter('output_topic', '/f1r3/car/cmd')
        self.declare_parameter('resend_period', 1.0)
        self.declare_parameter('min_pwm', 150)          # same as vector_bridge MIN_PWM   # <<< CHANGED
        self.declare_parameter('max_pwm', 255)          # same as vector_bridge MAX_PWM   # <<< CHANGED
        self.declare_parameter('pedal_deadzone', 0.05)  # same as vector_bridge            # <<< CHANGED

        self.mapping = str(self.get_parameter('servo_mapping').value).lower()
        if self.mapping not in ('real', 'original'):                                  # <<< CHANGED
            self.get_logger().warn(f"unknown servo_mapping '{self.mapping}', using 'real'")  # <<< CHANGED
            self.mapping = 'real'                                                     # <<< CHANGED
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.resend_period = float(self.get_parameter('resend_period').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)                      # <<< CHANGED
        self.max_pwm = int(self.get_parameter('max_pwm').value)                      # <<< CHANGED
        self.deadzone = float(self.get_parameter('pedal_deadzone').value)            # <<< CHANGED

        # Same start state as vector_bridge: speed 0, servo 100, "previous" values 0
        self.speed = 0.0
        self.steering = 0.0
        self.servo = SERVO_NEUTRAL
        self.prev_pwm = 0
        self.prev_servo = 0
        self.have_msg = False
        self.warned_clamp = False
        self.since_resend = 0.0

        self.pub = self.create_publisher(String, out_topic, 10)
        self.create_subscription(AckermannDriveStamped, in_topic, self.on_wheel, 10)
        self.create_timer(TIMER_SEC, self.tick)
        self.get_logger().info(
            f'Wheel relay: {in_topic} -> {out_topic}  (mapping = {self.mapping}, '
            f'pwm {self.min_pwm}..{self.max_pwm})')                                   # <<< CHANGED

    def to_servo(self, steering):
        if self.mapping == 'real':                                                    # <<< CHANGED
            servo = int(round(SERVO_NEUTRAL - steering * (SERVO_MAX - SERVO_NEUTRAL)))  # <<< CHANGED
            return clamp(servo, SERVO_MIN, SERVO_MAX)                                 # <<< CHANGED
        return int(round(steering * 180 / 3.14))          # original vector_bridge

    def to_pwm(self, pedal):                                                          # <<< CHANGED
        """Signed PWM, same as the bridge that drives the real car."""               # <<< CHANGED
        if self.mapping == 'original':                                                # <<< CHANGED
            pwm = int(round(pedal * PWM_MAX))                                         # <<< CHANGED
            return pwm if abs(pwm) > 2 else 0                                         # <<< CHANGED
        amount = min(1.0, abs(pedal))                                                 # <<< CHANGED
        if amount < self.deadzone:                                                    # <<< CHANGED
            return 0                                                                  # <<< CHANGED
        span = (amount - self.deadzone) / (1.0 - self.deadzone)                       # <<< CHANGED
        pwm = int(round(self.min_pwm + span * (self.max_pwm - self.min_pwm)))         # <<< CHANGED
        return pwm if pedal > 0 else -pwm                                             # <<< CHANGED

    def on_wheel(self, msg):
        self.speed = msg.drive.speed
        self.steering = msg.drive.steering_angle
        self.servo = self.to_servo(self.steering)
        if not self.have_msg:
            self.have_msg = True
            self.get_logger().info('receiving wheel commands')
        if not (SERVO_MIN <= self.servo <= SERVO_MAX) and not self.warned_clamp:
            self.warned_clamp = True
            self.get_logger().warn(
                f'SER:{self.servo} is outside {SERVO_MIN}..{SERVO_MAX}: the firmware snaps it to '
                f'{clamp(self.servo, SERVO_MIN, SERVO_MAX)} (steering stuck at one side).')

    def send(self, cmds):
        for c in cmds:
            self.pub.publish(String(data=c))

    @staticmethod
    def dc_command(pwm):                                                              # <<< CHANGED
        if pwm > 0:                                                                   # <<< CHANGED
            return f'DC:F:{pwm}'                                                      # <<< CHANGED
        if pwm < 0:                                                                   # <<< CHANGED
            return f'DC:R:{abs(pwm)}'                                                 # <<< CHANGED
        return 'DC:S'                                                                 # <<< CHANGED

    def tick(self):
        cmds = []
        pwm = self.to_pwm(self.speed)                                                 # <<< CHANGED
        if pwm != self.prev_pwm:
            self.prev_pwm = pwm
            cmds.append(self.dc_command(pwm))                                         # <<< CHANGED
        if self.servo != self.prev_servo:
            self.prev_servo = self.servo
            cmds.append(f'SER:{self.servo}')

        if cmds:
            self.send(cmds)
            self.since_resend = 0.0
            print(f'WHEEL speed={self.speed:+.2f} steer={self.steering:+.2f}  ->  {"  ".join(cmds)}', flush=True)
            return

        # Periodic re-send of the current state (so a late (re)connect still gets it)
        self.since_resend += TIMER_SEC
        if self.resend_period > 0 and self.since_resend >= self.resend_period:
            self.since_resend = 0.0
            self.send([self.dc_command(pwm), f'SER:{self.servo}'])                    # <<< CHANGED


def main():
    rclpy.init()
    node = WheelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.send(['DC:S'])   # leave the car stopped
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
