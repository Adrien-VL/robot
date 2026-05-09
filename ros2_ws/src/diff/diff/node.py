#!/usr/bin/env python3
# /// script
# dependencies = ["pigpio", "rclpy"]
# ///

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import pigpio


# ========================== CONFIG (tunable via ROS parameters) ==========================
DEFAULT_PIN_LEFT_FWD = 24
DEFAULT_PIN_LEFT_REV = 25
DEFAULT_PIN_LEFT_PWM = 23
DEFAULT_PIN_RIGHT_FWD = 22
DEFAULT_PIN_RIGHT_REV = 27
DEFAULT_PIN_RIGHT_PWM = 13
DEFAULT_PIN_LEFT_ENC = 17
DEFAULT_PIN_RIGHT_ENC = 26

DEFAULT_PWM_FREQ_HZ = 200
DEFAULT_PWM_RANGE = 100
DEFAULT_ENC_DEBOUNCE_US = 200

DEFAULT_DEADBAND = 0.18
DEFAULT_MAX_WHEEL_SPEED_MPS = 0.288  # from your benchmark (avg steady-state)
DEFAULT_CMD_TIMEOUT_SEC = 0.5

DEFAULT_TICKS_PER_REV = 525 * 2
DEFAULT_WHEEL_RADIUS_M = 0.070
DEFAULT_WHEEL_BASE_M = 0.237

DIST_PER_TICK = 2 * math.pi * DEFAULT_WHEEL_RADIUS_M / DEFAULT_TICKS_PER_REV


# ========================== HELPERS ==========================
def clamp(value, low, high):
    return max(low, min(high, value))


def sign(value):
    return (value > 0) - (value < 0)


def apply_deadband(cmd, deadband):
    if abs(cmd) < 1e-6:
        return 0.0
    return math.copysign(deadband + (1.0 - deadband) * abs(cmd), cmd)


# ========================== ENCODER ==========================
class Encoder:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin
        self.tick_count = 0
        self.direction = 1
        self.last_tick = 0
        self.last_time = time.monotonic()
        self.ticks_per_sec = 0.0

        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        pi.set_noise_filter(pin, DEFAULT_ENC_DEBOUNCE_US, 0)
        self.callback = pi.callback(pin, pigpio.EITHER_EDGE, self._on_tick)

    def _on_tick(self, gpio, level, timestamp):
        self.tick_count += self.direction

    def set_direction(self, direction):
        if direction != 0:
            self.direction = direction

    def read(self):
        return self.tick_count

    def update_speed(self):
        now = time.monotonic()
        dt = now - self.last_time
        if dt >= 0.08:
            delta = self.tick_count - self.last_tick
            self.ticks_per_sec = delta / dt
            self.last_tick = self.tick_count
            self.last_time = now

    def get_rpm(self):
        return self.ticks_per_sec * 60 / DEFAULT_TICKS_PER_REV

    def close(self):
        if self.callback:
            self.callback.cancel()


# ========================== MOTOR ==========================
class Motor:
    def __init__(self, pi, pin_fwd, pin_rev, pin_pwm, encoder):
        self.pi = pi
        self.pin_fwd = pin_fwd
        self.pin_rev = pin_rev
        self.pin_pwm = pin_pwm
        self.encoder = encoder
        self.command = 0.0

        for p in (pin_fwd, pin_rev, pin_pwm):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin_pwm, DEFAULT_PWM_FREQ_HZ)
        pi.set_PWM_range(pin_pwm, DEFAULT_PWM_RANGE)
        self.set_command(0.0)

    def set_command(self, cmd):
        cmd = clamp(cmd, -1.0, 1.0)
        self.command = cmd
        self.encoder.set_direction(sign(cmd) or self.encoder.direction)
        self.pi.write(self.pin_fwd, 1 if cmd > 0 else 0)
        self.pi.write(self.pin_rev, 1 if cmd < 0 else 0)
        duty = int(abs(cmd) * DEFAULT_PWM_RANGE)
        self.pi.set_PWM_dutycycle(self.pin_pwm, duty)

    def stop(self):
        self.pi.write(self.pin_fwd, 0)
        self.pi.write(self.pin_rev, 0)
        self.pi.set_PWM_dutycycle(self.pin_pwm, 0)
        self.command = 0.0


# ========================== ODOMETRY (renamed to avoid collision with ROS msg) ==========================
class DiffDriveOdometry:
    """
    Differential-drive odometry using midpoint (Runge-Kutta 2) integration.
    Same logic as your original Odometry class.
    """
    def __init__(self, dist_per_tick: float, wheel_base_m: float, scale: float = 1.0):
        self.dist_per_tick = dist_per_tick
        self.wheel_base = wheel_base_m
        self.scale = scale

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._prev_left = 0
        self._prev_right = 0

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta

    def seed_ticks(self, left_ticks: int, right_ticks: int):
        self._prev_left = left_ticks
        self._prev_right = right_ticks

    def update(self, left_ticks: int, right_ticks: int):
        delta_l = (left_ticks - self._prev_left) * self.dist_per_tick * self.scale
        delta_r = (right_ticks - self._prev_right) * self.dist_per_tick * self.scale

        self._prev_left = left_ticks
        self._prev_right = right_ticks

        delta_dist = (delta_l + delta_r) / 2.0
        delta_theta = (delta_r - delta_l) / self.wheel_base

        mid_theta = self.theta + delta_theta / 2.0
        self.x += delta_dist * math.cos(mid_theta)
        self.y += delta_dist * math.sin(mid_theta)
        self.theta += delta_theta

        return self.x, self.y, self.theta

    @property
    def pose(self):
        return self.x, self.y, self.theta


# ========================== ROS 2 NODE ==========================
class DiffDriveRobotNode(Node):
    def __init__(self):
        super().__init__('diff_drive_robot')

        # Parameters (override with ros2 param set if needed)
        self.declare_parameter('deadband', DEFAULT_DEADBAND)
        self.declare_parameter('max_wheel_speed_mps', DEFAULT_MAX_WHEEL_SPEED_MPS)
        self.declare_parameter('cmd_timeout_sec', DEFAULT_CMD_TIMEOUT_SEC)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 200.0)

        self.deadband = self.get_parameter('deadband').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed_mps').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value

        self.get_logger().info(f"Starting diff-drive node | max_wheel_speed={self.max_wheel_speed:.3f} m/s | deadband={self.deadband:.2f}")

        # Hardware
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpiod. Run: sudo pigpiod")
            raise RuntimeError("pigpiod not running")

        self.left_enc = Encoder(self.pi, DEFAULT_PIN_LEFT_ENC)
        self.right_enc = Encoder(self.pi, DEFAULT_PIN_RIGHT_ENC)

        self.left_motor = Motor(self.pi, DEFAULT_PIN_LEFT_FWD, DEFAULT_PIN_LEFT_REV,
                                DEFAULT_PIN_LEFT_PWM, self.left_enc)
        self.right_motor = Motor(self.pi, DEFAULT_PIN_RIGHT_FWD, DEFAULT_PIN_RIGHT_REV,
                                 DEFAULT_PIN_RIGHT_PWM, self.right_enc)

        self.odometry = DiffDriveOdometry(
            dist_per_tick=DIST_PER_TICK,
            wheel_base_m=DEFAULT_WHEEL_BASE_M,
            scale=1.0
        )
        self.odometry.seed_ticks(self.left_enc.read(), self.right_enc.read())

        # ROS 2 interfaces
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.odom_pub = self.create_publisher(OdometryMsg, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        self._last_cmd_time = self.get_clock().now()
        self._last_cmd = Twist()

        self.get_logger().info("Diff-drive node ready for Nav2!")

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _timer_callback(self):
        now = self.get_clock().now()

        # Safety timeout
        if (now - self._last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.left_motor.stop()
            self.right_motor.stop()

        # Kinematics: cmd_vel → wheel velocities (m/s)
        linear = self._last_cmd.linear.x
        angular = self._last_cmd.angular.z
        half_base = DEFAULT_WHEEL_BASE_M / 2.0

        v_left = linear - angular * half_base
        v_right = linear + angular * half_base

        # Scale to motor command [-1, 1]
        cmd_left = clamp(v_left / self.max_wheel_speed, -1.0, 1.0)
        cmd_right = clamp(v_right / self.max_wheel_speed, -1.0, 1.0)

        # Apply deadband (same logic as original script)
        lc = apply_deadband(cmd_left, self.deadband)
        rc = apply_deadband(cmd_right, self.deadband)

        self.left_motor.set_command(lc)
        self.right_motor.set_command(rc)

        # Update encoders + odometry
        self.left_enc.update_speed()
        self.right_enc.update_speed()
        x, y, theta = self.odometry.update(
            self.left_enc.read(), self.right_enc.read()
        )

        # Current measured body velocities (for Odometry twist)
        left_v = self.left_enc.ticks_per_sec * DIST_PER_TICK
        right_v = self.right_enc.ticks_per_sec * DIST_PER_TICK
        measured_linear = (left_v + right_v) / 2.0
        measured_angular = (right_v - left_v) / DEFAULT_WHEEL_BASE_M

        # Publish Odometry
        odom_msg = OdometryMsg()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        # quaternion (yaw only)
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sy
        odom_msg.pose.pose.orientation.w = cy

        odom_msg.twist.twist.linear.x = measured_linear
        odom_msg.twist.twist.angular.z = measured_angular

        # Simple covariance (tune as needed)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom_msg)

        # Broadcast TF odom → base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f"Odom: x={x:.3f} y={y:.3f} θ={math.degrees(theta):+.1f}°", throttle_duration_sec=0.5)

    def destroy_node(self):
        self.get_logger().info("Shutting down – stopping motors")
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_enc.close()
        self.right_enc.close()
        if self.pi:
            self.pi.stop()
        super().destroy_node()


# ========================== MAIN ==========================
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()