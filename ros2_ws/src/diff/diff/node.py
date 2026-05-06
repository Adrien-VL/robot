#!/usr/bin/env python3
import math
import sys
import time
import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg      # ← Renamed import for clarity
from tf2_ros import TransformBroadcaster


# ========================== CONFIG DEFAULTS ==========================
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
DEFAULT_MAX_SPEED = 0.10
DEFAULT_TICKS_PER_REV = 525 * 2
DEFAULT_WHEEL_RADIUS_M = 0.070
DEFAULT_WHEEL_BASE_M = 0.237
DEFAULT_CMD_TIMEOUT_S = 0.5
DEFAULT_ODOM_RATE_HZ = 50.0


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
    def __init__(self, pi, pin, debounce_us):
        self.pi = pi
        self.pin = pin
        self.tick_count = 0
        self.direction = 1
        self.last_tick = 0
        self.last_time = time.monotonic()
        self.ticks_per_sec = 0.0
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        pi.set_noise_filter(pin, debounce_us, 0)
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

    def get_rpm(self, ticks_per_rev):
        return self.ticks_per_sec * 60.0 / float(ticks_per_rev)

    def close(self):
        if self.callback:
            self.callback.cancel()


# ========================== MOTOR ==========================
class Motor:
    def __init__(self, pi, pin_fwd, pin_rev, pin_pwm, encoder,
                 pwm_freq_hz, pwm_range):
        self.pi = pi
        self.pin_fwd = pin_fwd
        self.pin_rev = pin_rev
        self.pin_pwm = pin_pwm
        self.encoder = encoder
        self.command = 0.0
        self.pwm_range = pwm_range
        for p in (pin_fwd, pin_rev, pin_pwm):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin_pwm, pwm_freq_hz)
        pi.set_PWM_range(pin_pwm, pwm_range)
        self.set_command(0.0)

    def set_command(self, cmd):
        cmd = clamp(cmd, -1.0, 1.0)
        self.command = cmd
        self.encoder.set_direction(sign(cmd) or self.encoder.direction)
        self.pi.write(self.pin_fwd, 1 if cmd > 0 else 0)
        self.pi.write(self.pin_rev, 1 if cmd < 0 else 0)
        duty = int(abs(cmd) * self.pwm_range)
        self.pi.set_PWM_dutycycle(self.pin_pwm, duty)

    def stop(self):
        self.pi.write(self.pin_fwd, 0)
        self.pi.write(self.pin_rev, 0)
        self.pi.set_PWM_dutycycle(self.pin_pwm, 0)
        self.command = 0.0


# ========================== WHEEL ODOMETRY ==========================
class WheelOdometry:
    """
    Differential-drive odometry using midpoint (RK2) integration.
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


# ========================== NODE ==========================
class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive')

        # ---------- Parameters ----------
        self.declare_parameter('left_fwd_pin', DEFAULT_PIN_LEFT_FWD)
        self.declare_parameter('left_rev_pin', DEFAULT_PIN_LEFT_REV)
        self.declare_parameter('left_pwm_pin', DEFAULT_PIN_LEFT_PWM)
        self.declare_parameter('right_fwd_pin', DEFAULT_PIN_RIGHT_FWD)
        self.declare_parameter('right_rev_pin', DEFAULT_PIN_RIGHT_REV)
        self.declare_parameter('right_pwm_pin', DEFAULT_PIN_RIGHT_PWM)
        self.declare_parameter('left_enc_pin', DEFAULT_PIN_LEFT_ENC)
        self.declare_parameter('right_enc_pin', DEFAULT_PIN_RIGHT_ENC)
        self.declare_parameter('pwm_freq_hz', DEFAULT_PWM_FREQ_HZ)
        self.declare_parameter('pwm_range', DEFAULT_PWM_RANGE)
        self.declare_parameter('enc_debounce_us', DEFAULT_ENC_DEBOUNCE_US)
        self.declare_parameter('ticks_per_rev', DEFAULT_TICKS_PER_REV)
        self.declare_parameter('wheel_radius_m', DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('wheel_base_m', DEFAULT_WHEEL_BASE_M)
        self.declare_parameter('deadband', DEFAULT_DEADBAND)
        self.declare_parameter('max_speed_cmd', DEFAULT_MAX_SPEED)
        self.declare_parameter('cmd_timeout_s', DEFAULT_CMD_TIMEOUT_S)
        self.declare_parameter('odom_rate_hz', DEFAULT_ODOM_RATE_HZ)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')

        # ---------- Read parameters ----------
        self.left_fwd_pin = self.get_parameter('left_fwd_pin').value
        self.left_rev_pin = self.get_parameter('left_rev_pin').value
        self.left_pwm_pin = self.get_parameter('left_pwm_pin').value
        self.right_fwd_pin = self.get_parameter('right_fwd_pin').value
        self.right_rev_pin = self.get_parameter('right_rev_pin').value
        self.right_pwm_pin = self.get_parameter('right_pwm_pin').value
        self.left_enc_pin = self.get_parameter('left_enc_pin').value
        self.right_enc_pin = self.get_parameter('right_enc_pin').value
        self.pwm_freq_hz = self.get_parameter('pwm_freq_hz').value
        self.pwm_range = self.get_parameter('pwm_range').value
        self.enc_debounce = self.get_parameter('enc_debounce_us').value
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.max_speed_cmd = float(self.get_parameter('max_speed_cmd').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.odom_rate_hz = float(self.get_parameter('odom_rate_hz').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # ---------- pigpio setup ----------
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon. Run 'sudo pigpiod'")
            raise RuntimeError("pigpio connection failed")

        self.pi.set_PWM_range(self.left_pwm_pin, self.pwm_range)
        self.pi.set_PWM_range(self.right_pwm_pin, self.pwm_range)

        # ---------- Encoders & motors ----------
        self.left_enc = Encoder(self.pi, self.left_enc_pin, self.enc_debounce)
        self.right_enc = Encoder(self.pi, self.right_enc_pin, self.enc_debounce)

        self.left_motor = Motor(self.pi, self.left_fwd_pin, self.left_rev_pin,
                                self.left_pwm_pin, self.left_enc,
                                self.pwm_freq_hz, self.pwm_range)
        self.right_motor = Motor(self.pi, self.right_fwd_pin, self.right_rev_pin,
                                 self.right_pwm_pin, self.right_enc,
                                 self.pwm_freq_hz, self.pwm_range)

        # ---------- Odometry ----------
        dist_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        self.odom = WheelOdometry(dist_per_tick=dist_per_tick,
                                  wheel_base_m=self.wheel_base,
                                  scale=1.0)
        self.odom.seed_ticks(self.left_enc.read(), self.right_enc.read())

        # ---------- ROS interfaces ----------
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(
            OdometryMsg, self.odom_topic, 10          # ← Use the imported message
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_cmd_time = self.get_clock().now()
        self.current_cmd = Twist()
        self.odom_timer = self.create_timer(1.0 / self.odom_rate_hz, self.update_loop)

        self.get_logger().info("DiffDrive node started.")

    def cmd_vel_callback(self, msg: Twist):
        self.current_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout_s:
            self.current_cmd = Twist()

        v = self.current_cmd.linear.x
        w = self.current_cmd.angular.z

        # --- Inverse kinematics ---
        v_r = v + (w * self.wheel_base / 2.0)
        v_l = v - (w * self.wheel_base / 2.0)

        max_wheel = self.max_speed_cmd

        # === SATURATION HANDLING ===
        max_cmd = max(abs(v_r), abs(v_l), 1e-6)
        if max_cmd > max_wheel:
            scale = max_wheel / max_cmd
            v_r *= scale
            v_l *= scale

        # === AGGRESSIVE TURNING BOOST ===
        if abs(v) > 0.05:          # Only apply when driving forward/backward
            turn_factor = 1.3     # ← Tune this! (1.3 = mild, 1.6 = very aggressive)

            boost = w * self.wheel_base * (turn_factor - 1.0) / 2.0
            v_r += boost
            v_l -= boost

            # Re-apply saturation after boost
            max_cmd = max(abs(v_r), abs(v_l), 1e-6)
            if max_cmd > max_wheel:
                scale = max_wheel / max_cmd
                v_r *= scale
                v_l *= scale

        # Convert to motor commands [-1.0 ... 1.0]
        cmd_r = clamp(v_r / max_wheel, -1.0, 1.0)
        cmd_l = clamp(v_l / max_wheel, -1.0, 1.0)

        # Apply deadband
        cmd_r = apply_deadband(cmd_r, self.deadband) if abs(cmd_r) > 1e-3 else 0.0
        cmd_l = apply_deadband(cmd_l, self.deadband) if abs(cmd_l) > 1e-3 else 0.0

        # Send to motors
        self.left_motor.set_command(cmd_l)
        self.right_motor.set_command(cmd_r)

        # Update encoders and odometry
        self.left_enc.update_speed()
        self.right_enc.update_speed()

        x, y, theta = self.odom.update(self.left_enc.read(), self.right_enc.read())
        self.publish_odom_and_tf(now, x, y, theta)

    def publish_odom_and_tf(self, stamp, x, y, theta):
        # TF
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom_msg = OdometryMsg()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = t.transform.rotation

        odom_msg.twist.twist.linear.x = float(self.current_cmd.linear.x)
        odom_msg.twist.twist.angular.z = float(self.current_cmd.angular.z)

        self.odom_pub.publish(odom_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down DiffDrive node...")
        try:
            self.left_motor.stop()
            self.right_motor.stop()
            self.left_enc.close()
            self.right_enc.close()
        except Exception:
            pass
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()