#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import pigpio
import struct
import time
import math


class LidarNode(Node):
  def __init__(self):
    super().__init__('lidar')

    # ========================== PARAMETERS ==========================
    self.declare_parameter('pwm_pin', 12)
    self.declare_parameter('pwm_freq', 1000)
    self.declare_parameter('pwm_duty', 70)

    self.declare_parameter('port', '/dev/ttyUSB0')
    self.declare_parameter('baud', 115200)

    self.declare_parameter('min_dist_mm', 200)
    self.declare_parameter('max_dist_mm', 6000)
    self.declare_parameter('angle_offset_deg', 90)

    self.declare_parameter('frame_id', 'laser_frame')
    self.declare_parameter('scan_topic', 'scan')

    # Get parameters
    self.PWM_PIN      = self.get_parameter('pwm_pin').value
    self.PWM_FREQ     = self.get_parameter('pwm_freq').value
    self.PWM_DUTY     = self.get_parameter('pwm_duty').value

    self.LIDAR_PORT   = self.get_parameter('port').value
    self.BAUD         = self.get_parameter('baud').value

    self.MIN_DIST_MM  = self.get_parameter('min_dist_mm').value
    self.MAX_DIST_MM  = self.get_parameter('max_dist_mm').value
    self.OFFSET       = self.get_parameter('angle_offset_deg').value % 360

    self.FRAME_ID     = self.get_parameter('frame_id').value
    self.SCAN_TOPIC   = self.get_parameter('scan_topic').value

    # pigpio setup
    self.pi = pigpio.pi()
    if not self.pi.connected:
      self.get_logger().error("Failed to connect to pigpio daemon. Run 'sudo pigpiod'")
      raise RuntimeError("pigpio connection failed")

    self.pi.set_mode(self.PWM_PIN, pigpio.OUTPUT)
    self.pi.set_PWM_frequency(self.PWM_PIN, self.PWM_FREQ)
    self.pi.set_PWM_dutycycle(self.PWM_PIN, int(self.PWM_DUTY * 255 / 100))
    self.get_logger().info(f"Motor running at {self.PWM_DUTY}% duty on pin {self.PWM_PIN}")

    # Serial
    self.ser = serial.Serial(self.LIDAR_PORT, self.BAUD, timeout=0.05)
    self.ser.reset_input_buffer()
    self.get_logger().info(f"Serial opened on {self.LIDAR_PORT}")

    # Publisher
    self.scan_pub = self.create_publisher(LaserScan, self.SCAN_TOPIC, 10)

    # State
    self.buf = bytearray()
    self.scan = [None] * 360
    self.last_angle = -1
    self.rpm = 0.0
    self.packets_processed = 0
    self.last_print = time.monotonic()

    # Timer (fast reading)
    self.timer = self.create_timer(0.005, self.update)

    self.get_logger().info(
      f"LiDAR node started — offset={self.OFFSET}° (CW→CCW mirror + rotation)"
    )

  def checksum_ok(self, p):
    expected = struct.unpack_from('<H', p, 20)[0]
    chk = 0
    for i in range(10):
      word = (p[i*2+1] << 8) | p[i*2]
      chk = (chk << 1) + word
    chk = (chk & 0x7FFF) + (chk >> 15)
    return (chk & 0x7FFF) == expected

  def parse_packet(self, p):
    if p[0] != 0xFA:
      return None
    if not self.checksum_ok(p):
      return None

    angle_base = (p[1] - 0xA0) * 4
    self.rpm = ((p[3] << 8) | p[2]) / 64.0

    points = []
    for i in range(4):
      off = 4 + i * 4
      dist = ((p[off+1] & 0x3F) << 8) | p[off]
      # Store raw ascending angle — mirror/offset handled at publish time
      angle = (angle_base + i) % 360
      points.append((angle, dist))
    return self.rpm, points

  def update(self):
    chunk = self.ser.read(256)
    if chunk:
      self.buf.extend(chunk)

    while len(self.buf) >= 22:
      idx = self.buf.find(0xFA)
      if idx == -1:
        break
      if idx > 0:
        del self.buf[:idx]

      if len(self.buf) < 22:
        break

      pkt = self.buf[:22]
      result = self.parse_packet(pkt)
      del self.buf[:22]

      if not result:
        continue

      self.packets_processed += 1
      rpm, pts = result

      scan_complete = False

      for a, d in pts:
        # Wrap detection: raw angles always ascend 0→359, so only need one direction
        if self.last_angle > 300 and a < 50:
          scan_complete = True

        if 0 <= a < 360:
          self.scan[a] = d
        self.last_angle = a

      if scan_complete:
        self.publish_scan()
        self.scan = [None] * 360

    if time.monotonic() - self.last_print > 1.0:
      points_in_scan = sum(1 for x in self.scan if x is not None)
      self.get_logger().debug(
        f"Buf:{len(self.buf):3d} Pkts:{self.packets_processed} "
        f"RPM:{self.rpm:5.1f} Points:{points_in_scan:3d} Last:{self.last_angle}"
      )
      self.last_print = time.monotonic()

  def publish_scan(self):
    msg = LaserScan()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.FRAME_ID

    msg.angle_min = 0.0
    msg.angle_max = 2 * math.pi
    msg.angle_increment = math.radians(1.0)

    msg.range_min = self.MIN_DIST_MM / 1000.0
    msg.range_max = self.MAX_DIST_MM / 1000.0

    msg.scan_time = 60.0 / self.rpm if self.rpm > 10 else 0.1
    msg.time_increment = msg.scan_time / 360.0 if msg.scan_time > 0 else 0.0

    # Build raw ranges (index = raw CW angle from sensor)
    raw = []
    valid = 0
    for i in range(360):
      d = self.scan[i]
      if d is not None and self.MIN_DIST_MM <= d <= self.MAX_DIST_MM:
        raw.append(float(d) / 1000.0)
        valid += 1
      else:
        raw.append(float('inf'))

    # Step 1: reverse to convert CW → CCW (mirrors the scan direction)
    # Step 2: rotate by offset to align robot forward = index 0
    mirrored = raw[::-1]
    msg.ranges = mirrored[self.OFFSET:] + mirrored[:self.OFFSET]

    self.scan_pub.publish(msg)
    self.get_logger().info(f"Published scan - {valid} valid points @ {self.rpm:.1f} RPM")

  def destroy_node(self):
    self.get_logger().info("Shutting down LiDAR node...")
    if hasattr(self, 'pi') and self.pi.connected:
      self.pi.set_PWM_dutycycle(self.PWM_PIN, 0)
      time.sleep(0.05)
      self.pi.write(self.PWM_PIN, 0)
      self.pi.stop()
    if hasattr(self, 'ser'):
      self.ser.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = LidarNode()
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