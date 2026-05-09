# sync_relay_node.py
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class SyncRelay(Node):
  def __init__(self):
    super().__init__('sync_relay')
    self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
    self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

    # rf2o publishes on internal topic, relay is the only /odom publisher
    odom_sub = Subscriber(self, Odometry, '/odom_rf2o')
    scan_sub = Subscriber(self, LaserScan, '/scan_raw')

    # slop can be very tight — rf2o stamps odom with the scan's stamp
    self.sync = ApproximateTimeSynchronizer(
        [odom_sub, scan_sub], queue_size=20, slop=0.005
    )
    self.sync.registerCallback(self.cb)

  def cb(self, odom: Odometry, scan: LaserScan):
    stamp = scan.header.stamp   # canonical timestamp
    odom.header.stamp = stamp
    scan.header.stamp = stamp
    self.odom_pub.publish(odom)
    self.scan_pub.publish(scan)


def main(args=None):
  rclpy.init(args=args)
  node = SyncRelay()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()