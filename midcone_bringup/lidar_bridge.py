#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class LIDARBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')

        # 1. Subscribe to your LiDAR driver
        # We use qos_profile_sensor_data because LiDARs usually publish "Best Effort"
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',            # The topic your LiDAR driver publishes to
            self.scan_cb,
            qos_profile_sensor_data
        )

        # 2. Setup Publisher to MAVROS
        # MAVROS converts this LaserScan into MAVLink OBSTACLE_DISTANCE messages
        self.pub = self.create_publisher(
            LaserScan,
            '/mavros/obstacle/send',
            10
        )
        
        self.get_logger().info("Bridge Started: Relaying /scan -> /mavros/obstacle/send")

    def scan_cb(self, msg):
        # Optional: You can filter or modify 'msg' here if needed
        # For now, we just pass it straight through
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LIDARBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()