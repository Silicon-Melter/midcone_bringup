#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus, State

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge_node')
        # Listen to the VIO output
        self.sub = self.create_subscription(Odometry, '/mavros/odometry/out', self.cb, 10)
        # Talk to PX4
        self.pub = self.create_publisher(CompanionProcessStatus, '/mavros/companion_process/status', 1)
        self.last_msg = self.get_clock().now()
        self.create_timer(0.5, self.timer_cb) # 2Hz Heartbeat

    def cb(self, msg):
        self.last_msg = self.get_clock().now()

    def timer_cb(self):
        msg = CompanionProcessStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component = 197  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        
        # Check if data is fresh (within last 0.5s)
        if (self.get_clock().now() - self.last_msg).nanoseconds / 1e9 < 0.5:
            msg.state = 3 # MAV_STATE_ACTIVE
        else:
            msg.state = 5 # MAV_STATE_CRITICAL
            
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VIOBridge())
    rclpy.shutdown()