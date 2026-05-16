#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Quaternion
import math

def quaternion_to_yaw(orientation: Quaternion):
  # Extract quaternion components
  x = orientation.x
  y = orientation.y
  z = orientation.z
  w = orientation.w

  # Calculate Euler angles (yaw)
  t0 = +2.0 * (w * z + x * y)
  t1 = +1.0 - 2.0 * (y * y + z * z)
  yaw = math.atan2(t0, t1)
  
  return yaw

class HeadingNode(Node):
  def __init__(self):
    super().__init__('heading_node')
    self.subscription = self.create_subscription(Odometry, '/odometry/global', self.odometry_callback,10  )
    self.publisher = self.create_publisher(Float64, '/heading', 10)

  def odometry_callback(self, msg: Odometry):
    # Extract quaternion orientation from the Odometry message
    orientation = msg.pose.pose.orientation
    heading = quaternion_to_yaw(orientation)   
    heading_degrees = math.degrees(heading)     
    self.get_logger().info(f"Published heading: {heading:.4f} radians ({heading_degrees:.2f} degrees)")      
    heading_msg = Float64()
    heading_msg.data = heading
    # header = Header()
    # header.stamp = self.get_clock().now().to_msg()
    # header.frame_id = msg.header.frame_id
    self.publisher.publish(heading_msg)

def main(args=None):
  rclpy.init(args=args)
  heading_node = HeadingNode()
  rclpy.spin(heading_node)
  heading_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()







