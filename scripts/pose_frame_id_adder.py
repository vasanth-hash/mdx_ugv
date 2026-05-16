#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class PoseArrayFrameIdAdder(Node):
	def __init__(self):
		super().__init__('pose_array_frame_id_adder')
		self.subscription = self.create_subscription(PoseArray,'person_pose_info',self.pose_callback,10)
		self.publisher = self.create_publisher(PoseArray,'person_pose_info_with_frame',10)
		self.frame_id = 'map' 

	def pose_callback(self, msg):
		new_msg = PoseArray()
		new_msg.header = msg.header
		new_msg.header.stamp = self.get_clock().now().to_msg()
		new_msg.header.frame_id = self.frame_id  
		new_msg.poses = msg.poses 

		self.publisher.publish(new_msg)

def main(args=None):
	rclpy.init(args=args)
	node = PoseArrayFrameIdAdder()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()