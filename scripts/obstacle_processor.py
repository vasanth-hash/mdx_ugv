#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, Point32, TwistWithCovariance
from std_msgs.msg import Header, ColorRGBA
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray

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


class ObstacleProcessor(Node):

    def __init__(self):
        super().__init__('obstacle_processor')
        self.get_logger().info("ObstacleProcessor initialized")

        # Publishers
        self.obstacle_publisher = self.create_publisher(
            ObstacleArrayMsg, '/obstacles', 10
        )
        self.trajectory_publisher = self.create_publisher(
            MarkerArray, '/obstacle_trajectories', 10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            PoseArray,
            '/person_pose_info',
            self.pose_callback,
            1
        )

        # State for velocity estimation
        self.prev_pose = None
        self.prev_time = None

        # Fixed obstacle ID (matches your SDF)
        self.obstacle_id = 4

    # ---------------------------------------------------

    def pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        pose = msg.poses[0]
        stamp = msg.header.stamp

        # ---------------- Velocity estimation ----------------
        velocity = TwistWithCovariance()

        if self.prev_pose is not None and self.prev_time is not None:
            dt = (
                (stamp.sec - self.prev_time.sec) +
                (stamp.nanosec - self.prev_time.nanosec) * 1e-9
            )

            if dt > 0.0:
                vx = (pose.position.x - self.prev_pose.position.x) / dt
                vy = (pose.position.y - self.prev_pose.position.y) / dt

                velocity.twist.linear.x = vx
                velocity.twist.linear.y = vy
                velocity.twist.angular.z = 0.0

        # Covariance (important for TEB prediction confidence)
        velocity.covariance[0] = 0.1    # vx
        velocity.covariance[7] = 0.1    # vy
        velocity.covariance[35] = 0.5   # yaw

        # Save for next iteration
        self.prev_pose = pose
        self.prev_time = stamp

        # ---------------- Obstacle message ----------------
        obstacle = ObstacleMsg()
        obstacle.header = Header(stamp=stamp, frame_id='map')
        obstacle.id = self.obstacle_id
        obstacle.radius = 1.0
        obstacle.orientation = pose.orientation
        obstacle.velocities = velocity

        obstacle.polygon.points = [
            Point32(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z
            )
        ]

        obstacle_array = ObstacleArrayMsg()
        obstacle_array.header = Header(stamp=stamp, frame_id='map')
        obstacle_array.obstacles.append(obstacle)

        # ---------------- Visualization ----------------
        markers = MarkerArray()
        markers.markers.append(
            self.create_trajectory_marker(pose, stamp)
        )

        # Publish
        self.obstacle_publisher.publish(obstacle_array)
        self.trajectory_publisher.publish(markers)

    # ---------------------------------------------------

    def create_trajectory_marker(self, pose, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'obstacle_trajectory'
        marker.id = self.obstacle_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker.pose.position = pose.position
        marker.pose.orientation = pose.orientation

        marker.lifetime = Duration(seconds=0.2).to_msg()
        return marker

# -------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProcessor()
    node = PoseArrayFrameIdAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Point32, TwistWithCovariance, Point
# from std_msgs.msg import Header, ColorRGBA
# from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
# from visualization_msgs.msg import Marker, MarkerArray
# from rclpy.duration import Duration

# class ObstacleProcessor(Node):
# 	def __init__(self):
# 		super().__init__('obstacle_processor')
# 		self.get_logger().info("Obstacle_processor initialized")

# 		self.obstacle_publisher = self.create_publisher(ObstacleArrayMsg, '/obstacles', 1)
# 		self.trajectory_publisher = self.create_publisher(MarkerArray, '/obstacle_trajectories', 1)

# 		self.subscription = self.create_subscription(PoseArray, '/person_pose_info_with_frame', self.pose_callback,1) # added frame id in the pose_frame_id_adder.py

# 	def pose_callback(self, msg: PoseArray):
# 		if not msg.poses:
# 			return

# 		pose = msg.poses[0]  # There is only one dynamic obstacle(standing person) added in the world 
# 		idx = 4              		# Known obstacle id which is in the sand_hieghtmap_boxes.sdf world
# 		msg_t_stamp = msg.header.stamp

# 		obstacle_arr_msg = ObstacleArrayMsg()
# 		obstacle_arr_msg.header = Header()
# 		obstacle_arr_msg.header.stamp = msg_t_stamp
# 		obstacle_arr_msg.header.frame_id = 'map'
# 		obstacle_arr_msg.obstacles = []

# 		trajectory_markers = MarkerArray()

# 		velocity = TwistWithCovariance()

# 		# Create obstacle message
# 		obstacle_msg = ObstacleMsg()
# 		obstacle_msg.header = Header()
# 		obstacle_msg.header.stamp = msg_t_stamp
# 		obstacle_msg.header.frame_id = 'map' # since map is the frame id all over used in nav2
# 		obstacle_msg.radius = self.calculate_radius(pose)
# 		obstacle_msg.id = idx
# 		obstacle_msg.orientation = pose.orientation
# 		obstacle_msg.velocities = velocity
# 		obstacle_msg.polygon.points = [
# 			Point32(x=pose.position.x, #- 2.0
# 					 y=pose.position.y,# + 2.5, 
# 					 z=pose.position.z)
# 		]

# 		obstacle_arr_msg.obstacles.append(obstacle_msg)

# 		trajectory_marker = self.publish_trajectory(pose, msg_t_stamp, 'map', idx)
# 		trajectory_markers.markers.append(trajectory_marker)

# 		self.obstacle_publisher.publish(obstacle_arr_msg)
# 		self.trajectory_publisher.publish(trajectory_markers)

# 	def calculate_radius(self, pose):
# 		return 1.0

# 	def publish_trajectory(self, pose, timestamp, frame_id, idx):
# 		marker = Marker()
# 		marker.header.frame_id = frame_id
# 		marker.header.stamp = timestamp
# 		marker.ns = "obstacle_trajectory"
# 		marker.id = idx
# 		marker.type = Marker.SPHERE
# 		marker.action = Marker.ADD
# 		marker.scale.x = 1.0
# 		marker.scale.y = 1.0
# 		marker.scale.z = 1.0
# 		marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
# 			# added this 5.6 because there is an offset between the map and world
# 			# (5.6 in x is where the robot is spawned in the world see the navigation_launch.py file)
# 		marker.pose.position.x = pose.position.x# -2.0 
# 		marker.pose.position.y = pose.position.y# + 2.5 
# 		marker.pose.position.z = pose.position.z
# 		marker.pose.orientation = pose.orientation  
# 		marker.lifetime = Duration(seconds=0.2).to_msg()
# 		return marker

# def main(args=None):
# 	rclpy.init(args=args)
# 	node = ObstacleProcessor()
# 	rclpy.spin(node)
# 	node.destroy_node()
# 	rclpy.shutdown()

# if __name__ == '__main__':
# 	main()

