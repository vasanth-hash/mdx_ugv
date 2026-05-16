#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import (
    PoseArray,
    Point32,
    Point,
    TwistWithCovariance
)
from std_msgs.msg import Header, ColorRGBA
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
import math


class DynamicObstacleProcessor(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_processor')
        self.get_logger().info("DynamicObstacleProcessor started")

        # ---------------------------------------------------
        # HARD-CODED world → map offset (robot spawn offset)
        # ---------------------------------------------------
        self.OFFSET_X = 29.0 # for car
        self.OFFSET_Y = 97.9
        # self.OFFSET_X = 0.0 # for person
        # self.OFFSET_Y = 0.0

        # ---------------------------------------------------
        # Known dynamic obstacle IDs (order = PoseArray order)
        # ---------------------------------------------------
        self.obstacle_ids = [4, 38]  # car, person
        self.obstacle_radius = {
            4: 2.0,   # car
            38: 0.6   # person
        }

        self.frame_id = "map"

        # ---------------------------------------------------
        # Publishers
        # ---------------------------------------------------
        self.obstacle_pub = self.create_publisher(
            ObstacleArrayMsg, "/obstacles", 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/obstacle_trajectories", 10
        )

        # ---------------------------------------------------
        # Subscriber
        # ---------------------------------------------------
        self.create_subscription(
            PoseArray,
            "person_pose_info",
            self.pose_callback,
            10
        )

        # ---------------------------------------------------
        # State for velocity estimation
        # ---------------------------------------------------
        self.prev_states = {}  # id -> (x, y, time)

    # ---------------------------------------------------

    def pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()

        obstacle_array = ObstacleArrayMsg()
        obstacle_array.header = Header(stamp=stamp, frame_id=self.frame_id)

        marker_array = MarkerArray()

        for idx, pose in enumerate(msg.poses):
            if idx >= len(self.obstacle_ids):
                continue

            obs_id = self.obstacle_ids[idx]

            # -------------------------------
            # Apply world → map offset
            # -------------------------------
            x = pose.position.x + self.OFFSET_X
            y = pose.position.y + self.OFFSET_Y

            # -------------------------------
            # Velocity estimation
            # -------------------------------
            vx, vy = 0.0, 0.0

            if obs_id in self.prev_states:
                px, py, pt = self.prev_states[obs_id]
                dt = (now - pt).nanoseconds * 1e-9
                if dt > 0.0:
                    vx = (x - px) / dt
                    vy = (y - py) / dt

            self.prev_states[obs_id] = (x, y, now)

            velocity = TwistWithCovariance()
            velocity.twist.linear.x = vx
            velocity.twist.linear.y = vy
            velocity.covariance[0] = 0.1
            velocity.covariance[7] = 0.1
            velocity.covariance[35] = 0.5

            # -------------------------------
            # Obstacle message (TEB)
            # -------------------------------
            obstacle = ObstacleMsg()
            obstacle.header = Header(stamp=stamp, frame_id=self.frame_id)
            obstacle.id = obs_id
            obstacle.radius = self.obstacle_radius.get(obs_id, 0.6)
            obstacle.orientation = pose.orientation
            obstacle.velocities = velocity

            obstacle.polygon.points.append(
                Point32(x=x, y=y, z=0.0)
            )

            obstacle_array.obstacles.append(obstacle)

            # -------------------------------
            # Sphere marker
            # -------------------------------
            marker_array.markers.append(
                self.create_sphere_marker(obs_id, x, y, stamp)
            )

            # -------------------------------
            # Velocity arrow marker
            # -------------------------------
            marker_array.markers.append(
                self.create_velocity_arrow(
                    obs_id, x, y, vx, vy, stamp
                )
            )

        self.obstacle_pub.publish(obstacle_array)
        self.marker_pub.publish(marker_array)

    # ---------------------------------------------------

    def create_sphere_marker(self, obs_id, x, y, stamp):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "dynamic_obstacles"
        m.id = obs_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        r = self.obstacle_radius.get(obs_id, 0.6)
        m.scale.x = r * 2.0
        m.scale.y = r * 2.0
        m.scale.z = 0.8

        if obs_id == 4:     # car
            m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        else:               # person
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.4

        m.lifetime = Duration(seconds=0.3).to_msg()
        return m

    # ---------------------------------------------------

    def create_velocity_arrow(self, obs_id, x, y, vx, vy, stamp):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "velocity_arrows"
        m.id = obs_id + 1000
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Constant arrow length (direction only)
        length = 1.5
        speed = math.hypot(vx, vy)

        if speed > 0.01:
            dx = (vx / speed) * length
            dy = (vy / speed) * length
        else:
            dx = dy = 0.0

        m.points = [
            Point(x=x, y=y, z=0.8),
            Point(x=x + dx, y=y + dy, z=0.8)
        ]

        m.scale.x = 0.15
        m.scale.y = 0.3
        m.scale.z = 0.3

        m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        m.lifetime = Duration(seconds=0.3).to_msg()
        return m


# ---------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

##----------------------------------------------------
# single dynamic obstacle id

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration

# from geometry_msgs.msg import PoseArray, Point32, TwistWithCovariance
# from std_msgs.msg import Header, ColorRGBA
# from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
# from visualization_msgs.msg import Marker, MarkerArray


# class DynamicObstacleProcessor(Node):

#     def __init__(self):
#         super().__init__('dynamic_obstacle_processor')
#         self.get_logger().info("DynamicObstacleProcessor initialized")

#         # ---- Publishers ----
#         self.obstacle_pub = self.create_publisher(
#             ObstacleArrayMsg, '/obstacles', 10
#         )

#         self.marker_pub = self.create_publisher(
#             MarkerArray, '/obstacle_trajectories', 10
#         )

#         # ---- Subscriber ----
#         self.subscription = self.create_subscription(
#             PoseArray,
#             'person_pose_info',   
#             self.pose_callback,
#             10
#         )

#         # ---- Velocity estimation state ----
#         self.prev_pose = None
#         self.prev_time = None

#         # Fixed obstacle ID (must be consistent)
#         self.obstacle_id = 4

#         # Fixed frame
#         self.frame_id = 'map'

#     # -----------------------------------------------------

#     def pose_callback(self, msg: PoseArray):
#         if not msg.poses:
#             return

#         pose = msg.poses[0]
#         stamp = self.get_clock().now().to_msg()

#         # ---------- Velocity estimation ----------
#         velocity = TwistWithCovariance()

#         if self.prev_pose is not None and self.prev_time is not None:
#             dt = (
#                 (stamp.sec - self.prev_time.sec) +
#                 (stamp.nanosec - self.prev_time.nanosec) * 1e-9
#             )

#             if dt > 0.0:
#                 vx = (pose.position.x - self.prev_pose.position.x) / dt
#                 vy = (pose.position.y - self.prev_pose.position.y) / dt

#                 velocity.twist.linear.x = vx
#                 velocity.twist.linear.y = vy
#                 velocity.twist.angular.z = 0.0

#         # Covariance (important for TEB dynamic obstacles)
#         velocity.covariance[0] = 0.1    # vx
#         velocity.covariance[7] = 0.1    # vy
#         velocity.covariance[35] = 0.5   # yaw

#         self.prev_pose = pose
#         self.prev_time = stamp

#         # ---------- Obstacle message ----------
#         obstacle = ObstacleMsg()
#         obstacle.header = Header(stamp=stamp, frame_id=self.frame_id)
#         obstacle.id = self.obstacle_id
#         obstacle.radius = 0.7
#         obstacle.orientation = pose.orientation
#         obstacle.velocities = velocity

#         obstacle.polygon.points = [
#             Point32(
#                 x=pose.position.x,
#                 y=pose.position.y,
#                 z=pose.position.z
#             )
#         ]

#         obstacle_array = ObstacleArrayMsg()
#         obstacle_array.header = Header(stamp=stamp, frame_id=self.frame_id)
#         obstacle_array.obstacles.append(obstacle)

#         # ---------- Visualization ----------
#         marker_array = MarkerArray()
#         marker_array.markers.append(
#             self.create_marker(pose, stamp)
#         )

#         # ---------- Publish ----------
#         self.obstacle_pub.publish(obstacle_array)
#         self.marker_pub.publish(marker_array)

#     # -----------------------------------------------------

#     def create_marker(self, pose, stamp):
#         marker = Marker()
#         marker.header.frame_id = self.frame_id
#         marker.header.stamp = stamp
#         marker.ns = "dynamic_obstacle"
#         marker.id = self.obstacle_id
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD

#         marker.scale.x = 1.0
#         marker.scale.y = 1.0
#         marker.scale.z = 1.0

#         marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

#         marker.pose.position = pose.position
#         marker.pose.orientation = pose.orientation

#         marker.lifetime = Duration(seconds=0.2).to_msg()
#         return marker


# # -----------------------------------------------------

# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamicObstacleProcessor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()