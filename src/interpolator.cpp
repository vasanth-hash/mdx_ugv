#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>

class Interpolator : public rclcpp::Node
{
public:
  Interpolator() : Node("interpolator")
  {
    declare_parameter<double>("interpolate_distance", 2.0);
    /* command to set the interpolate distance
    ros2 param set /interpolator interpolate_distance 1.0
    */

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    std::bind(&Interpolator::planCallback, this, std::placeholders::_1));
    
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/interpolate_markers", 10);

    // path_pub_ = create_publisher<nav_msgs::msg::Path>(
    //   "/interpolated_path", 10);

    RCLCPP_INFO(get_logger(),
      "Interpolator started (sampling global plan)");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void planCallback(const nav_msgs::msg::Path::SharedPtr plan)
  {
    if (plan->poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Global plan too short to interpolate");
      return;
    }

    const double step =
      get_parameter("interpolate_distance").as_double();

    nav_msgs::msg::Path interp_path;
    interp_path.header = plan->header;

    visualization_msgs::msg::Marker marker;
    marker.header = plan->header;
    marker.ns = "interpolated_waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.1f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    geometry_msgs::msg::PoseStamped last_pose = plan->poses.front();
    interp_path.poses.push_back(last_pose);
    addMarkerPoint(marker, last_pose);

    double accumulated_dist = 0.0;

    for (size_t i = 1; i < plan->poses.size(); ++i)
    {
      const auto &curr = plan->poses[i];

      double dx = curr.pose.position.x - last_pose.pose.position.x;
      double dy = curr.pose.position.y - last_pose.pose.position.y;
      double segment_dist = std::hypot(dx, dy);

      accumulated_dist += segment_dist;

      if (accumulated_dist >= step) {
        interp_path.poses.push_back(curr);
        addMarkerPoint(marker, curr);
        accumulated_dist = 0.0;
      }

      last_pose = curr;
    }

    // Ensure final goal is included
    interp_path.poses.push_back(plan->poses.back());
    addMarkerPoint(marker, plan->poses.back());

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);

    marker_pub_->publish(markers);
    //path_pub_->publish(interp_path);

    RCLCPP_INFO(get_logger(),
      "Global plan sampled → %zu waypoints (spacing %.2f m)",
      interp_path.poses.size(), step);
  }

  void addMarkerPoint(
    visualization_msgs::msg::Marker &marker,
    const geometry_msgs::msg::PoseStamped &pose)
  {
    geometry_msgs::msg::Point p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = 0.2;
    marker.points.push_back(p);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Interpolator>());
  rclcpp::shutdown();
  return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/path.hpp>

// #include <visualization_msgs/msg/marker.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>

// #include <nav2_msgs/action/navigate_through_poses.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <cmath>

// class Interpolator : public rclcpp::Node
// {
// public:
//   using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
//   using GoalHandleNavThroughPoses =
//       rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

//   Interpolator() : Node("interpolator")
//   {
//     declare_parameter<double>("interpolate_distance", 2.0);

//     goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/goal_pose", 10,
//       std::bind(&Interpolator::goalCallback, this, std::placeholders::_1));

//     odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//       "/odometry/filtered/global", 10,
//       std::bind(&Interpolator::odomCallback, this, std::placeholders::_1));

//     costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
//       "/global_costmap/costmap", 10,
//       std::bind(&Interpolator::costmapCallback, this, std::placeholders::_1));

//     marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
//       "/interpolate_markers", 10);

//     path_pub_ = create_publisher<nav_msgs::msg::Path>(
//       "/interpolated_path", 10);

//     nav_through_poses_client_ =
//       rclcpp_action::create_client<NavigateThroughPoses>(
//         this, "navigate_through_poses");

//     RCLCPP_INFO(get_logger(),
//       "Interpolator started (waypoints + path + NavigateThroughPoses)");
//   }

// private:
//   // ---------------- ROS Interfaces ----------------
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

//   rclcpp_action::Client<NavigateThroughPoses>::SharedPtr
//       nav_through_poses_client_;

//   geometry_msgs::msg::Pose current_pose_;
//   nav_msgs::msg::OccupancyGrid costmap_;

//   bool pose_received_{false};
//   bool costmap_received_{false};

//   // ---------------- Callbacks ----------------

//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     current_pose_ = msg->pose.pose;
//     pose_received_ = true;
//   }

//   void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//   {
//     costmap_ = *msg;
//     costmap_received_ = true;
//   }

//   void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
//   {
//     if (!pose_received_ || !costmap_received_) {
//       RCLCPP_WARN(get_logger(), "Waiting for pose or costmap...");
//       return;
//     }

//     if (!nav_through_poses_client_->wait_for_action_server(
//           std::chrono::seconds(2))) {
//       RCLCPP_ERROR(get_logger(),
//         "NavigateThroughPoses action server not available");
//       return;
//     }

//     const double step = get_parameter("interpolate_distance").as_double();

//     const double sx = current_pose_.position.x;
//     const double sy = current_pose_.position.y;
//     const double gx = goal->pose.position.x;
//     const double gy = goal->pose.position.y;

//     const double dx = gx - sx;
//     const double dy = gy - sy;
//     const double dist = std::hypot(dx, dy);
//     const int steps = std::max(1, static_cast<int>(dist / step));

//     std::vector<geometry_msgs::msg::PoseStamped> waypoints;

//     nav_msgs::msg::Path path;
//     path.header = goal->header;

//     visualization_msgs::msg::MarkerArray markers;
//     visualization_msgs::msg::Marker marker;

//     marker.header = goal->header;
//     marker.ns = "interpolated_waypoints";
//     marker.id = 0;
//     marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//     marker.action = visualization_msgs::msg::Marker::ADD;

//     marker.scale.x = 0.3;
//     marker.scale.y = 0.3;
//     marker.scale.z = 0.3;

//     marker.color.r = 0.0f;
//     marker.color.g = 0.4f;
//     marker.color.b = 1.0f;
//     marker.color.a = 1.0f;

//     for (int i = 0; i <= steps; ++i)
//     {
//       double ratio = static_cast<double>(i) / steps;
//       double x = sx + ratio * dx;
//       double y = sy + ratio * dy;

//       if (isLethal(x, y)) {
//         if (!shiftToFreeCell(x, y)) {
//           continue;
//         }
//       }

//       geometry_msgs::msg::PoseStamped pose;
//       pose.header = goal->header;
//       pose.pose.position.x = x;
//       pose.pose.position.y = y;
//       // pose.pose.orientation = goal->pose.orientation;
//       double yaw = std::atan2(dy, dx);
//       tf2::Quaternion q;
//       q.setRPY(0, 0, yaw);
//       pose.pose.orientation = tf2::toMsg(q);

//       waypoints.push_back(pose);
//       path.poses.push_back(pose);

//       geometry_msgs::msg::Point p;
//       p.x = x;
//       p.y = y;
//       p.z = 0.2;
//       marker.points.push_back(p);
//     }

//     markers.markers.push_back(marker);

//     // ---- Publish visualization ----
//     marker_pub_->publish(markers);
//     path_pub_->publish(path);

//     // ---- Send to Nav2 ----
//     NavigateThroughPoses::Goal nav_goal;
//     nav_goal.poses = waypoints;

//     auto options =
//       rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

//     nav_through_poses_client_->async_send_goal(nav_goal, options);

//     RCLCPP_INFO(get_logger(),
//       "Published interpolated path (%zu poses) and sent waypoints to Nav2",
//       path.poses.size());
//   }

//   // ---------------- Costmap Helpers ----------------

//   bool isLethal(double wx, double wy)
//   {
//     const auto &info = costmap_.info;

//     int mx = static_cast<int>(
//       (wx - info.origin.position.x) / info.resolution);
//     int my = static_cast<int>(
//       (wy - info.origin.position.y) / info.resolution);

//     if (mx < 0 || my < 0 ||
//         mx >= static_cast<int>(info.width) ||
//         my >= static_cast<int>(info.height)) {
//       return true;
//     }

//     int index = my * info.width + mx;
//     return costmap_.data[index] >= 100;
//   }

//   bool shiftToFreeCell(double &x, double &y)
//   {
//     const double radius = 0.6;
//     const int samples = 16;

//     for (int i = 0; i < samples; ++i) {
//       double angle = i * 2.0 * M_PI / samples;
//       double nx = x + radius * std::cos(angle);
//       double ny = y + radius * std::sin(angle);

//       if (!isLethal(nx, ny)) {
//         x = nx;
//         y = ny;
//         return true;
//       }
//     }
//     return false;
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Interpolator>());
//   rclcpp::shutdown();
//   return 0;
// }



