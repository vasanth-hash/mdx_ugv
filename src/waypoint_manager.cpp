#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class WaypointManager : public rclcpp::Node
{
public:
  using NavAction = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  WaypointManager() : Node("waypoint_manager")
  {
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&WaypointManager::goalCb, this, _1));

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/gcs/start_mission", 10,
      std::bind(&WaypointManager::startCb, this, _1));

    pause_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/gcs/pause_mission", 10,
      std::bind(&WaypointManager::pauseCb, this, _1));

    rtb_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/gcs/rtb", 10,
      std::bind(&WaypointManager::rtbCb, this, _1));

    // Add odom subscriber to check if robot is moving
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&WaypointManager::odomCb, this, _1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/interpolated_waypoints", 10);

    nav_client_ = rclcpp_action::create_client<NavAction>(
      this, "navigate_through_poses");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create cmd_vel publisher upfront
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    interpolation_res_ = 3.0;

    RCLCPP_INFO(get_logger(), "Waypoint Manager READY | Waiting for robot pose...");
  }

private:
  /* ================= ROS ================= */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr rtb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr active_goal_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* ================= STATE ================= */
  std::vector<geometry_msgs::msg::PoseStamped> raw_goals_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;     // Single path for Nav2
  std::vector<geometry_msgs::msg::PoseStamped> rtb_path_; // RTB path

  // Store the robot’s pose at the start of the first mission
  geometry_msgs::msg::PoseStamped rtb_goal_pose_;
  bool rtb_goal_set_{false};
  bool rtb_mode_{false};
  bool cancel_for_rtb_{false};

  bool mission_active_{false};
  bool paused_{false};
  bool is_odom_stale_{true};
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  size_t last_complete_index_{0}; // Index from which to start next mission

  double interpolation_res_;

  /* ================= UTIL for motion check ================= */
  bool isRobotMoving(double linear_thresh = 0.01, double angular_thresh = 0.01)
  {
    if (!last_odom_ || is_odom_stale_) {
      return true;  // Assume moving if we don't know
    }

    double v = std::abs(last_odom_->twist.twist.linear.x);
    double w = std::abs(last_odom_->twist.twist.angular.z);

    return (v > linear_thresh) || (w > angular_thresh);
  }

  void publishStopCommand()
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;

    // Send multiple times to ensure it is received
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(stop);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
  }

  /* ================= CALLBACKS ================= */
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
    is_odom_stale_ = false;
  }

  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    raw_goals_.push_back(*msg);
    RCLCPP_INFO(get_logger(), "Waypoints: %zu | Latest: %.2f, %.2f",
                raw_goals_.size(),
                msg->pose.position.x, msg->pose.position.y);
    recomputePath();
    publishMarkers();
  }

  void startCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (mission_active_) {
      RCLCPP_WARN(get_logger(), "Mission already active");
      return;
    }

    if (raw_goals_.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints for mission");
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot pose");
      return;
    }

    // ✅ Only store rtb_goal_pose_ once, at the very first mission start
    if (!rtb_goal_set_) {
      rtb_goal_pose_ = robot_pose;
      rtb_goal_pose_.header.frame_id = "map";
      rtb_goal_pose_.header.stamp = get_clock()->now();
      rtb_goal_pose_.pose.position.z = 0.0;

      RCLCPP_INFO(get_logger(), "✅ RTB goal set to current robot pose: (%.2f, %.2f)",
                  rtb_goal_pose_.pose.position.x, rtb_goal_pose_.pose.position.y);
      rtb_goal_set_ = true;
    }

    size_t start_idx = last_complete_index_;
    if (start_idx >= raw_goals_.size()) {
      RCLCPP_WARN(get_logger(), "No new goals beyond last complete index %zu", last_complete_index_);
      return;
    }

    // Always use current robot pose as the starting point
    std::vector<geometry_msgs::msg::PoseStamped> current_path;

    // Robot → first waypoint in this segment
    interpolate(robot_pose, raw_goals_[start_idx], current_path);

    // Add path from start_idx to end
    for (size_t i = start_idx + 1; i < raw_goals_.size(); i++) {
      interpolate(raw_goals_[i-1], raw_goals_[i], current_path);
    }

    RCLCPP_INFO(get_logger(), "🚀 START_MISSION | Current: %.2f, %.2f → goal: %.2f, %.2f | Total: %zu",
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                raw_goals_[start_idx].pose.position.x, raw_goals_[start_idx].pose.position.y,
                current_path.size());

    path_ = current_path;
    sendPath(current_path, false);
    mission_active_ = true;
    paused_ = false;

    RCLCPP_INFO(get_logger(), "MISSION STARTED / RESUMED");
  }

  void pauseCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!mission_active_) {
      RCLCPP_WARN(get_logger(), "Pause requested but no active mission");
      return;
    }

    if (paused_) {
      RCLCPP_WARN(get_logger(), "Mission already paused");
      return;
    }

    nav_client_->async_cancel_goal(active_goal_);
    paused_ = true;

    RCLCPP_WARN(get_logger(), "MISSION PAUSED");
  }

  /* ================= RTB (REVERSE → SPAWN) ================= */
  void rtbCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!rtb_goal_set_) {
      RCLCPP_WARN(get_logger(), "RTB: RTB goal not set yet, cannot return to start");
      return;
    }

    if (raw_goals_.empty()) {
      RCLCPP_WARN(get_logger(), "RTB requested but no waypoints defined");
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot pose for RTB");
      return;
    }

    double dist_to_goal = std::hypot(
      robot_pose.pose.position.x - rtb_goal_pose_.pose.position.x,
      robot_pose.pose.position.y - rtb_goal_pose_.pose.position.y
    );
    if (dist_to_goal < 2.0) {
      RCLCPP_WARN(get_logger(), "RTB: Already near RTB goal (%.2f m), canceling", dist_to_goal);
      nav_client_->async_cancel_goal(active_goal_);
      mission_active_ = false;
      paused_ = false;
      publishStopCommand();
      return;
    }

    // Find closest waypoint
    size_t closest = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < raw_goals_.size(); ++i) {
      double dx = raw_goals_[i].pose.position.x - robot_pose.pose.position.x;
      double dy = raw_goals_[i].pose.position.y - robot_pose.pose.position.y;
      double d = std::hypot(dx, dy);
      if (d < best_dist) {
        best_dist = d;
        closest = i;
      }
    }

    RCLCPP_INFO(get_logger(), "RTB: Closest waypoint #%zu (%.2fm)", closest, best_dist);

    // Cancel active mission
    if (mission_active_ || paused_) {
      cancel_for_rtb_ = true;
      nav_client_->async_cancel_goal(active_goal_);
      mission_active_ = false;
      paused_ = false;
    }

    // Clear markers
    {
      visualization_msgs::msg::MarkerArray clear_arr;
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = "map";
      clear_marker.header.stamp = get_clock()->now();
      clear_marker.ns = "waypoints";
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_arr.markers.push_back(clear_marker);
      marker_pub_->publish(clear_arr);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    /* ===================== RTB VISUAL PATH (INTERPOLATED) ===================== */

    rtb_path_.clear();

    // 1️⃣ Robot → previous waypoint
    if (closest > 0) {
      interpolate(robot_pose, raw_goals_[closest - 1], rtb_path_);
    }

    // 2️⃣ Walk backwards
    for (int i = static_cast<int>(closest - 1); i > 0; --i) {
      interpolate(raw_goals_[i], raw_goals_[i - 1], rtb_path_);
    }

    // 3️⃣ First waypoint → spawn
    interpolate(raw_goals_[0], rtb_goal_pose_, rtb_path_);

    /* ===================== RTB NAV2 PATH (SPARSE) ===================== */

    path_.clear();

    // Previous waypoint (NOT closest)
    if (closest > 0) {
      path_.push_back(raw_goals_[closest - 1]);
    }

    // Walk backwards using RAW waypoints
    for (int i = static_cast<int>(closest - 1); i > 0; --i) {
      path_.push_back(raw_goals_[i - 1]);
    }

    // Final RTB goal
    path_.push_back(rtb_goal_pose_);

    RCLCPP_WARN(get_logger(),
                "✅ RTB path built (%zu waypoints) → RTB goal (%.2f, %.2f)",
                path_.size(),
                rtb_goal_pose_.pose.position.x,
                rtb_goal_pose_.pose.position.y);
    
    sendPath(path_, true);
    // Publish interpolated markers ONLY for visualization
    publishMarkers();
  }

  /* ================= PATH LOGIC ================= */
  void recomputePath()
  {
    path_.clear();
    if (raw_goals_.empty()) return;

    path_.push_back(raw_goals_[0]);

    for (size_t i = 1; i < raw_goals_.size(); i++) {
      interpolate(raw_goals_[i-1], raw_goals_[i], path_);
    }

    RCLCPP_INFO(get_logger(), "Path ready: %zu poses (starts at waypoint1)", path_.size());
  }

  // Interpolate between a and b and push to target_path
  void interpolate(const geometry_msgs::msg::PoseStamped &a,
                   const geometry_msgs::msg::PoseStamped &b,
                   std::vector<geometry_msgs::msg::PoseStamped> &target_path)
  {
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < 0.1) {
      geometry_msgs::msg::PoseStamped p = b;
      p.header.frame_id = "map";
      p.header.stamp = get_clock()->now();
      p.pose.position.z = 0.0;
      target_path.push_back(p);
      return;
    }

    int steps = std::max(1, static_cast<int>(dist / interpolation_res_));

    for (int i = 1; i <= steps; i++) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = get_clock()->now();
      double r = static_cast<double>(i) / steps;
      p.pose.position.x = a.pose.position.x + dx * r;
      p.pose.position.y = a.pose.position.y + dy * r;
      p.pose.position.z = 0.0;
      p.pose.orientation = b.pose.orientation;
      target_path.push_back(p);
    }
  }

  /* ================= INTERPOLATION MARKERS ================= */
  void interpolateMarker(const geometry_msgs::msg::PoseStamped &a,
                         const geometry_msgs::msg::PoseStamped &b,
                         visualization_msgs::msg::MarkerArray &arr,
                         uint32_t start_id)
  {
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    double dist = std::hypot(dx, dy);
    int steps = std::max(1, static_cast<int>(dist / interpolation_res_));

    for (int i = 1; i <= steps; i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = get_clock()->now();
      marker.ns = "waypoints";
      marker.id = start_id + i;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = a.pose.position.x + dx * i / steps;
      marker.pose.position.y = a.pose.position.y + dy * i / steps;
      marker.pose.position.z = 0.0;
      marker.pose.orientation = b.pose.orientation;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      marker.lifetime = rclcpp::Duration::from_seconds(0.0);
      marker.frame_locked = false;
      arr.markers.push_back(marker);
    }
  }

  /* ================= NAV2 ================= */
  void sendPath(const std::vector<geometry_msgs::msg::PoseStamped> & poses, bool is_rtb = false)
  {
    if (poses.empty()) {
      RCLCPP_WARN(get_logger(), "Empty path, not sending to Nav2");
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses action not available");
      return;
    }

    NavAction::Goal goal;
    goal.poses = poses;

    auto options = rclcpp_action::Client<NavAction>::SendGoalOptions();

    options.goal_response_callback = [this, poses](GoalHandle::SharedPtr handle)
    {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Path rejected by Nav2");
        // mission_active_ = false;
      } else {
        active_goal_ = handle;
        // mission_active_ = true;
        RCLCPP_INFO(get_logger(), "Nav2 accepted path (%zu poses)", poses.size());
      }
    };

    options.result_callback = [this, is_rtb](const GoalHandle::WrappedResult & result)
    {
      mission_active_ = false;
      paused_ = false;

      // if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      //   last_complete_index_ = raw_goals_.size();
      //   RCLCPP_INFO(get_logger(), "MISSION COMPLETED (final goal reached)");
      // } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      //   RCLCPP_WARN(get_logger(), "Mission canceled");
      // } else {
      //   RCLCPP_ERROR(get_logger(), "Mission failed");
      // }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (is_rtb) {
          RCLCPP_INFO(get_logger(), "RTB completed successfully");
          raw_goals_.clear();
          path_.clear();
          rtb_path_.clear();
          last_complete_index_ = 0;
          publishMarkers();
          return;
        } 
        last_complete_index_ = raw_goals_.size();
        RCLCPP_INFO(get_logger(), "MISSION COMPLETED (final goal reached)");
        
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        if (is_rtb) {
          RCLCPP_WARN(get_logger(), "RTB canceled");
          return;
        } else {
          RCLCPP_WARN(get_logger(), "Mission canceled");
          return;
        }
      } else {
        if (is_rtb) {
          RCLCPP_ERROR(get_logger(), "RTB failed");
        } else {
          RCLCPP_ERROR(get_logger(), "Mission failed");
        }
      }

      path_.clear();
      //rtb_path_.clear();
      publishMarkers();
    };

    nav_client_->async_send_goal(goal, options);
  }

    /* ================= UTIL ================= */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      pose.header.frame_id = "map";
      pose.header.stamp = get_clock()->now();
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Failed to get robot pose: %s", e.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Failed to get robot pose (unknown error)");
      return false;
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    geometry_msgs::msg::PoseStamped robot_pose;

    if (!getRobotPose(robot_pose)) {
      // Still publish clear message so markers are cleared
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = "map";
      clear_marker.header.stamp = get_clock()->now();
      clear_marker.ns = "waypoints";
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(clear_marker);
    }

    if (!getRobotPose(robot_pose)) {
      marker_pub_->publish(arr);
      return;
    }

    if (path_.empty()) {
      marker_pub_->publish(arr);
      return;
    }

    // Robot → first pose
    interpolateMarker(robot_pose, path_[0], arr, 1);

    // Path segments
    for (size_t i = 0; i + 1 < path_.size(); i++) {
      interpolateMarker(path_[i], path_[i + 1], arr, 1000 + i * 100);
    }

    marker_pub_->publish(arr);
  }   
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
