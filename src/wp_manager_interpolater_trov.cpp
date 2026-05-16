// wp_manager_interpolater.cpp - CORRECTED VERSION
// Fixes:
// 1. TF timestamp issue (use Time(0) everywhere)
// 2. Infinite retry on abort (until manual stop)
// 3. MIN_GOAL_DIST reduced to 1.0m
// 4. Smart abort detection (check if actually at goal)
// 5. HYBRID collision mode for dynamic obstacles

#include <vector>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WaypointManagerInterpolater : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowClient = rclcpp_action::Client<FollowPath>;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowPath>;

  WaypointManagerInterpolater()
  : Node("waypoint_manager"),
    interpolation_res_(3.0),
    mission_active_(false),
    rtb_mode_(false),
    gps_printed_(false),
    path_total_length_(0.0),
    last_goal_accepted_time_(rclcpp::Time(0)),
    received_first_feedback_(false),
    retry_count_(0),
    user_stopped_(false),  // Track manual stop
    current_goal_index_(0)  // Start at first goal
  {
    // Subscriptions
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", 1,
        std::bind(&WaypointManagerInterpolater::costmapCb, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&WaypointManagerInterpolater::goalCb, this, _1));

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/start_navigation", 10,
        std::bind(&WaypointManagerInterpolater::startCb, this, _1));

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/stop_navigation", 10,
        std::bind(&WaypointManagerInterpolater::stopCb, this, _1));

    rtb_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/rtb", 10,
        std::bind(&WaypointManagerInterpolater::rtbCb, this, _1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsat/fix", 10,
        std::bind(&WaypointManagerInterpolater::gpsCb, this, _1));

    // Publishers
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/interpolated_waypoints", 10);

    // ✅ RViz Nav2 display compatibility
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);
    global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_plan", 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_plan", 10);

    // ✅ Action client for FollowPath
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "✅ WAYPOINT MANAGER INITIALIZED (CORRECTED VERSION)");
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "   🔧 FIXES APPLIED:");
    RCLCPP_INFO(get_logger(), "   - TF timestamp fix (Time(0) for latest transforms)");
    RCLCPP_INFO(get_logger(), "   - Infinite retry on abort (until manual stop)");
    RCLCPP_INFO(get_logger(), "   - MIN_GOAL_DIST reduced to 1.0m");
    RCLCPP_INFO(get_logger(), "   - Smart abort detection");
    RCLCPP_INFO(get_logger(), "   - HYBRID collision mode (dynamic obstacles)");
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "   Configuration:");
    RCLCPP_INFO(get_logger(), "   - Interpolation: %.2f meters", interpolation_res_);
    RCLCPP_INFO(get_logger(), "   - Min goal distance: 1.0m");
    RCLCPP_INFO(get_logger(), "   - Retry policy: INFINITE (until /stop_navigation)");
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "   Path visualization topics:");
    RCLCPP_INFO(get_logger(), "   - /plan (RViz Nav2 display)");
    RCLCPP_INFO(get_logger(), "   - /global_plan (redundant)");
    RCLCPP_INFO(get_logger(), "   - /local_plan (current segment)");
    RCLCPP_INFO(get_logger(), "   - /interpolated_waypoints (markers)");
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "   Usage:");
    RCLCPP_INFO(get_logger(), "   1. Add goals via /goal_pose");
    RCLCPP_INFO(get_logger(), "   2. Start with /start_navigation");
    RCLCPP_INFO(get_logger(), "   3. RTB via /rtb (returns through waypoints)");
    RCLCPP_INFO(get_logger(), "   4. Stop with /stop_navigation (stops retries)");
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");
  }

private:
  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_, stop_sub_, rtb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap latest_costmap_;
  std::mutex costmap_mutex_;
  bool costmap_received_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Action client
  FollowClient::SharedPtr follow_client_;
  GoalHandleFollow::SharedPtr current_goal_handle_{nullptr};

  // Goal management
  std::vector<geometry_msgs::msg::PoseStamped> goal_points_;            // permanent store (forward mission)
  std::vector<geometry_msgs::msg::PoseStamped> active_mission_goals_;   // current mission goals
  std::vector<geometry_msgs::msg::PoseStamped> interpolated_path_;     // full interpolated path

  double interpolation_res_;
  bool mission_active_;
  bool rtb_mode_;

  double latitude_{0.0}, longitude_{0.0};
  bool gps_printed_;

  // Track path metrics
  double path_total_length_;
  rclcpp::Time last_goal_accepted_time_;
  bool received_first_feedback_;

  // ✅ NEW: Retry management
  int retry_count_;
  bool user_stopped_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // ✅ NEW: Sequential waypoint tracking
  size_t current_goal_index_;  // Track which goal we're currently navigating to

  // ═══════════════════════════════════════════════════════════════════
  // HELPER: CREATE TURN-IN-PLACE ANCHOR
  // ═══════════════════════════════════════════════════════════════════
  geometry_msgs::msg::PoseStamped makeTurnAnchor(
      const geometry_msgs::msg::PoseStamped &robot,
      const geometry_msgs::msg::PoseStamped &next)
  {
    geometry_msgs::msg::PoseStamped anchor = robot;
    
    // Compute yaw toward next waypoint
    double dx = next.pose.position.x - robot.pose.position.x;
    double dy = next.pose.position.y - robot.pose.position.y;
    double yaw = std::atan2(dy, dx);

    // Set orientation toward target
    anchor.pose.orientation.x = 0.0;
    anchor.pose.orientation.y = 0.0;
    anchor.pose.orientation.z = std::sin(yaw * 0.5);
    anchor.pose.orientation.w = std::cos(yaw * 0.5);
    anchor.header.frame_id = "map";
    anchor.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
    
    return anchor;
  }

  // ═══════════════════════════════════════════════════════════════════
  // HELPER: Calculate distance between poses
  // ═══════════════════════════════════════════════════════════════════
  double distance(const geometry_msgs::msg::PoseStamped &a,
                  const geometry_msgs::msg::PoseStamped &b)
  {
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    return std::hypot(dx, dy);
  }

  // ═══════════════════════════════════════════════════════════════════
  // GOAL CALLBACK - Store goals from RViz
  // ═══════════════════════════════════════════════════════════════════
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped p = *msg;
    p.header.frame_id = "map";
    p.pose.position.z = 0.0;
    
    // Ensure valid orientation
    if (p.pose.orientation.w == 0.0 && p.pose.orientation.x == 0.0 &&
        p.pose.orientation.y == 0.0 && p.pose.orientation.z == 0.0)
    {
      p.pose.orientation.w = 1.0;
    }

    // Only store in goal_points_ if NOT in RTB mode
    if (!rtb_mode_) {
      goal_points_.push_back(p);
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "📍 NEW FORWARD GOAL RECEIVED (#%zu)", goal_points_.size());
      RCLCPP_INFO(get_logger(), "   Position: (%.3f, %.3f)", p.pose.position.x, p.pose.position.y);
      RCLCPP_INFO(get_logger(), "   Total stored: %zu", goal_points_.size());
    } else {
      RCLCPP_WARN(get_logger(), "⚠️  Goal received during RTB mode - IGNORING");
      RCLCPP_WARN(get_logger(), "   Stop RTB first to add new goals");
      return;
    }

    active_mission_goals_.push_back(p);
    publishMarkers();
    publishPathVisualization();
  }

  // ═══════════════════════════════════════════════════════════════════
  // START CALLBACK - Build path and send to Nav2 (SEGMENT BY SEGMENT)
  // ═══════════════════════════════════════════════════════════════════
  void startCb(const std_msgs::msg::Empty::SharedPtr)
  {
    // ✅ Reset user_stopped flag when starting
    user_stopped_ = false;
    
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "🚀 START NAVIGATION TRIGGERED");
    if (retry_count_ > 0) {
      RCLCPP_INFO(get_logger(), "   (Retry attempt #%d)", retry_count_);
    }
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");

    if (active_mission_goals_.empty()) {
      if (rtb_mode_) {
        RCLCPP_INFO(get_logger(), "✅ RTB already complete - at destination");
      } else {
        RCLCPP_WARN(get_logger(), "⚠️  No active goals! Add goals via /goal_pose first");
      }
      rtb_mode_ = false;
      mission_active_ = false;
      retry_count_ = 0;
      current_goal_index_ = 0;
      return;
    }

    // Cancel existing action if running
    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "⏹️  Canceling existing FollowPath action...");
      follow_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
      rclcpp::sleep_for(200ms);
    }

    // Get robot position
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_ERROR(get_logger(), "❌ CANNOT GET ROBOT POSE - Navigation aborted");
      
      // ✅ Retry on TF failure too
      if (!user_stopped_) {
        RCLCPP_WARN(get_logger(), "   Will retry in 3 seconds...");
        scheduleRetry(3.0);
      }
      return;
    }

    RCLCPP_INFO(get_logger(), "📊 Mission Info:");
    RCLCPP_INFO(get_logger(), "   Type: %s", rtb_mode_ ? "🛬 RTB (Return to Base)" : "➡️  FORWARD");
    RCLCPP_INFO(get_logger(), "   Robot at: (%.2f, %.2f)", robot_pose.pose.position.x, robot_pose.pose.position.y);
    RCLCPP_INFO(get_logger(), "   Goals stored: %zu", goal_points_.size());
    RCLCPP_INFO(get_logger(), "   Active goals: %zu", active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "   Current goal index: %zu/%zu", current_goal_index_ + 1, active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "   Interpolation: %.2f m spacing", interpolation_res_);

    // ✅ SEGMENT-BY-SEGMENT: Only navigate to current goal
    if (current_goal_index_ >= active_mission_goals_.size()) {
      RCLCPP_INFO(get_logger(), "✅ All segments completed!");
      current_goal_index_ = 0;
      active_mission_goals_.clear();
      rtb_mode_ = false;
      mission_active_ = false;
      retry_count_ = 0;
      return;
    }

    // Build interpolated path for CURRENT SEGMENT ONLY
    interpolated_path_.clear();

    // ✅ RTB anchor: force turn-in-place at robot position (only for first segment)
    if (rtb_mode_ && current_goal_index_ == 0 && !active_mission_goals_.empty()) {
      auto anchor = makeTurnAnchor(robot_pose, active_mission_goals_[current_goal_index_]);
      interpolated_path_.push_back(anchor);
      RCLCPP_WARN(get_logger(), "");
      RCLCPP_WARN(get_logger(), "🔁 RTB MODE: Inserted turn-in-place anchor");
      RCLCPP_WARN(get_logger(), "   Anchor position: (%.2f, %.2f)", anchor.pose.position.x, anchor.pose.position.y);
      RCLCPP_WARN(get_logger(), "   Target heading: %.2f rad toward first RTB waypoint", 
                  std::atan2(active_mission_goals_[current_goal_index_].pose.position.y - robot_pose.pose.position.y,
                            active_mission_goals_[current_goal_index_].pose.position.x - robot_pose.pose.position.x));
    }

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "🔧 Building path for segment %zu/%zu:", 
                current_goal_index_ + 1, active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────");

    // Interpolate from robot to current goal only
    geometry_msgs::msg::PoseStamped current = robot_pose;
    if (!interpolated_path_.empty()) {
      // Start from anchor if we added one
      current = interpolated_path_.back();
    }

    double dx = active_mission_goals_[current_goal_index_].pose.position.x - current.pose.position.x;
    double dy = active_mission_goals_[current_goal_index_].pose.position.y - current.pose.position.y;
    double segment_dist = std::hypot(dx, dy);

    RCLCPP_INFO(get_logger(), "   From: (%.2f, %.2f)", current.pose.position.x, current.pose.position.y);
    RCLCPP_INFO(get_logger(), "   To:   (%.2f, %.2f)", 
                active_mission_goals_[current_goal_index_].pose.position.x, 
                active_mission_goals_[current_goal_index_].pose.position.y);
    RCLCPP_INFO(get_logger(), "   Distance: %.2f m", segment_dist);

    size_t before = interpolated_path_.size();
    interpolate(current, active_mission_goals_[current_goal_index_], interpolated_path_);
    size_t after = interpolated_path_.size();
    
    RCLCPP_INFO(get_logger(), "   ✅ Added %zu interpolated points", (after - before));

    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────");

    if (interpolated_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ INTERPOLATION FAILED - No valid path for this segment!");
      RCLCPP_ERROR(get_logger(), "   Points may be in collision or too close");
      
      // ✅ Retry on interpolation failure
      if (!user_stopped_) {
        RCLCPP_WARN(get_logger(), "   Will retry in 5 seconds...");
        scheduleRetry(5.0);
      }
      return;
    }

    // Calculate segment length
    path_total_length_ = 0.0;
    for (size_t i = 1; i < interpolated_path_.size(); ++i) {
      double dx = interpolated_path_[i].pose.position.x - interpolated_path_[i-1].pose.position.x;
      double dy = interpolated_path_[i].pose.position.y - interpolated_path_[i-1].pose.position.y;
      path_total_length_ += std::hypot(dx, dy);
    }

    // ✅ CRITICAL FIX: Ensure ALL poses have Time(0) timestamp
    for (auto &ps : interpolated_path_) {
      ps.header.frame_id = "map";
      ps.header.stamp = rclcpp::Time(0);  // Use latest TF!
    }

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "✅ SEGMENT PATH GENERATION COMPLETE:");
    RCLCPP_INFO(get_logger(), "   Segment: %zu/%zu", current_goal_index_ + 1, active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "   Waypoints in segment: %zu", interpolated_path_.size());
    RCLCPP_INFO(get_logger(), "   Segment length: %.2f meters", path_total_length_);

    // Build nav_msgs::Path for current segment
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = rclcpp::Time(0);  // ✅ CRITICAL FIX: Use Time(0)!
    path_msg.poses = interpolated_path_;

    // Publish for RViz visualization
    path_pub_->publish(path_msg);
    global_path_pub_->publish(path_msg);
    
    RCLCPP_INFO(get_logger(), "📍 Segment path published to /plan and /global_plan");

    publishMarkers();

    // Send to Nav2
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "📤 Sending segment %zu/%zu to Nav2 FollowPath action...", 
                current_goal_index_ + 1, active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");
    
    sendPathToNav2(path_msg);
    mission_active_ = true;
  }

  // ═══════════════════════════════════════════════════════════════════
  // STOP CALLBACK - Cancel navigation
  // ═══════════════════════════════════════════════════════════════════
  void stopCb(const std_msgs::msg::Empty::SharedPtr)
  {
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_WARN(get_logger(), "════════════════════════════════════════════════════════");
    RCLCPP_WARN(get_logger(), "🔴 STOP NAVIGATION TRIGGERED (Manual Stop)");
    RCLCPP_WARN(get_logger(), "════════════════════════════════════════════════════════");

    // ✅ Set flag to prevent retries
    user_stopped_ = true;
    
    // Cancel retry timer if active
    if (retry_timer_) {
      retry_timer_->cancel();
      retry_timer_.reset();
    }

    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "⏹️  Canceling FollowPath action...");
      follow_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
      rclcpp::sleep_for(200ms);
    }

    // Publish empty path to clear visualization
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
    path_pub_->publish(empty);
    global_path_pub_->publish(empty);

    mission_active_ = false;
    interpolated_path_.clear();
    retry_count_ = 0;  // Reset retry counter
    current_goal_index_ = 0;  // Reset goal index

    RCLCPP_INFO(get_logger(), "📊 Mission State:");
    RCLCPP_INFO(get_logger(), "   Goals stored: %zu", goal_points_.size());
    RCLCPP_INFO(get_logger(), "   Active remaining: %zu", active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "   Mode: %s", rtb_mode_ ? "RTB" : "NORMAL");
    RCLCPP_INFO(get_logger(), "   Retries stopped: YES");
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "💡 Use /start_navigation to resume");
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");
    
    publishMarkers();
  }

  // ═══════════════════════════════════════════════════════════════════
  // RTB CALLBACK - Return to base through stored waypoints
  // ═══════════════════════════════════════════════════════════════════
  void rtbCb(const std_msgs::msg::Empty::SharedPtr)
  {
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_WARN(get_logger(), "════════════════════════════════════════════════════════");
    RCLCPP_WARN(get_logger(), "🛬 RTB (RETURN TO BASE) TRIGGERED");
    RCLCPP_WARN(get_logger(), "════════════════════════════════════════════════════════");

    // ✅ Reset user_stopped flag for RTB
    user_stopped_ = false;
    retry_count_ = 0;
    current_goal_index_ = 0;  // Start from first RTB goal

    // Cancel current action
    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "⏹️  Canceling current FollowPath action...");
      follow_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
      rclcpp::sleep_for(200ms);
    }

    // Get robot position
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_ERROR(get_logger(), "❌ RTB FAILED: Cannot get robot pose!");
      return;
    }

    RCLCPP_INFO(get_logger(), "🤖 Current robot: (%.2f, %.2f)", 
                robot_pose.pose.position.x, robot_pose.pose.position.y);
    RCLCPP_INFO(get_logger(), "📊 Forward goals stored: %zu", goal_points_.size());

    if (goal_points_.empty()) {
      RCLCPP_WARN(get_logger(), "⚠️  No stored waypoints for RTB!");
      RCLCPP_WARN(get_logger(), "   Robot will only return to [0,0]");
    }

    // ✅ CRITICAL: Clear active goals BEFORE building RTB path
    active_mission_goals_.clear();

    // Build REVERSE path through stored waypoints
    const double MIN_GOAL_DIST = 1.0;  // ✅ Reduced from 2.5m to 1.0m

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "🔧 Building RTB path (reverse order):");
    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────");

    // Traverse stored goals in reverse
    for (int i = static_cast<int>(goal_points_.size()) - 1; i >= 0; --i) {
      auto p = goal_points_[i];
      p.header.frame_id = "map";
      p.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
      p.pose.position.z = 0.0;
      
      // Ensure valid orientation
      if (p.pose.orientation.w == 0.0) {
        p.pose.orientation.w = 1.0;
      }

      double dist = distance(p, robot_pose);
      bool collision_free = isPoseCollisionFree(p);

      RCLCPP_INFO(get_logger(), "   Waypoint #%d:", i + 1);
      RCLCPP_INFO(get_logger(), "     Position: (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
      RCLCPP_INFO(get_logger(), "     Distance from robot: %.2f m", dist);
      RCLCPP_INFO(get_logger(), "     Collision check: %s", collision_free ? "FREE" : "BLOCKED");

      if (dist > MIN_GOAL_DIST && collision_free) {
        active_mission_goals_.push_back(p);
        RCLCPP_INFO(get_logger(), "     ✅ ADDED to RTB path");
      } else if (dist <= MIN_GOAL_DIST) {
        RCLCPP_INFO(get_logger(), "     ⏭️  SKIPPED (too close)");
      } else {
        RCLCPP_WARN(get_logger(), "     🚫 SKIPPED (collision)");
      }
    }

    // Add home [0,0]
    geometry_msgs::msg::PoseStamped home;
    home.header.frame_id = "map";
    home.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
    home.pose.position.x = 0.0;
    home.pose.position.y = 0.0;
    home.pose.position.z = 0.0;
    home.pose.orientation.w = 1.0;

    double home_dist = std::hypot(-robot_pose.pose.position.x, -robot_pose.pose.position.y);
    
    RCLCPP_INFO(get_logger(), "   Home [0,0]:");
    RCLCPP_INFO(get_logger(), "     Distance from robot: %.2f m", home_dist);

    if (isPoseCollisionFree(home) && home_dist > MIN_GOAL_DIST) {
      active_mission_goals_.push_back(home);
      RCLCPP_INFO(get_logger(), "     ✅ ADDED to RTB path");
    } else if (home_dist <= MIN_GOAL_DIST) {
      RCLCPP_INFO(get_logger(), "     ⏭️  SKIPPED (already home)");
    }

    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────");

    // Validate RTB path
    if (active_mission_goals_.empty()) {
      RCLCPP_ERROR(get_logger(), "❌ RTB FAILED: No valid waypoints!");
      RCLCPP_ERROR(get_logger(), "   All waypoints too close or in collision");
      return;
    }

    // Set RTB state
    rtb_mode_ = true;
    mission_active_ = false;

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_WARN(get_logger(), "🛬 RTB PATH READY:");
    RCLCPP_WARN(get_logger(), "   %zu waypoints → [0,0]", active_mission_goals_.size());
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");

    publishMarkers();
    publishPathVisualization();

    // Start RTB mission
    startCb(std::make_shared<std_msgs::msg::Empty>());
  }

  // ═══════════════════════════════════════════════════════════════════
  // RETRY SCHEDULER - Schedule retry after delay
  // ═══════════════════════════════════════════════════════════════════
  void scheduleRetry(double delay_seconds)
  {
    if (user_stopped_) {
      RCLCPP_INFO(get_logger(), "🛑 User stopped mission - not retrying");
      return;
    }

    retry_count_++;
    
    RCLCPP_WARN(get_logger(), "");
    RCLCPP_WARN(get_logger(), "⏳ Scheduling retry #%d in %.1f seconds...", 
                retry_count_, delay_seconds);
    RCLCPP_WARN(get_logger(), "   (Stop with /stop_navigation)");
    
    // Cancel existing timer if any
    if (retry_timer_) {
      retry_timer_->cancel();
    }
    
    // Create one-shot timer
    retry_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delay_seconds * 1000)),
        [this]() {
          if (!user_stopped_) {
            RCLCPP_INFO(get_logger(), "🔄 Executing retry...");
            this->startCb(std::make_shared<std_msgs::msg::Empty>());
          }
          retry_timer_->cancel();  // One-shot
        });
  }

  // ═══════════════════════════════════════════════════════════════════
  // COSTMAP CALLBACK
  // ═══════════════════════════════════════════════════════════════════
  void costmapCb(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = *msg;
    
    if (!costmap_received_) {
      RCLCPP_INFO(get_logger(), "✅ Costmap received:");
      RCLCPP_INFO(get_logger(), "   Size: %.1f x %.1f m", 
                  msg->metadata.size_x * msg->metadata.resolution,
                  msg->metadata.size_y * msg->metadata.resolution);
      RCLCPP_INFO(get_logger(), "   Resolution: %.3f m/cell", msg->metadata.resolution);
      costmap_received_ = true;
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  // COLLISION CHECKING - HYBRID MODE (allow inflated, block lethal)
  // ═══════════════════════════════════════════════════════════════════
  bool isPoseCollisionFree(const geometry_msgs::msg::PoseStamped& pose)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    
    if (!costmap_received_ || latest_costmap_.data.empty()) {
      return true;  // Optimistically free if no costmap
    }

    const auto& costmap = latest_costmap_;
    double ox = costmap.metadata.origin.position.x;
    double oy = costmap.metadata.origin.position.y;
    double res = costmap.metadata.resolution;

    int cx = static_cast<int>((pose.pose.position.x - ox) / res);
    int cy = static_cast<int>((pose.pose.position.y - oy) / res);

    // Check bounds
    if (cx < 0 || cy < 0 ||
        cx >= static_cast<int>(costmap.metadata.size_x) ||
        cy >= static_cast<int>(costmap.metadata.size_y)) {
      RCLCPP_DEBUG(get_logger(), "Pose (%.2f, %.2f) outside costmap", 
                   pose.pose.position.x, pose.pose.position.y);
      return false;
    }

    size_t idx = static_cast<size_t>(cy) * costmap.metadata.size_x + cx;
    if (idx >= costmap.data.size()) return false;

    uint8_t cost = costmap.data[idx];
    
    // ✅ HYBRID MODE: Allow inflated (1-253), block lethal (254+)
    // This allows paths through areas with dynamic obstacles
    return cost < 254;  // Changed from 253 to 254
  }

  // ═══════════════════════════════════════════════════════════════════
  // INTERPOLATION (with collision avoidance)
  // ═══════════════════════════════════════════════════════════════════
  void interpolate(const geometry_msgs::msg::PoseStamped &start,
                   const geometry_msgs::msg::PoseStamped &end,
                   std::vector<geometry_msgs::msg::PoseStamped> &out)
  {
    double dx = end.pose.position.x - start.pose.position.x;
    double dy = end.pose.position.y - start.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < 0.1) {
      out.push_back(end);
      return;
    }

    int steps = std::max(1, static_cast<int>(std::ceil(dist / interpolation_res_)));
    double path_yaw = std::atan2(dy, dx);

    RCLCPP_DEBUG(get_logger(), "     Interpolating %.2fm with %d steps", dist, steps);

    size_t collision_count = 0;
    
    for (int i = 1; i <= steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
      p.pose.position.x = start.pose.position.x + t * dx;
      p.pose.position.y = start.pose.position.y + t * dy;
      p.pose.position.z = 0.0;
      
      // Set orientation along path
      p.pose.orientation.w = std::cos(path_yaw * 0.5);
      p.pose.orientation.z = std::sin(path_yaw * 0.5);
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;

      if (isPoseCollisionFree(p)) {
        out.push_back(p);
      } else {
        collision_count++;
        RCLCPP_WARN(get_logger(), "     ⚠️  Point %d/%d (%.2f, %.2f) COLLISION - skipped",
                    i, steps, p.pose.position.x, p.pose.position.y);
      }
    }

    if (collision_count > 0) {
      RCLCPP_WARN(get_logger(), "     ⚠️  %zu/%d points skipped (collision)",
                  collision_count, steps);
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  // VISUALIZATION
  // ═══════════════════════════════════════════════════════════════════
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    // Stored goals
    for (size_t i = 0; i < goal_points_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = rtb_mode_ ? "rtb_stored_goals" : "stored_goals";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = goal_points_[i].pose;
      m.scale.x = m.scale.y = m.scale.z = 0.6;
      m.color.a = 1.0;
      
      if (rtb_mode_) {
        m.color.r = 1.0;  // Orange
        m.color.g = 0.6;
      } else {
        m.color.g = 1.0;  // Green
      }
      
      arr.markers.push_back(m);

      // Text labels
      visualization_msgs::msg::Marker text;
      text.header = m.header;
      text.ns = rtb_mode_ ? "rtb_labels" : "goal_labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = goal_points_[i].pose;
      text.pose.position.z = 1.0;
      text.scale.z = 0.5;
      text.color.r = text.color.g = text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = rtb_mode_ ? ("RTB-" + std::to_string(i + 1)) : ("G" + std::to_string(i + 1));
      arr.markers.push_back(text);
    }

    // Interpolated path
    for (size_t i = 0; i < interpolated_path_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "interpolated_path";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = interpolated_path_[i].pose;
      m.scale.x = 0.25;
      m.scale.y = 0.25;
      m.scale.z = 0.15;
      m.color.b = 1.0;  // Blue
      m.color.a = 0.7;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  void publishPathVisualization()
  {
    // Publish current path to visualization topics
    if (interpolated_path_.empty()) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
    path_msg.poses = interpolated_path_;

    path_pub_->publish(path_msg);
    global_path_pub_->publish(path_msg);
  }

  // ═══════════════════════════════════════════════════════════════════
  // GPS CALLBACK
  // ═══════════════════════════════════════════════════════════════════
  void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    
    if (!gps_printed_) {
      RCLCPP_INFO(get_logger(), "📡 GPS Fix: lat=%.8f, lon=%.8f", latitude_, longitude_);
      gps_printed_ = true;
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  // GET ROBOT POSE
  // ═══════════════════════════════════════════════════════════════════
  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
          "map", "base_link",
          tf2::TimePointZero,
          std::chrono::milliseconds(500));

      pose.header.frame_id = "map";
      pose.header.stamp = rclcpp::Time(0);  // ✅ Use Time(0)
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf.transform.rotation;
      return true;
      
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                          "TF error: %s", ex.what());
      return false;
    } catch (...) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                          "TF error: unknown exception");
      return false;
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  // SEND PATH TO NAV2 VIA FOLLOWPATH ACTION
  // ═══════════════════════════════════════════════════════════════════
  void sendPathToNav2(const nav_msgs::msg::Path &path)
  {
    if (!follow_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "❌ FollowPath action server NOT available!");
      RCLCPP_ERROR(get_logger(), "   Is controller_server running?");
      RCLCPP_ERROR(get_logger(), "   Check: ros2 node list | grep controller_server");
      mission_active_ = false;
      
      // ✅ Retry if server not available
      if (!user_stopped_) {
        scheduleRetry(5.0);
      }
      return;
    }

    // Build goal
    FollowPath::Goal goal;
    goal.path = path;
    
    // Set controller_id for Nav2 Humble
    goal.controller_id = "FollowPath";
    
    // ✅ CRITICAL FIX: Ensure timestamp is Time(0) everywhere
    goal.path.header.stamp = rclcpp::Time(0);
    for (auto &ps : goal.path.poses) {
      ps.header.stamp = rclcpp::Time(0);
    }

    auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

    // ─────────────────────────────────────────────────────────────────
    // Goal Response
    // ─────────────────────────────────────────────────────────────────
    options.goal_response_callback = [this](GoalHandleFollow::SharedPtr handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "");
        RCLCPP_ERROR(get_logger(), "❌ FollowPath REJECTED by Nav2!");
        RCLCPP_ERROR(get_logger(), "   Check controller_server logs for details");
        mission_active_ = false;
        
        // ✅ Retry if rejected
        if (!user_stopped_) {
          scheduleRetry(3.0);
        }
      } else {
        current_goal_handle_ = handle;
        last_goal_accepted_time_ = this->now();
        received_first_feedback_ = false;
        
        RCLCPP_INFO(get_logger(), "✅ FollowPath ACCEPTED by Nav2");
        RCLCPP_INFO(get_logger(), "   Goal ID: %s", 
                    rclcpp_action::to_string(handle->get_goal_id()).c_str());
      }
    };

    // ─────────────────────────────────────────────────────────────────
    // Feedback - Progress updates
    // ─────────────────────────────────────────────────────────────────
    options.feedback_callback = [this](
        GoalHandleFollow::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback) {
      
      if (!feedback) return;
      
      received_first_feedback_ = true;
      
      static auto last_log = std::chrono::steady_clock::now();
      auto now_time = std::chrono::steady_clock::now();
      auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
          now_time - last_log).count();

      // Log every 3 seconds
      if (elapsed_sec >= 3) {
        last_log = now_time;
        
        double dist_remaining = static_cast<double>(feedback->distance_to_goal);
        double speed = static_cast<double>(feedback->speed);
        
        // Calculate approximate progress
        double progress = 0.0;
        if (path_total_length_ > 0.01) {
          double dist_traveled = path_total_length_ - dist_remaining;
          progress = (dist_traveled / path_total_length_) * 100.0;
          progress = std::max(0.0, std::min(100.0, progress));
        }
        
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "📊 NAV2 PROGRESS UPDATE:");
        RCLCPP_INFO(get_logger(), "   Distance to goal: %.2f m", dist_remaining);
        RCLCPP_INFO(get_logger(), "   Current speed: %.2f m/s", speed);
        RCLCPP_INFO(get_logger(), "   Progress: %.1f%%", progress);
        
        if (speed < 0.01) {
          RCLCPP_WARN(get_logger(), "   ⚠️  Robot speed very low - may be stuck!");
        }
      }
    };

    // ─────────────────────────────────────────────────────────────────
    // Result - Mission completion
    // ─────────────────────────────────────────────────────────────────
    options.result_callback = [this](const GoalHandleFollow::WrappedResult & result) {
      current_goal_handle_.reset();

      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");

      // Safety check
      double elapsed = (this->now() - last_goal_accepted_time_).seconds();
      if (elapsed < 2.0 && !received_first_feedback_) {
        RCLCPP_WARN(get_logger(), "⚠️  FollowPath completed in %.2fs with NO feedback!", elapsed);
        RCLCPP_WARN(get_logger(), "   This usually means controller didn't actually run");
        RCLCPP_WARN(get_logger(), "   Check Nav2 configuration!");
      }

      // Handle result
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "🎉 SEGMENT COMPLETED SUCCESSFULLY!");
        RCLCPP_INFO(get_logger(), "   Segment: %zu/%zu", current_goal_index_ + 1, active_mission_goals_.size());
        
        // ✅ Advance to next goal
        current_goal_index_++;
        retry_count_ = 0;  // Reset retry counter for next segment
        
        // Check if more segments remain
        if (current_goal_index_ < active_mission_goals_.size()) {
          RCLCPP_INFO(get_logger(), "");
          RCLCPP_INFO(get_logger(), "➡️  Proceeding to next segment (%zu/%zu)...", 
                      current_goal_index_ + 1, active_mission_goals_.size());
          
          // Send next segment after short delay
          rclcpp::sleep_for(500ms);
          startCb(std::make_shared<std_msgs::msg::Empty>());
          return;  // Don't clear state yet
          
        } else {
          // All segments complete!
          RCLCPP_INFO(get_logger(), "");
          RCLCPP_INFO(get_logger(), "🎉🎉🎉 ALL SEGMENTS COMPLETED! 🎉🎉🎉");
          
          if (rtb_mode_) {
            RCLCPP_INFO(get_logger(), "   ✈️  RTB Complete - Robot at base");
            RCLCPP_INFO(get_logger(), "   Forward goals preserved: %zu", goal_points_.size());
            RCLCPP_INFO(get_logger(), "   💡 You can now add new goals or run forward mission");
          } else {
            RCLCPP_INFO(get_logger(), "   ✅ All %zu goals reached", active_mission_goals_.size());
          }
          
          active_mission_goals_.clear();
          current_goal_index_ = 0;
        }
        
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(get_logger(), "⏸️  SEGMENT CANCELED");
        RCLCPP_INFO(get_logger(), "   Current segment: %zu/%zu", current_goal_index_ + 1, active_mission_goals_.size());
        RCLCPP_INFO(get_logger(), "   Remaining segments: %zu", active_mission_goals_.size() - current_goal_index_);
        RCLCPP_INFO(get_logger(), "   Use /start_navigation to resume from current segment");
        
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(get_logger(), "⚠️  SEGMENT ABORTED BY NAV2");
        RCLCPP_INFO(get_logger(), "   Segment: %zu/%zu", current_goal_index_ + 1, active_mission_goals_.size());
        
        // ✅ Check if we're actually at the current segment goal despite abort
        geometry_msgs::msg::PoseStamped robot_pose;
        bool at_goal = false;
        
        if (getRobotPose(robot_pose) && current_goal_index_ < active_mission_goals_.size()) {
          double dist_to_goal = distance(robot_pose, active_mission_goals_[current_goal_index_]);
          
          if (dist_to_goal < 2.0) {  // Within reasonable tolerance
            RCLCPP_INFO(get_logger(), "✅ Robot is at segment goal despite ABORT status!");
            RCLCPP_INFO(get_logger(), "   Distance to goal: %.2f m", dist_to_goal);
            RCLCPP_INFO(get_logger(), "   Treating as SUCCESS");
            
            // ✅ Advance to next segment
            current_goal_index_++;
            retry_count_ = 0;
            
            if (current_goal_index_ < active_mission_goals_.size()) {
              RCLCPP_INFO(get_logger(), "");
              RCLCPP_INFO(get_logger(), "➡️  Proceeding to next segment (%zu/%zu)...", 
                          current_goal_index_ + 1, active_mission_goals_.size());
              rclcpp::sleep_for(500ms);
              startCb(std::make_shared<std_msgs::msg::Empty>());
              return;
            } else {
              RCLCPP_INFO(get_logger(), "");
              RCLCPP_INFO(get_logger(), "🎉🎉🎉 ALL SEGMENTS COMPLETED! 🎉🎉🎉");
              if (rtb_mode_) {
                RCLCPP_INFO(get_logger(), "   ✈️  RTB Complete - Robot at base");
              } else {
                RCLCPP_INFO(get_logger(), "   ✅ All %zu goals reached", active_mission_goals_.size());
              }
              active_mission_goals_.clear();
              current_goal_index_ = 0;
            }
            
            at_goal = true;
          }
        }
        
        // ✅ If not at goal, retry current segment infinitely unless user stopped
        if (!at_goal && !user_stopped_) {
          RCLCPP_WARN(get_logger(), "   Possible causes:");
          RCLCPP_WARN(get_logger(), "   - TF extrapolation error (timestamp issue)");
          RCLCPP_WARN(get_logger(), "   - Robot temporarily stuck");
          RCLCPP_WARN(get_logger(), "   - Path blocked by obstacle");
          RCLCPP_WARN(get_logger(), "   - Controller timeout");
          
          // Schedule retry for current segment
          scheduleRetry(3.0);
          return;  // Don't clear state yet
        } else if (user_stopped_) {
          RCLCPP_INFO(get_logger(), "🛑 User stopped mission - not retrying");
        }
        
      } else {
        RCLCPP_WARN(get_logger(), "⚠️  Segment ended - code: %d", 
                    static_cast<int>(result.code));
        
        // ✅ Retry on unknown result codes too
        if (!user_stopped_) {
          scheduleRetry(5.0);
          return;
        }
      }

      RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════");

      mission_active_ = false;
      interpolated_path_.clear();
      path_total_length_ = 0.0;
      
      // Clear RTB mode and reset index only on final completion or user stop
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED || 
          result.code == rclcpp_action::ResultCode::CANCELED ||
          user_stopped_) {
        rtb_mode_ = false;
        
        // Only reset index if all segments done or user stopped
        if (current_goal_index_ >= active_mission_goals_.size() || user_stopped_) {
          current_goal_index_ = 0;
        }
      }
      
      // Publish empty path to clear visualization on final completion/cancel
      if ((result.code == rclcpp_action::ResultCode::SUCCEEDED && current_goal_index_ == 0) ||
          result.code == rclcpp_action::ResultCode::CANCELED || 
          user_stopped_) {
        nav_msgs::msg::Path empty;
        empty.header.frame_id = "map";
        empty.header.stamp = rclcpp::Time(0);
        path_pub_->publish(empty);
        global_path_pub_->publish(empty);
      }
      
      publishMarkers();
    };

    RCLCPP_INFO(get_logger(), "📤 Sending %zu waypoints to FollowPath...", path.poses.size());
    follow_client_->async_send_goal(goal, options);
  }
};

// ═══════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointManagerInterpolater>();

  RCLCPP_INFO(node->get_logger(), "");
  RCLCPP_INFO(node->get_logger(), "Waypoint Manager spinning...");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to shutdown");
  RCLCPP_INFO(node->get_logger(), "");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}