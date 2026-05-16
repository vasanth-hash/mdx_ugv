/*
pause and resume functionalities are removed 
normal navigation and rtb are working 
*/

#include <vector>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class WaypointManagerInterpolater : public rclcpp::Node
{
public:
  using NavAction = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  WaypointManagerInterpolater()
  : Node("waypoint_manager"), interpolation_res_(3.0),
    current_goal_handle_(nullptr), is_paused_(false), mission_active_(false),
    last_reached_goal_idx_(0), completed_goals_count_(0), goal_rtb_mode_(false), gps_printed_(false)
  {
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", 1, 
        std::bind(&WaypointManagerInterpolater::costmapCb, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&WaypointManagerInterpolater::goalCb, this, _1));

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/start_navigation", 10, std::bind(&WaypointManagerInterpolater::startCb, this, _1));

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/stop_navigation", 10, std::bind(&WaypointManagerInterpolater::stopCb, this, _1));
    
    // ✅ RTB topic (goal_points_ only)
    rtb_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/rtb", 10, std::bind(&WaypointManagerInterpolater::rtbCb, this, _1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsat/fix", 10, std::bind(&WaypointManagerInterpolater::gpsCb, this, _1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/interpolated_waypoints", 10);

    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_through_poses");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Waypoint Manager READY - goal_points_ RTB enabled");
  }

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_, stop_sub_, rtb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap latest_costmap_;
  std::mutex costmap_mutex_;
  bool costmap_received_ = false;  

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State - ✅ ONLY goal_points_ (no breadcrumbs)
  std::vector<geometry_msgs::msg::PoseStamped> goal_points_;      // ALL goals from RViz (permanent)
  std::vector<geometry_msgs::msg::PoseStamped> raw_goals_;        // Active mission goals
  std::vector<geometry_msgs::msg::PoseStamped> interpolated_path_;
  double interpolation_res_;

  GoalHandle::SharedPtr current_goal_handle_;
  bool is_paused_ = false;
  bool mission_active_ = false;
  bool goal_rtb_mode_ = false;
  size_t last_reached_goal_idx_ = 0;
  size_t completed_goals_count_ = 0;
  rclcpp::TimerBase::SharedPtr pause_timer_;
  const double GOAL_REACHED_DIST = 0.5;

  double latitude_{0.0}, longitude_{0.0};
  bool gps_printed_{false};

  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped p = *msg;
    p.header.frame_id = "map";
    p.pose.position.z = 0.0;
    
    // ✅ Store in goal_points_ (permanent storage)
    goal_points_.push_back(p);
    raw_goals_.push_back(p);

    RCLCPP_INFO(get_logger(), "Goal %zu added to goal_points_: (%.6f, %.6f) | Total: %zu", 
                goal_points_.size(), p.pose.position.x, p.pose.position.y, goal_points_.size());
    publishMarkers();
  }

  void rtbCb(const std_msgs::msg::Empty::SharedPtr)
  { 
    static bool rtb_in_progress = false;
    if (rtb_in_progress) {
        RCLCPP_WARN(get_logger(), "RTB already running, ignoring trigger");
        return;
    }
    rtb_in_progress = true;

    RCLCPP_WARN(get_logger(), "🛬 GOAL_POINTS RTB TRIGGERED - ALL %zu goals", goal_points_.size());

    // 1. Cancel current navigation
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_ = nullptr;
        rclcpp::sleep_for(std::chrono::milliseconds(800));
    }

    // 2. Get current robot position
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        RCLCPP_ERROR(get_logger(), "RTB: Cannot get robot pose");
        rtb_in_progress = false;
        return;
    }

    // ✅ FIXED: Build REVERSE path from ALL goal_points_ → [0,0]
    std::vector<geometry_msgs::msg::PoseStamped> rtb_path;
    const double MIN_RTB_GOAL_DIST = 2.0;

    // Reverse ALL goals (Goal3 → Goal2 → Goal1)
    for (int i = static_cast<int>(goal_points_.size()) - 1; i >= 0; --i) {
        auto p = goal_points_[i];
        p.header.frame_id = "map";
        p.header.stamp = now();
        p.pose.position.z = 0.0;
        p.pose.orientation.w = 1.0;

        double dist = std::hypot(p.pose.position.x - robot_pose.pose.position.x,
                                p.pose.position.y - robot_pose.pose.position.y);

        if (dist > MIN_RTB_GOAL_DIST && isPoseCollisionFree(p)) {
            rtb_path.push_back(p);
            RCLCPP_INFO(get_logger(), "✅ RTB Goal %d added: (%.1f, %.1f) dist=%.1fm",
                       i+1, p.pose.position.x, p.pose.position.y, dist);
        } else if (dist <= MIN_RTB_GOAL_DIST) {
            RCLCPP_INFO(get_logger(), "⏭️ RTB Goal %d skipped (too close): (%.1f, %.1f) dist=%.1fm",
                       i+1, p.pose.position.x, p.pose.position.y, dist);
        } else {
            RCLCPP_WARN(get_logger(), "🚫 RTB Goal %d rejected (collision): (%.1f, %.1f)",
                       i+1, p.pose.position.x, p.pose.position.y);
        }
    }

    // Add home [0,0] as final goal
    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.frame_id = "map";
    base_pose.header.stamp = now();
    base_pose.pose.position.x = 0.0;
    base_pose.pose.position.y = 0.0;
    base_pose.pose.position.z = 0.0;
    base_pose.pose.orientation.w = 1.0;

    double home_dist = std::hypot(base_pose.pose.position.x - robot_pose.pose.position.x,
                                  base_pose.pose.position.y - robot_pose.pose.position.y);
    if (isPoseCollisionFree(base_pose) && home_dist > MIN_RTB_GOAL_DIST) {
        rtb_path.push_back(base_pose);
        RCLCPP_INFO(get_logger(), "🏠 RTB Datum added: (0.0, 0.0) dist=%.1fm", home_dist);
    }

    // 3. Update both vectors for RTB navigation
    goal_points_ = rtb_path;
    raw_goals_ = rtb_path;
    
    interpolated_path_.clear();
    mission_active_ = false;
    goal_rtb_mode_ = true;

    RCLCPP_INFO(get_logger(), "🛬 RTB Path COMPLETE: %zu goals → [0,0]", rtb_path.size()-1);
    publishMarkers();
    startCb(std::make_shared<std_msgs::msg::Empty>());
    rtb_in_progress = false;
  }

  void startCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (raw_goals_.empty()) {
      RCLCPP_WARN(get_logger(), "No raw_goals_ to execute");
      goal_rtb_mode_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_ERROR(get_logger(), "Cannot get robot pose");
      return;
    }

    RCLCPP_INFO(get_logger(), "🤖 Robot(%.1f, %.1f) | %s | goal_points_: %zu", 
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                goal_rtb_mode_ ? "RTB" : "NORMAL", goal_points_.size());

    interpolated_path_.clear();

    // Start from completed goals (RTB always starts from 0)
    size_t start_idx = 0;
    if (start_idx >= raw_goals_.size()) {
      RCLCPP_INFO(get_logger(), "%s All goal_points_ completed!", goal_rtb_mode_ ? "✅" : "✅");
      goal_rtb_mode_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped current_pose = robot_pose;
    
    for (size_t i = start_idx; i < raw_goals_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "➡️ Goal %zu (%.1f, %.1f)",
                  i + 1,
                  raw_goals_[i].pose.position.x,
                  raw_goals_[i].pose.position.y);

      interpolate(current_pose, raw_goals_[i], interpolated_path_);
      current_pose = raw_goals_[i];
    }

    if (interpolated_path_.empty()) {
      RCLCPP_INFO(get_logger(), "✅ No path needed!");
      goal_rtb_mode_ = false;
      return;
    }

    publishMarkers();

    RCLCPP_INFO(get_logger(), "🚀 %s MISSION START (%zu interpolated poses)", 
                goal_rtb_mode_ ? "RTB🛬" : "NORMAL➡️", interpolated_path_.size());
    completed_goals_count_ = 0;

    sendPathToNav2(interpolated_path_);
    mission_active_ = true;
  }

  void stopCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (current_goal_handle_) {
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
    }
    mission_active_ = false;
    is_paused_ = false;
    goal_rtb_mode_ = false;
    if (pause_timer_) {
      pause_timer_->cancel();
      pause_timer_ = nullptr;
    }
    interpolated_path_.clear();
    RCLCPP_INFO(get_logger(), "🔴 MISSION STOPPED | goal_points_ preserved: %zu", goal_points_.size());
    publishMarkers();
  }

  void costmapCb(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = *msg;
    costmap_received_ = true;
  }

  bool isPoseCollisionFree(const geometry_msgs::msg::PoseStamped& pose)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_received_ || latest_costmap_.data.empty()) return true;
    
    const auto& costmap = latest_costmap_;
    double ox = costmap.metadata.origin.position.x;
    double oy = costmap.metadata.origin.position.y;
    double res = costmap.metadata.resolution;
    
    int cx = static_cast<int>((pose.pose.position.x - ox) / res);
    int cy = static_cast<int>((pose.pose.position.y - oy) / res);
    
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(costmap.metadata.size_x) || 
        cy >= static_cast<int>(costmap.metadata.size_y)) return false;
    
    size_t idx = static_cast<size_t>(cy) * costmap.metadata.size_x + cx;
    if (idx >= costmap.data.size()) return false;
    
    return costmap.data[idx] < 252;
  }

  void sendPathToNav2(const std::vector<geometry_msgs::msg::PoseStamped>& path)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "Nav2 unavailable");
      return;
    }

    size_t path_size = path.size();

    NavAction::Goal goal;
    goal.poses = path;

    auto options = rclcpp_action::Client<NavAction>::SendGoalOptions();
    options.goal_response_callback = [this, path_size](GoalHandle::SharedPtr handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Mission rejected");
        mission_active_ = false;
      } else {
        current_goal_handle_ = handle;
        RCLCPP_INFO(get_logger(), "✅ %s Mission accepted (%zu poses)", 
                    goal_rtb_mode_ ? "RTB" : "NORMAL", path_size);
      }
    };

    options.feedback_callback = [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const NavAction::Feedback> feedback) {
      if (!feedback || feedback->current_pose.header.frame_id.empty()) return;

      const auto& current = feedback->current_pose;
      
      // ✅ Check ALL goals, not just front()
      for (size_t i = 0; i < raw_goals_.size(); ++i) {
        const auto& goal = raw_goals_[i];
        double dx = goal.pose.position.x - current.pose.position.x;
        double dy = goal.pose.position.y - current.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < GOAL_REACHED_DIST) {
          RCLCPP_INFO(get_logger(), "🎯 %s GOAL %zu REACHED (%.2f, %.2f) | remaining: %zu",
                      goal_rtb_mode_ ? "RTB" : "NORMAL", i+1,
                      goal.pose.position.x, goal.pose.position.y,
                      raw_goals_.size() - 1);

          // 🔥 Remove reached goal (by index, not always front)
          raw_goals_.erase(raw_goals_.begin() + i);
          
          if (!goal_rtb_mode_) {
            completed_goals_count_++;
          }
          return; // Only process one goal per feedback
        }
      }
    };

    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      current_goal_handle_ = nullptr;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (!goal_rtb_mode_) {
          completed_goals_count_ = raw_goals_.size();
        }
        RCLCPP_INFO(get_logger(), "🎉 %s MISSION COMPLETED SUCCESSFULLY", 
                    goal_rtb_mode_ ? "RTB" : "NORMAL");
      } else {
        RCLCPP_WARN(get_logger(), "Mission ended: %d", result.code);
      }
      mission_active_ = false;
      is_paused_ = false;
      goal_rtb_mode_ = false;
      interpolated_path_.clear();
      publishMarkers();
    };

    nav_client_->async_send_goal(goal, options);
  }

  void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    if (!gps_printed_) {
      RCLCPP_INFO(get_logger(), "GPS: lat=%.15f lon=%.15f", latitude_, longitude_);
      gps_printed_ = true;
    }  
  }

  void interpolate(const geometry_msgs::msg::PoseStamped &a,
                   const geometry_msgs::msg::PoseStamped &b,
                   std::vector<geometry_msgs::msg::PoseStamped> &out)
  {
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    double dist = std::hypot(dx, dy);
    int steps = std::max(1, static_cast<int>(dist / interpolation_res_));
    double path_yaw = std::atan2(dy, dx);

    for (int i = 1; i < steps; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = now();
      double r = static_cast<double>(i) / steps;
      p.pose.position.x = a.pose.position.x + r * dx;
      p.pose.position.y = a.pose.position.y + r * dy;
      p.pose.position.z = 0.0;
      // p.pose.orientation = b.pose.orientation;
      double interp_dx = p.pose.position.x - a.pose.position.x;
      double interp_dy = p.pose.position.y - a.pose.position.y;
      double yaw = std::atan2(interp_dy, interp_dx);
      p.pose.orientation.w = std::cos(path_yaw / 2.0);
      p.pose.orientation.z = std::sin(path_yaw / 2.0);
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
        
      if (!isPoseCollisionFree(p)) {
        RCLCPP_WARN(get_logger(), "⚠️ LETHAL OBSTACLE at interpolated point (%.2f, %.2f) - SKIPPED",
                    p.pose.position.x, p.pose.position.y);
      } else {
        out.push_back(p);
      }
    }

    geometry_msgs::msg::PoseStamped final_goal = b;  // Copy b
    final_goal.pose.orientation.w = std::cos(path_yaw / 2.0);
    final_goal.pose.orientation.z = std::sin(path_yaw / 2.0);
    final_goal.pose.orientation.x = 0.0;
    final_goal.pose.orientation.y = 0.0;
      
    if (!isPoseCollisionFree(final_goal)) {
      RCLCPP_WARN(get_logger(), "⚠️ LETHAL OBSTACLE at endpoint Goal (%.2f, %.2f) - SKIPPED",
                  final_goal.pose.position.x, final_goal.pose.position.y);
    } else {
      out.push_back(final_goal);  // Add COPY, not original b
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    // RTB markers (orange) vs Normal (green/red)
    std::string ns = goal_rtb_mode_ ? "rtb_goals" : "goal_points";
    
    for (size_t i = 0; i < goal_points_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = ns;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = goal_points_[i].pose;
      m.scale.x = m.scale.y = m.scale.z = 0.4;
      m.color.a = 1.0;
      m.color.g = 1.0;
      arr.markers.push_back(m);
    }

    // Active mission path (blue)
    for (size_t i = 0; i < interpolated_path_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "path";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = interpolated_path_[i].pose;
      m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.15;
      m.color.b = 1.0; m.color.a = 0.8;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose)
  {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      pose.header.frame_id = "map";
      pose.header.stamp = now();
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (...) {
      return false;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManagerInterpolater>());
  rclcpp::shutdown();
  return 0;
}
