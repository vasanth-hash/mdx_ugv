
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>

class DynmaicObstacleProcessor : public rclcpp::Node
{
public:
  DynmaicObstacleProcessor()
  : Node("dynamic_obstacle_processor"),
    next_obstacle_id_(0),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("trigger_radius", 8.0);
    declare_parameter("human_height_min", 0.3);
    declare_parameter("human_height_max", 2.2);
    declare_parameter("inflation_radius", 0.6);
    declare_parameter("velocity_arrow_scale", 3.0);
    declare_parameter("max_human_radius", 0.9);
    declare_parameter("min_cluster_points", 17);
    declare_parameter("fixed_frame", "map");
    declare_parameter("obstacle_timeout", 0.3);
    declare_parameter("disable_point_filtering", false);
    declare_parameter("min_velocity_display", 0.05);
    declare_parameter("clustering_distance", 1.5);
    declare_parameter("association_distance", 2.0);
    declare_parameter("min_dynamic_velocity", 0.15);
    declare_parameter("static_observation_time", 2.0);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster_points", rclcpp::SensorDataQoS(),
      std::bind(&DynmaicObstacleProcessor::cloudCallback, this, std::placeholders::_1));

    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ouster_points_filtered", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/human_markers", 10);
    obstacle_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/obstacles", 10);
    obstacle_traj_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/obstacle_trajectory", 10);

    RCLCPP_INFO(get_logger(), "Dynamic Obstacle Node: Multi-object tracking enabled");
  }

private:
  struct ObstacleState
  {
    Eigen::Vector2f pos;
    Eigen::Vector2f velocity;
    float radius;
    rclcpp::Time stamp;
    rclcpp::Time last_detected;
    rclcpp::Time first_detected;
    int missed_frames;
    std::string frame_id;
    bool is_dynamic;
    float max_observed_velocity;
  };

  struct Cluster
  {
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector2f center;
    float radius;
  };

  std::unordered_map<int, ObstacleState> prev_state_;
  int next_obstacle_id_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_traj_pub_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double trigger_radius = get_parameter("trigger_radius").as_double();
    const double z_min = get_parameter("human_height_min").as_double();
    const double z_max = get_parameter("human_height_max").as_double();
    const double inflation = get_parameter("inflation_radius").as_double();
    const double max_radius = get_parameter("max_human_radius").as_double();
    const int min_points = get_parameter("min_cluster_points").as_int();
    const std::string fixed_frame = get_parameter("fixed_frame").as_string();
    const bool disable_filtering = get_parameter("disable_point_filtering").as_bool();
    const double cluster_dist = get_parameter("clustering_distance").as_double();
    const double assoc_dist = get_parameter("association_distance").as_double();
    const double min_dynamic_vel = get_parameter("min_dynamic_velocity").as_double();
    const double static_obs_time = get_parameter("static_observation_time").as_double();

    rclcpp::Time current_time(msg->header.stamp);

    std::vector<Eigen::Vector3f> candidate_points;
    candidate_points.reserve(2000);

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz)
    {
      float distance = std::hypot(*ix, *iy);
      if (distance < trigger_radius && distance > 0.1 && *iz > z_min && *iz < z_max)
      {
        candidate_points.emplace_back(*ix, *iy, *iz);
      }
    }

    std::vector<Cluster> clusters = clusterPoints(candidate_points, cluster_dist);

    std::vector<bool> prev_matched(prev_state_.size(), false);
    std::vector<int> prev_ids;
    for (const auto &[id, state] : prev_state_)
    {
      prev_ids.push_back(id);
    }

    std::vector<int> detected_ids;

    for (auto &cluster : clusters)
    {
      if ((int)cluster.points.size() < min_points || cluster.radius > max_radius)
        continue;

      Eigen::Vector3f centroid(0, 0, 0);
      for (const auto &p : cluster.points)
        centroid += p;
      centroid /= static_cast<float>(cluster.points.size());

      float radius = 0.0f;
      for (const auto &p : cluster.points)
        radius = std::max(radius, (p.head<2>() - centroid.head<2>()).norm());

      radius += inflation;

      geometry_msgs::msg::PointStamped in, out;
      in.header = msg->header;
      in.point.x = centroid.x();
      in.point.y = centroid.y();
      in.point.z = centroid.z();

      try
      {
        tf_buffer_.transform(in, out, fixed_frame, tf2::durationFromSec(0.2));
        
        Eigen::Vector2f center_fixed(out.point.x, out.point.y);
        rclcpp::Time transform_time(msg->header.stamp);

        int matched_id = findClosestObstacle(center_fixed, assoc_dist);
        
        if (matched_id >= 0)
        {
          updateObstacleState(matched_id, center_fixed, radius, transform_time, fixed_frame, min_dynamic_vel, static_obs_time);
          detected_ids.push_back(matched_id);
          
          auto it = std::find(prev_ids.begin(), prev_ids.end(), matched_id);
          if (it != prev_ids.end())
          {
            size_t idx = std::distance(prev_ids.begin(), it);
            prev_matched[idx] = true;
          }
        }
        else
        {
          int new_id = next_obstacle_id_++;
          // RCLCPP_INFO(get_logger(), "New obstacle ID %d at (%.2f, %.2f)", 
          //             new_id, center_fixed.x(), center_fixed.y());
          updateObstacleState(new_id, center_fixed, radius, transform_time, fixed_frame, min_dynamic_vel, static_obs_time);
          detected_ids.push_back(new_id);
        }
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed: %s", ex.what());
      }
    }

    for (size_t i = 0; i < prev_ids.size(); ++i)
    {
      if (!prev_matched[i])
      {
        prev_state_[prev_ids[i]].missed_frames++;
      }
    }

    cleanExpiredObstacles();
    publishAllObstacles(msg->header, fixed_frame, current_time);

    // Only filter out DYNAMIC obstacles from the lidar data
    if (!disable_filtering)
    {
      std::vector<std::pair<Eigen::Vector2f, float>> dynamic_obstacles_to_filter;
      for (const auto &[id, state] : prev_state_)
      {
        // ONLY filter dynamic obstacles
        if (state.is_dynamic)
        {
          dynamic_obstacles_to_filter.emplace_back(state.pos, state.radius);
        }
      }
      
      if (!dynamic_obstacles_to_filter.empty())
      {
        publishFilteredCloud(msg, dynamic_obstacles_to_filter, msg->header.frame_id, fixed_frame);
      }
      else
      {
        // No dynamic obstacles, publish original cloud
        filtered_cloud_pub_->publish(*msg);
      }
    }
    else
    {
      filtered_cloud_pub_->publish(*msg);
    }
  }

  std::vector<Cluster> clusterPoints(const std::vector<Eigen::Vector3f> &points, double max_dist)
  {
    std::vector<Cluster> clusters;
    if (points.empty()) return clusters;

    std::vector<bool> assigned(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (assigned[i]) continue;

      Cluster cluster;
      cluster.points.push_back(points[i]);
      assigned[i] = true;

      for (size_t j = 0; j < cluster.points.size(); ++j)
      {
        for (size_t k = 0; k < points.size(); ++k)
        {
          if (assigned[k]) continue;

          float dist = (cluster.points[j].head<2>() - points[k].head<2>()).norm();
          if (dist < max_dist)
          {
            cluster.points.push_back(points[k]);
            assigned[k] = true;
          }
        }
      }

      Eigen::Vector2f center(0, 0);
      for (const auto &p : cluster.points)
      {
        center.x() += p.x();
        center.y() += p.y();
      }
      center /= static_cast<float>(cluster.points.size());
      cluster.center = center;

      float radius = 0.0f;
      for (const auto &p : cluster.points)
      {
        radius = std::max(radius, (p.head<2>() - center).norm());
      }
      cluster.radius = radius;

      clusters.push_back(cluster);
    }

    return clusters;
  }

  int findClosestObstacle(const Eigen::Vector2f &pos, double max_dist)
  {
    int closest_id = -1;
    double min_dist = max_dist;

    for (const auto &[id, state] : prev_state_)
    {
      double dist = (state.pos - pos).norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        closest_id = id;
      }
    }

    return closest_id;
  }

  void updateObstacleState(int id, const Eigen::Vector2f &pos, float radius, 
                          const rclcpp::Time &stamp, const std::string &frame_id,
                          double min_dynamic_vel, double static_obs_time)
  {
    Eigen::Vector2f velocity(0.0f, 0.0f);
    rclcpp::Time first_detect = stamp;
    float max_vel = 0.0f;

    if (prev_state_.count(id))
    {
      auto &prev = prev_state_[id];
      double dt = (stamp - prev.stamp).seconds();
      first_detect = prev.first_detected;
      max_vel = prev.max_observed_velocity;

      if (dt > 0.02 && dt < 1.5)
      {
        velocity.x() = (pos.x() - prev.pos.x()) / dt;
        velocity.y() = (pos.y() - prev.pos.y()) / dt;

        velocity.x() = std::clamp(velocity.x(), -3.0f, 3.0f);
        velocity.y() = std::clamp(velocity.y(), -3.0f, 3.0f);
        
        velocity = 0.6f * velocity + 0.4f * prev.velocity;
        
        float current_speed = velocity.norm();
        if (current_speed > max_vel)
        {
          max_vel = current_speed;
        }
      }
      else
      {
        velocity = prev.velocity * 0.8f;
      }
    }

    double observation_duration = (stamp - first_detect).seconds();
    bool is_dynamic = true;

    if (observation_duration > static_obs_time)
    {
      if (max_vel < min_dynamic_vel)
      {
        is_dynamic = false;
        if (!prev_state_.count(id) || prev_state_[id].is_dynamic)
        {
          // RCLCPP_INFO(get_logger(), 
          //   "Obstacle %d classified as STATIC (max_vel=%.3f < %.3f m/s, observed %.1fs)",
          //   id, max_vel, min_dynamic_vel, observation_duration);
        }
      }
      else
      {
        if (!prev_state_.count(id) || !prev_state_[id].is_dynamic)
        {
          // RCLCPP_INFO(get_logger(), 
          //   "Obstacle %d confirmed as DYNAMIC (max_vel=%.3f m/s)",
          //   id, max_vel);
        }
      }
    }

    prev_state_[id] = {pos, velocity, radius, stamp, stamp, first_detect, 0, frame_id, 
                       is_dynamic, max_vel};
  }

  void cleanExpiredObstacles()
  {
    std::vector<int> to_remove;
    
    for (const auto &[id, state] : prev_state_)
    {
      if (state.missed_frames > 3)
      {
        to_remove.push_back(id);
        // RCLCPP_INFO(get_logger(), "Removed obstacle %d (missed %d frames)", 
        //             id, state.missed_frames);
      }
    }

    for (int id : to_remove)
    {
      prev_state_.erase(id);
    }
    
    if (prev_state_.empty())
    {
      visualization_msgs::msg::MarkerArray empty_array;
      visualization_msgs::msg::Marker delete_all;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      empty_array.markers.push_back(delete_all);
      marker_pub_->publish(empty_array);
    }
  }

  void publishAllObstacles(const std_msgs::msg::Header &header, 
                          const std::string &fixed_frame,
                          const rclcpp::Time &current_time)
  {
    costmap_converter_msgs::msg::ObstacleArrayMsg obs_array;
    costmap_converter_msgs::msg::ObstacleArrayMsg traj_array;
    
    std_msgs::msg::Header fixed_header = header;
    fixed_header.frame_id = fixed_frame;
    fixed_header.stamp = current_time;
    
    obs_array.header = fixed_header;
    traj_array.header = fixed_header;

    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto &[id, state] : prev_state_)
    {
      // Only publish DYNAMIC obstacles to TEB controller
      if (state.is_dynamic)
      {
        costmap_converter_msgs::msg::ObstacleMsg obs;
        obs.id = id;
        obs.radius = state.radius;

        geometry_msgs::msg::Point32 p;
        p.x = state.pos.x();
        p.y = state.pos.y();
        p.z = 0.0f;
        obs.polygon.points.push_back(p);

        geometry_msgs::msg::TwistWithCovariance vel_cov;
        vel_cov.twist.linear.x = state.velocity.x();
        vel_cov.twist.linear.y = state.velocity.y();
        vel_cov.twist.linear.z = 0.0;

        obs.velocities = vel_cov;
        obs_array.obstacles.push_back(obs);

        const double T = 2.0;
        const double dt = 0.2;

        for (double t = dt; t <= T; t += dt)
        {
          costmap_converter_msgs::msg::ObstacleMsg future = obs;
          geometry_msgs::msg::Point32 fp;
          fp.x = state.pos.x() + state.velocity.x() * t;
          fp.y = state.pos.y() + state.velocity.y() * t;
          fp.z = 0.0;
          future.polygon.points.clear();
          future.polygon.points.push_back(fp);
          traj_array.obstacles.push_back(future);
        }
      }

      // Visualize ALL obstacles (dynamic and static) with different colors/labels
      createCombinedMarker(marker_array, fixed_header, id, state);
    }

    obstacle_pub_->publish(obs_array);
    obstacle_traj_pub_->publish(traj_array);
    marker_pub_->publish(marker_array);
  }

  void createCombinedMarker(visualization_msgs::msg::MarkerArray &array,
                           const std_msgs::msg::Header &header,
                           int id,
                           const ObstacleState &state)
  {
    const double min_vel = get_parameter("min_velocity_display").as_double();

    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "human_sphere";
    sphere.id = id;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;

    sphere.pose.position.x = state.pos.x();
    sphere.pose.position.y = state.pos.y();
    sphere.pose.position.z = 1.0;
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = state.radius * 2.0;
    sphere.scale.y = state.radius * 2.0;
    sphere.scale.z = state.radius * 2.0;

    if (state.is_dynamic)
    {
      // Bright colors for dynamic obstacles
      if (id == 0) {
        sphere.color.r = 1.0; sphere.color.g = 0.0; sphere.color.b = 0.0;
      } else if (id == 1) {
        sphere.color.r = 0.0; sphere.color.g = 0.0; sphere.color.b = 1.0;
      } else if (id == 2) {
        sphere.color.r = 1.0; sphere.color.g = 0.5; sphere.color.b = 0.0;
      } else if (id == 3) {
        sphere.color.r = 0.5; sphere.color.g = 0.0; sphere.color.b = 0.5;
      } else {
        float hue = (id * 0.3f);
        hue = hue - std::floor(hue);
        sphere.color.r = hue; 
        sphere.color.g = 1.0 - hue; 
        sphere.color.b = 0.5;
      }
      sphere.color.a = 0.5;
    }
    else
    {
      // Gray/transparent for static obstacles
      sphere.color.r = 0.5;
      sphere.color.g = 0.5;
      sphere.color.b = 0.5;
      sphere.color.a = 0.3;
    }

    sphere.lifetime = rclcpp::Duration::from_seconds(0.2);
    array.markers.push_back(sphere);

    float vel_norm = state.velocity.norm();
    if (state.is_dynamic && vel_norm > min_vel && state.missed_frames == 0)
    {
      // Velocity arrow for dynamic obstacles
      visualization_msgs::msg::Marker arrow;
      arrow.header = header;
      arrow.ns = "human_velocity";
      arrow.id = id + 1000;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point a, b;
      a.x = state.pos.x();
      a.y = state.pos.y();
      a.z = 1.2;

      double arrow_scale = get_parameter("velocity_arrow_scale").as_double();

      b.x = state.pos.x() + state.velocity.x() * arrow_scale;
      b.y = state.pos.y() + state.velocity.y() * arrow_scale;
      b.z = 1.2;

      arrow.points = {a, b};
      arrow.scale.x = 0.07;
      arrow.scale.y = 0.15;
      arrow.scale.z = 0.25;

      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.0;
      arrow.color.a = 1.0;

      arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
      // array.markers.push_back(arrow);
    }
    else if (!state.is_dynamic)
    {
      // "STATIC" label for static obstacles
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "static_label";
      text.id = id + 2000;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = state.pos.x();
      text.pose.position.y = state.pos.y();
      text.pose.position.z = 1.8;
      text.pose.orientation.w = 1.0;

      text.text = "STATIC";
      text.scale.z = 0.3;

      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 0.8;

      text.lifetime = rclcpp::Duration::from_seconds(0.2);
      // array.markers.push_back(text);
    }
  }

  void publishFilteredCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input,
                            const std::vector<std::pair<Eigen::Vector2f, float>> &obstacles,
                            const std::string &sensor_frame,
                            const std::string &fixed_frame)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    output.height = 1;
    output.is_dense = false;
    output.is_bigendian = input->is_bigendian;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(input->width * input->height);

    sensor_msgs::PointCloud2Iterator<float> ox(output, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(output, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(output, "z");

    sensor_msgs::PointCloud2ConstIterator<float> ix(*input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*input, "z");

    std::vector<std::pair<Eigen::Vector2f, float>> obstacles_in_sensor;
    obstacles_in_sensor.reserve(obstacles.size());

    for (const auto &[pos_fixed, radius] : obstacles)
    {
      geometry_msgs::msg::PointStamped in, out;
      in.header.frame_id = fixed_frame;
      in.header.stamp = input->header.stamp;
      in.point.x = pos_fixed.x();
      in.point.y = pos_fixed.y();
      in.point.z = 0.0;

      try
      {
        tf_buffer_.transform(in, out, sensor_frame, tf2::durationFromSec(0.1));
        Eigen::Vector2f pos_sensor(out.point.x, out.point.y);
        obstacles_in_sensor.emplace_back(pos_sensor, radius);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
          "Transform failed for filtering: %s", ex.what());
        continue;
      }
    }

    size_t kept = 0;

    for (; ix != ix.end(); ++ix, ++iy, ++iz)
    {
      bool inside_any_obstacle = false;
      
      for (const auto &[center, radius] : obstacles_in_sensor)
      {
        float distance = std::hypot(*ix - center.x(), *iy - center.y());
        
        if (distance <= radius)
        {
          inside_any_obstacle = true;
          break;
        }
      }

      if (!inside_any_obstacle)
      {
        *ox = *ix;
        *oy = *iy;
        *oz = *iz;

        ++ox;
        ++oy;
        ++oz;
        kept++;
      }
    }

    modifier.resize(kept);
    output.width = kept;

    filtered_cloud_pub_->publish(output);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  try
  {
    auto node = std::make_shared<DynmaicObstacleProcessor>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}


/*
This below code is for the generic dynamic obstacles
*/
/*
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>

class DynamicObstacleProcessor : public rclcpp::Node
{
public:
  DynamicObstacleProcessor()
  : Node("dynamic_obstacle_processor"),
    next_obstacle_id_(0),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("trigger_radius", 15.0);
    declare_parameter("height_min", 0.0);
    declare_parameter("height_max", 4.0);
    declare_parameter("inflation_radius", 1.0);
    declare_parameter("velocity_arrow_scale", 3.0);
    declare_parameter("max_obstacle_radius", 3.0);
    declare_parameter("min_points_per_obstacle", 5);
    declare_parameter("fixed_frame", "map");
    declare_parameter("obstacle_timeout", 0.3);
    declare_parameter("disable_point_filtering", false);
    declare_parameter("min_velocity_display", 0.05);
    declare_parameter("association_distance", 2.0);
    declare_parameter("min_dynamic_velocity", 0.10);
    declare_parameter("static_observation_time", 2.0);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster_points", rclcpp::SensorDataQoS(),
      std::bind(&DynamicObstacleProcessor::cloudCallback, this, std::placeholders::_1));

    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ouster_points_filtered", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);
    obstacle_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/obstacles", 10);
    obstacle_traj_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/obstacle_trajectory", 10);

    RCLCPP_INFO(get_logger(), "Dynamic Obstacle Processor: Generic obstacle tracking enabled");
  }

private:
  struct ObstacleState
  {
    Eigen::Vector2f pos;
    Eigen::Vector2f velocity;
    float radius;
    rclcpp::Time stamp;
    rclcpp::Time last_detected;
    rclcpp::Time first_detected;
    int missed_frames;
    std::string frame_id;
    bool is_dynamic;
    float max_observed_velocity;
    int point_count;
  };

  std::unordered_map<int, ObstacleState> prev_state_;
  int next_obstacle_id_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_traj_pub_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double trigger_radius = get_parameter("trigger_radius").as_double();
    const double z_min = get_parameter("height_min").as_double();
    const double z_max = get_parameter("height_max").as_double();
    const double inflation = get_parameter("inflation_radius").as_double();
    const double max_radius = get_parameter("max_obstacle_radius").as_double();
    const int min_points = get_parameter("min_points_per_obstacle").as_int();
    const std::string fixed_frame = get_parameter("fixed_frame").as_string();
    const bool disable_filtering = get_parameter("disable_point_filtering").as_bool();
    const double assoc_dist = get_parameter("association_distance").as_double();
    const double min_dynamic_vel = get_parameter("min_dynamic_velocity").as_double();
    const double static_obs_time = get_parameter("static_observation_time").as_double();

    rclcpp::Time current_time(msg->header.stamp);

    std::vector<Eigen::Vector3f> candidate_points;
    candidate_points.reserve(5000);

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz)
    {
      float distance = std::hypot(*ix, *iy);
      if (distance < trigger_radius && distance > 0.1 && *iz > z_min && *iz < z_max)
      {
        candidate_points.emplace_back(*ix, *iy, *iz);
      }
    }

    std::vector<bool> point_used(candidate_points.size(), false);
    std::vector<int> prev_ids;
    for (const auto &[id, state] : prev_state_) prev_ids.push_back(id);

    std::vector<int> detected_ids;

    for (int id : prev_ids)
    {
      if (prev_state_.find(id) == prev_state_.end()) continue;

      std::vector<size_t> associated_points;
      for (size_t i = 0; i < candidate_points.size(); ++i)
      {
        if (point_used[i]) continue;
        Eigen::Vector2f point_pos(candidate_points[i].x(), candidate_points[i].y());
        float dist = (prev_state_[id].pos - point_pos).norm();
        if (dist < assoc_dist)
        {
          associated_points.push_back(i);
          point_used[i] = true;
        }
      }

      if (!associated_points.empty())
      {
        Eigen::Vector2f new_pos(0, 0);
        float max_radius_obs = 0.0f;
        for (size_t idx : associated_points)
        {
          Eigen::Vector2f p(candidate_points[idx].x(), candidate_points[idx].y());
          new_pos += p;
          float r = (p - prev_state_[id].pos).norm();
          max_radius_obs = std::max(max_radius_obs, r);
        }
        new_pos /= static_cast<float>(associated_points.size());
        float final_radius = std::min(max_radius_obs + inflation, max_radius);

        geometry_msgs::msg::PointStamped in, out;
        in.header = msg->header;
        in.point.x = new_pos.x();
        in.point.y = new_pos.y();
        in.point.z = 0.0;

        try
        {
          tf_buffer_.transform(in, out, fixed_frame, tf2::durationFromSec(0.2));
          Eigen::Vector2f center_fixed(out.point.x, out.point.y);
          
          updateObstacleState(id, center_fixed, final_radius, current_time, fixed_frame, 
                            min_dynamic_vel, static_obs_time, associated_points.size());
          detected_ids.push_back(id);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed: %s", ex.what());
        }
      }
    }

    for (size_t i = 0; i < candidate_points.size(); ++i)
    {
      if (point_used[i]) continue;

      std::vector<size_t> new_group;
      new_group.push_back(i);
      point_used[i] = true;

      for (size_t j = i + 1; j < candidate_points.size(); ++j)
      {
        if (point_used[j]) continue;
        Eigen::Vector2f p1(candidate_points[i].x(), candidate_points[i].y());
        Eigen::Vector2f p2(candidate_points[j].x(), candidate_points[j].y());
        if ((p1 - p2).norm() < assoc_dist)
        {
          new_group.push_back(j);
          point_used[j] = true;
        }
      }

      if ((int)new_group.size() >= min_points)
      {
        Eigen::Vector2f new_pos(0, 0);
        float max_radius_obs = 0.0f;
        for (size_t idx : new_group)
        {
          Eigen::Vector2f p(candidate_points[idx].x(), candidate_points[idx].y());
          new_pos += p;
          float r = p.norm();
          max_radius_obs = std::max(max_radius_obs, r);
        }
        new_pos /= static_cast<float>(new_group.size());
        float final_radius = std::min(max_radius_obs + inflation, max_radius);

        int new_id = next_obstacle_id_++;
        geometry_msgs::msg::PointStamped in, out;
        in.header = msg->header;
        in.point.x = new_pos.x();
        in.point.y = new_pos.y();
        in.point.z = 0.0;

        try
        {
          tf_buffer_.transform(in, out, fixed_frame, tf2::durationFromSec(0.2));
          Eigen::Vector2f center_fixed(out.point.x, out.point.y);
          
          // RCLCPP_INFO(get_logger(), "New obstacle ID %d at (%.2f, %.2f) with %zu points", 
          //             new_id, center_fixed.x(), center_fixed.y(), new_group.size());
          
          updateObstacleState(new_id, center_fixed, final_radius, current_time, fixed_frame, 
                            min_dynamic_vel, static_obs_time, new_group.size());
          detected_ids.push_back(new_id);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed for new obstacle: %s", ex.what());
        }
      }
    }

    for (size_t i = 0; i < prev_ids.size(); ++i)
    {
      if (std::find(detected_ids.begin(), detected_ids.end(), prev_ids[i]) == detected_ids.end())
      {
        if (prev_state_.find(prev_ids[i]) != prev_state_.end())
          prev_state_[prev_ids[i]].missed_frames++;
      }
    }

    cleanExpiredObstacles();
    publishAllObstacles(msg->header, fixed_frame, current_time);

    if (!disable_filtering)
    {
      std::vector<std::pair<Eigen::Vector2f, float>> dynamic_obstacles_to_filter;
      for (const auto &[id, state] : prev_state_)
      {
        if (state.is_dynamic)
        {
          dynamic_obstacles_to_filter.emplace_back(state.pos, state.radius);
        }
      }
      
      if (!dynamic_obstacles_to_filter.empty())
      {
        publishFilteredCloud(msg, dynamic_obstacles_to_filter, msg->header.frame_id, fixed_frame);
      }
      else
      {
        filtered_cloud_pub_->publish(*msg);
      }
    }
    else
    {
      filtered_cloud_pub_->publish(*msg);
    }
  }

  void updateObstacleState(int id, const Eigen::Vector2f &pos, float radius, 
                          const rclcpp::Time &stamp, const std::string &frame_id,
                          double min_dynamic_vel, double static_obs_time, int point_count)
  {
    Eigen::Vector2f velocity(0.0f, 0.0f);
    rclcpp::Time first_detect = stamp;
    float max_vel = 0.0f;

    if (prev_state_.count(id))
    {
      auto &prev = prev_state_[id];
      double dt = (stamp - prev.stamp).seconds();
      first_detect = prev.first_detected;
      max_vel = prev.max_observed_velocity;

      if (dt > 0.02 && dt < 1.5)
      {
        velocity.x() = (pos.x() - prev.pos.x()) / dt;
        velocity.y() = (pos.y() - prev.pos.y()) / dt;

        velocity.x() = std::clamp(velocity.x(), -5.0f, 5.0f);
        velocity.y() = std::clamp(velocity.y(), -5.0f, 5.0f);
        
        velocity = 0.7f * velocity + 0.3f * prev.velocity;
        
        float current_speed = velocity.norm();
        if (current_speed > max_vel)
          max_vel = current_speed;
      }
      else
      {
        velocity = prev.velocity * 0.9f;
      }
    }

    double observation_duration = (stamp - first_detect).seconds();
    bool is_dynamic = true;

    if (observation_duration > static_obs_time)
    {
      if (max_vel < min_dynamic_vel)
      {
        is_dynamic = false;
      }
    }

    prev_state_[id] = {pos, velocity, radius, stamp, stamp, first_detect, 0, frame_id, 
                       is_dynamic, max_vel, point_count};
  }

  void cleanExpiredObstacles()
  {
    std::vector<int> to_remove;
    
    for (const auto &[id, state] : prev_state_)
    {
      if (state.missed_frames > 5)
      {
        to_remove.push_back(id);
      }
    }

    for (int id : to_remove)
    {
      prev_state_.erase(id);
    }
  }

  void publishAllObstacles(const std_msgs::msg::Header &header, 
                        const std::string &fixed_frame,
                        const rclcpp::Time &current_time)
  {
    costmap_converter_msgs::msg::ObstacleArrayMsg obs_array;
    costmap_converter_msgs::msg::ObstacleArrayMsg traj_array;
    
    std_msgs::msg::Header fixed_header;
    fixed_header.frame_id = fixed_frame;
    fixed_header.stamp = current_time;
    
    obs_array.header = fixed_header;
    traj_array.header = fixed_header;

    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto &[id, state] : prev_state_)
    {
      // **ONLY PUBLISH DYNAMIC** obstacles to costmap converter
      if (state.is_dynamic)
      {
        costmap_converter_msgs::msg::ObstacleMsg obs;
        obs.id = id;
        obs.radius = state.radius;
        
        // ✅ FIXED: Set proper frame_id and timestamp
        obs.header.frame_id = fixed_frame;
        obs.header.stamp = current_time;

        geometry_msgs::msg::Point32 p;
        p.x = state.pos.x();
        p.y = state.pos.y();
        p.z = 0.0f;
        obs.polygon.points.push_back(p);

        geometry_msgs::msg::TwistWithCovariance vel_cov;
        vel_cov.twist.linear.x = state.velocity.x();
        vel_cov.twist.linear.y = state.velocity.y();
        vel_cov.twist.linear.z = 0.0;
        obs.velocities = vel_cov;
        obs_array.obstacles.push_back(obs);

        // Trajectory with proper frame_id
        const double T = 3.0;
        const double dt = 0.2;
        for (double t = dt; t <= T; t += dt)
        {
          costmap_converter_msgs::msg::ObstacleMsg future;
          future.id = id * 1000 + (int)(t/dt);  // Unique ID for trajectory points
          future.radius = state.radius;
          future.header.frame_id = fixed_frame;
          future.header.stamp = current_time;

          geometry_msgs::msg::Point32 fp;
          fp.x = state.pos.x() + state.velocity.x() * t;
          fp.y = state.pos.y() + state.velocity.y() * t;
          fp.z = 0.0;
          future.polygon.points.push_back(fp);
          
          future.velocities = vel_cov;
          traj_array.obstacles.push_back(future);
        }
      }

      // Visualize ALL obstacles (dynamic + static)
      createObstacleMarker(marker_array, fixed_header, id, state);
    }

    obstacle_pub_->publish(obs_array);
    obstacle_traj_pub_->publish(traj_array);
    marker_pub_->publish(marker_array);
  }

  void createObstacleMarker(visualization_msgs::msg::MarkerArray &array,
                           const std_msgs::msg::Header &header,
                           int id,
                           const ObstacleState &state)
  {
    const double min_vel = get_parameter("min_velocity_display").as_double();

    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "obstacles";
    sphere.id = id;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;

    sphere.pose.position.x = state.pos.x();
    sphere.pose.position.y = state.pos.y();
    sphere.pose.position.z = state.radius;
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = state.radius * 2.0;
    sphere.scale.y = state.radius * 2.0;
    sphere.scale.z = state.radius * 2.0;

    if (state.is_dynamic)
    {
      float hue = (id * 0.2f) - std::floor(id * 0.2f);
      sphere.color.r = std::max(0.3f, hue);
      sphere.color.g = std::max(0.3f, 1.0f - hue);
      sphere.color.b = 0.3f;
      sphere.color.a = 0.6f;
    }
    else
    {
      sphere.color.r = 0.5f;
      sphere.color.g = 0.5f;
      sphere.color.b = 0.5f;
      sphere.color.a = 0.4f;
    }

    sphere.lifetime = rclcpp::Duration::from_seconds(0.2);
    array.markers.push_back(sphere);

    float vel_norm = state.velocity.norm();
    if (state.is_dynamic && vel_norm > min_vel)
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header = header;
      arrow.ns = "velocity_arrows";
      arrow.id = id + 10000;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point a, b;
      a.x = state.pos.x();
      a.y = state.pos.y();
      a.z = state.radius * 1.5;

      double arrow_scale = get_parameter("velocity_arrow_scale").as_double();
      b.x = state.pos.x() + state.velocity.x() * arrow_scale;
      b.y = state.pos.y() + state.velocity.y() * arrow_scale;
      b.z = state.radius * 1.5;

      arrow.points = {a, b};
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.3;

      arrow.color.r = 1.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0f;

      arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
      // array.markers.push_back(arrow);
    }
  }

  void publishFilteredCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input,
                            const std::vector<std::pair<Eigen::Vector2f, float>> &obstacles,
                            const std::string &sensor_frame,
                            const std::string &fixed_frame)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    output.height = 1;
    output.is_dense = false;
    output.is_bigendian = input->is_bigendian;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(input->width * input->height);

    sensor_msgs::PointCloud2Iterator<float> ox(output, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(output, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(output, "z");

    sensor_msgs::PointCloud2ConstIterator<float> ix(*input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*input, "z");

    std::vector<std::pair<Eigen::Vector2f, float>> obstacles_in_sensor;
    for (const auto &[pos_fixed, radius] : obstacles)
    {
      geometry_msgs::msg::PointStamped in, out;
      in.header.frame_id = fixed_frame;
      in.header.stamp = input->header.stamp;
      in.point.x = pos_fixed.x();
      in.point.y = pos_fixed.y();
      in.point.z = 0.0;

      try
      {
        tf_buffer_.transform(in, out, sensor_frame, tf2::durationFromSec(0.1));
        Eigen::Vector2f pos_sensor(out.point.x, out.point.y);
        obstacles_in_sensor.emplace_back(pos_sensor, radius);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Transform failed: %s", ex.what());
      }
    }

    size_t kept = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz)
    {
      bool inside_obstacle = false;
      for (const auto &[center, radius] : obstacles_in_sensor)
      {
        float distance = std::hypot(*ix - center.x(), *iy - center.y());
        if (distance <= radius)
        {
          inside_obstacle = true;
          break;
        }
      }

      if (!inside_obstacle)
      {
        *ox = *ix;
        *oy = *iy;
        *oz = *iz;
        ++ox; ++oy; ++oz;
        kept++;
      }
    }

    modifier.resize(kept);
    output.width = kept;
    filtered_cloud_pub_->publish(output);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  try
  {
    auto node = std::make_shared<DynamicObstacleProcessor>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
*/
