// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <cmath>

// class LidarZoneBinaryNode : public rclcpp::Node
// {
// public:
//   LidarZoneBinaryNode() : Node("lidar_zone_binary_node")
//   {
//     sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
//       "/ouster/points",
//       rclcpp::SensorDataQoS(),
//       std::bind(&LidarZoneBinaryNode::cloudCallback, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Lidar zone binary node started (no ROS output)");
//   }

// private:
//   void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//   {
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::fromROSMsg(*msg, cloud);

//     bool left = false;
//     bool front = false;
//     bool right = false;

//     for (const auto &pt : cloud.points)
//     {
//       float range = std::sqrt(pt.x * pt.x + pt.y * pt.y);

//       // Distance limits: 200 mm – 3 m
//       if (range < 0.2 || range > 3.0)
//         continue;

//       float angle = std::atan2(pt.y, pt.x) * 180.0 / M_PI;

//       // Consider only 0–180 degrees
//       if (angle < 0.0 || angle > 180.0)
//         continue;

//       if (angle <= 60.0)
//         left = true;
//       else if (angle <= 120.0)
//         front = true;
//       else
//         right = true;

//       if (left && front && right)
//         break;
//     }

//     // Convert to binary output
//     int binary =
//         (left  ? 4 : 0) |
//         (front ? 2 : 0) |
//         (right ? 1 : 0);

//     // Print as 3-bit binary
//     RCLCPP_INFO_THROTTLE(
//       get_logger(), *get_clock(), 500,
//       "Lidar zone state: %d%d%d",
//       left ? 1 : 0,
//       front ? 1 : 0,
//       right ? 1 : 0
//     );

//     // 👇 USE THIS VARIABLE ANYWHERE YOU WANT
//     last_binary_state_ = binary;
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
//   int last_binary_state_{0};
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<LidarZoneBinaryNode>());
//   rclcpp::shutdown();
//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// ==================== CONFIG ====================
static constexpr float OBS_MM_THRESHOLD = 200.0f;   // >= 200 mm = obstacle

// Robot-relative angular sectors (radians)
static constexpr float CENTER_MIN = -0.87f;  // -50°
static constexpr float CENTER_MAX =  0.87f;  // +50°

static constexpr float LEFT_MIN   =  0.87f;  // +50°
static constexpr float LEFT_MAX   =  1.74f;  // +100°

static constexpr float RIGHT_MIN  = -1.74f;  // -100°
static constexpr float RIGHT_MAX  = -0.87f;  // -50°

// ==================== SHM STRUCT ====================
struct ObstacleFlags
{
    uint8_t left;
    uint8_t center;
    uint8_t right;
};

static ObstacleFlags* shm_obs = nullptr;

// ==================== NODE ====================
class OdafLidarNode : public rclcpp::Node
{
public:
    OdafLidarNode() : Node("odaf_lidar_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points",
            rclcpp::SensorDataQoS(),
            std::bind(&OdafLidarNode::cloudCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ODAF lidar node started - front 180° FOV");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Reset flags every frame
        shm_obs->left   = 0;
        shm_obs->center = 0;
        shm_obs->right  = 0;

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");

        for (; it_x != it_x.end(); ++it_x, ++it_y)
        {
            float x = *it_x;
            float y = *it_y;

            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y))
                continue;

            // Front-only (x forward)
            if (x <= 0.0f)
                continue;

            // Distance check (meters → mm)
            float dist_mm = std::sqrt(x * x + y * y) * 1000.0f;
            if (dist_mm < OBS_MM_THRESHOLD)
                continue;

            // Robot-relative angle
            float angle = std::atan2(y, x);

            // HARD front FOV guard (±90°)
            if (angle < -M_PI_2 || angle > M_PI_2)
                continue;

            // Sector classification
            if (angle >= CENTER_MIN && angle <= CENTER_MAX)
                shm_obs->center = 1;
            else if (angle > LEFT_MIN && angle <= LEFT_MAX)
                shm_obs->left = 1;
            else if (angle >= RIGHT_MIN && angle <= RIGHT_MAX)
                shm_obs->right = 1;

            // Early exit
            if (shm_obs->left && shm_obs->center && shm_obs->right)
                break;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "ODAF | L:%u C:%u R:%u",
            shm_obs->left,
            shm_obs->center,
            shm_obs->right
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

// ==================== MAIN ====================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ---------- Shared Memory Setup ----------
    int shm_fd = shm_open("/odaf_obs", O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0)
    {
        perror("shm_open failed");
        return 1;
    }

    if (ftruncate(shm_fd, sizeof(ObstacleFlags)) != 0)
    {
        perror("ftruncate failed");
        close(shm_fd);
        return 1;
    }

    shm_obs = static_cast<ObstacleFlags*>(
        mmap(nullptr,
             sizeof(ObstacleFlags),
             PROT_READ | PROT_WRITE,
             MAP_SHARED,
             shm_fd,
             0)
    );

    if (shm_obs == MAP_FAILED)
    {
        perror("mmap failed");
        close(shm_fd);
        return 1;
    }

    shm_obs->left = shm_obs->center = shm_obs->right = 0;

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shared memory /odaf_obs ready");

    // ---------- ROS Node ----------
    auto node = std::make_shared<OdafLidarNode>();
    rclcpp::spin(node);

    // ---------- Cleanup ----------
    munmap(shm_obs, sizeof(ObstacleFlags));
    close(shm_fd);
    shm_unlink("/odaf_obs");

    rclcpp::shutdown();
    return 0;
}

