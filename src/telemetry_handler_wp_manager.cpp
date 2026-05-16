/*
this code is working correctly, but the gps to map conversion is wrong
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* ================= EP PROTOCOL ================= */
static constexpr uint8_t EP_SOF1 = 0xAA;
static constexpr uint8_t EP_SOF2 = 0x55;
static constexpr uint8_t EP_VER  = 0x01;

static constexpr uint16_t CRC_POLY = 0x1021;
static constexpr size_t READ_BUFFER_SIZE = 512; // bigger buffer to avoid desync

/* AGX -> EDGE */
static constexpr uint8_t EP_MSG_STATE = 0x10;  // AGX -> edge_pilot (len=18)

/* EDGE -> AGX (match your edge_pilot.c) */
static constexpr uint8_t EP_MSG_ARM       = 0x20;
static constexpr uint8_t EP_MSG_DISARM    = 0x21;
static constexpr uint8_t EP_MSG_GOAL      = 0x21; // payload: 8 bytes (lat_e7 i32 + lon_e7 i32) [optional]
static constexpr uint8_t EP_MSG_SET_MODE  = 0x22; // payload: 1 byte
static constexpr uint8_t EP_MSG_WAYPOINT  = 0x23; // payload: 8 bytes (lat_e6 i32 + lon_e6 i32)
static constexpr uint8_t EP_MSG_SET_BASE  = 0x24; // payload: 0 bytes

/* RAW COMMANDS (your “single byte” local testing) */
static constexpr uint8_t CMD_SET_MODE_RAW      = 0x20; // +1 byte mode
static constexpr uint8_t CMD_ARM_SIMPLE        = 0x0A;
static constexpr uint8_t CMD_DISARM_SIMPLE     = 0x0B;
static constexpr uint8_t CMD_WAYPOINT_RAW      = 0x25; // +8 bytes
static constexpr uint8_t CMD_SET_BASE_RAW      = 0x26;

/* modes */
static constexpr uint8_t EP_MODE_HOTAS   = 0;
static constexpr uint8_t EP_MODE_SHADOW  = 1;
static constexpr uint8_t EP_MODE_PURSUIT = 2;
static constexpr uint8_t EP_MODE_BASE    = 3;

/* status */
static constexpr uint8_t EP_STAT_MOVING    = 0;
static constexpr uint8_t EP_STAT_RECOVERY  = 1;

static constexpr uint8_t EP_MSG_MODE_HOTAS   = 0x30;  // NEW: no payload
static constexpr uint8_t EP_MSG_MODE_SHADOW  = 0x31;  // NEW: no payload  
static constexpr uint8_t EP_MSG_MODE_PURSUIT = 0x32;  // NEW: no payload
static constexpr uint8_t EP_MSG_MODE_BASE    = 0x33;  // NEW: no payload

int32_t gpscb_lat_e7{0};
int32_t gpscb_lon_e7{0};

bool is_arming_valid_{false};
int32_t armed_lat_e7_{0};
int32_t armed_lon_e7_{0};
bool mode_pursuit_{false};
bool mode_rtb_{false};
bool set_waypoints_{false};
bool base_set_{false};
int32_t base_lat_e7_{0};
int32_t base_lon_e7_{0};
int32_t wp_lat_e6_{0};
int32_t wp_lon_e6_{0};

static inline int32_t clamp_i32(int64_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return static_cast<int32_t>(v);
}

static inline int16_t clamp_i16(int32_t v)
{
  if (v < -32768) return static_cast<int16_t>(-32768);
  if (v >  32767) return static_cast<int16_t>( 32767);
  return static_cast<int16_t>(v);
}

static inline uint16_t clamp_u16(int32_t v)
{
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return static_cast<uint16_t>(v);
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0x0000)
{
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = static_cast<uint16_t>((crc << 1) ^ CRC_POLY);
      else             crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static void append_le_i16(std::vector<uint8_t> &b, int16_t v)
{
  uint16_t u = static_cast<uint16_t>(v);
  b.push_back(static_cast<uint8_t>(u & 0xFF));
  b.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
}

static void append_le_u16(std::vector<uint8_t> &b, uint16_t v)
{
  b.push_back(static_cast<uint8_t>(v & 0xFF));
  b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

static void append_le_i32(std::vector<uint8_t> &b, int32_t v)
{
  uint32_t u = static_cast<uint32_t>(v);
  b.push_back(static_cast<uint8_t>(u & 0xFF));
  b.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
  b.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
  b.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
}

static uint16_t rd_u16_le(const uint8_t *p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}
static int16_t rd_i16_le(const uint8_t *p) { return static_cast<int16_t>(rd_u16_le(p)); }

static uint32_t rd_u32_le(const uint8_t *p)
{
  return static_cast<uint32_t>(static_cast<uint32_t>(p[0]) |
                               (static_cast<uint32_t>(p[1]) << 8) |
                               (static_cast<uint32_t>(p[2]) << 16) |
                               (static_cast<uint32_t>(p[3]) << 24));
}
static int32_t rd_i32_le(const uint8_t *p) { return static_cast<int32_t>(rd_u32_le(p)); }

class AgxEpTelemetrySender : public rclcpp::Node
{
public:
  AgxEpTelemetrySender()
  : Node("agx_ep_telemetry_sender"),
    rng_(std::random_device{}()),
    read_buffer_{},
    read_buffer_pos_(0)
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_        = declare_parameter<int>("baud", 115200);

    // IMPORTANT: sane default (edge user-space RX can’t keep up with 1000Hz reliably)
    rate_hz_     = declare_parameter<double>("rate_hz", 50.0);

    navsat_topic_ = declare_parameter<std::string>("navsat_topic", "/navsat/fix");
    odom_topic_   = declare_parameter<std::string>("odom_topic", "/odom");
    cmdvel_topic_ = declare_parameter<std::string>("cmdvel_topic", "/cmd_vel");

    open_serial();

    start_nav_pub_ = create_publisher<std_msgs::msg::Empty>(
      "start_navigation", rclcpp::QoS(10));

    stop_nav_pub_ = create_publisher<std_msgs::msg::Empty>(
      "stop_navigation", rclcpp::QoS(10));

    rtb_pub_ = create_publisher<std_msgs::msg::Empty>(
      "rtb", rclcpp::QoS(10));

    flush_wp_pub_ = create_publisher<std_msgs::msg::Empty>(
      "flush_waypoints", rclcpp::QoS(10));

    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS(10));

    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      navsat_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg){ on_navsat(*msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg){ on_odom(*msg); });

    cmdvel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmdvel_topic_, rclcpp::QoS(20),
      [this](geometry_msgs::msg::Twist::SharedPtr msg){ on_cmdvel(*msg); });

    mode_ = EP_MODE_HOTAS;
    arm_mode_ = 0;

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this](){ tick_send(); });

    RCLCPP_INFO(get_logger(),
      "AGX EP telemetry up. UART=%s @%d, tx_rate=%.1fHz. RX parsers: mode/set_base/waypoint/arm, mode=%u",
      serial_port_.c_str(), baud_, rate_hz_, mode_);
      
  }

  ~AgxEpTelemetrySender() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void open_serial()
  {
    fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", serial_port_.c_str(), strerror(errno));
      return;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= static_cast<tcflag_t>(CS8);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    speed_t spd = B115200;
    if (baud_ == 9600) spd = B9600;
    else if (baud_ == 19200) spd = B19200;
    else if (baud_ == 38400) spd = B38400;
    else if (baud_ == 57600) spd = B57600;
    else if (baud_ == 115200) spd = B115200;

    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

    tcflush(fd_, TCIOFLUSH);
  }

  void on_navsat(const sensor_msgs::msg::NavSatFix &msg)
  {
    if (msg.status.status < -1) { have_fix_ = false; return; }
    if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude)) { have_fix_ = false; return; }

    const int64_t lat_e7 = static_cast<int64_t>(std::llround(msg.latitude  * 10000000.0));
    const int64_t lon_e7 = static_cast<int64_t>(std::llround(msg.longitude * 10000000.0));

    lat_e7_ = clamp_i32(lat_e7, INT32_MIN, INT32_MAX);
    lon_e7_ = clamp_i32(lon_e7, INT32_MIN, INT32_MAX);
    gpscb_lat_e7 = lat_e7;
    gpscb_lon_e7 = lon_e7;
    have_fix_ = true;
    // RCLCPP_INFO(get_logger(),"lat_e7=%.7f lon_e7=%.7f gpscb_lat_e7=%d gpscb_lon_e7=%d",lat_e7_, lon_e7_, gpscb_lat_e7, gpscb_lon_e7);
  }

  void on_odom(const nav_msgs::msg::Odometry &msg)
  {
    const auto &q = msg.pose.pose.orientation;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

    double yaw_deg = yaw * (180.0 / M_PI);
    while (yaw_deg < 0.0) yaw_deg += 360.0;
    while (yaw_deg >= 360.0) yaw_deg -= 360.0;

    heading_cdeg_ = clamp_u16(static_cast<int32_t>(std::lround(yaw_deg * 100.0)));
    have_heading_ = true;
  }

  void on_cmdvel(const geometry_msgs::msg::Twist &msg)
  {
    int32_t lin_mm_s = static_cast<int32_t>(std::lround(msg.linear.x * 1000.0));
    int32_t ang_mrad_s = static_cast<int32_t>(std::lround(msg.angular.z * 1000.0));

    linear_x_mm_s_ = clamp_i16(lin_mm_s);
    angular_z_mrad_s_ = clamp_i16(ang_mrad_s);
    have_cmd_ = true;
  }

  void tick_send()
  {
    readSerialCommands();

    if (fd_ < 0) {
      static int retry = 0;
      if ((retry++ % 50) == 0) open_serial();
      return;
    }

    const uint8_t status = (std::abs(linear_x_mm_s_) > 10 || std::abs(angular_z_mrad_s_) > 10)
      ? EP_STAT_MOVING
      : (arm_mode_ ? EP_STAT_RECOVERY : 0);

    // your original constant pitch
    const int16_t pitch_cdeg = 900;

    std::vector<uint8_t> buf;
    buf.reserve(6 + 18 + 2);

    buf.push_back(EP_SOF1);
    buf.push_back(EP_SOF2);
    buf.push_back(EP_VER);
    buf.push_back(EP_MSG_STATE);
    buf.push_back(seq_++);
    buf.push_back(18);

    buf.push_back(mode_);
    buf.push_back(status);
    append_le_i32(buf, lat_e7_);
    append_le_i32(buf, lon_e7_);
    append_le_i16(buf, pitch_cdeg);
    append_le_u16(buf, heading_cdeg_);
    append_le_i16(buf, linear_x_mm_s_);
    append_le_i16(buf, angular_z_mrad_s_);

    uint16_t crc = crc16_ccitt(&buf[2], buf.size() - 2, 0x0000);
    buf.push_back(static_cast<uint8_t>(crc & 0xFF));
    buf.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    write_all(buf.data(), buf.size());
  }

  /* ================= RX (RAW + FRAMED) ================= */

  void readSerialCommands()
  {
    if (fd_ < 0) return;

    // read as much as fits
    if (read_buffer_pos_ >= read_buffer_.size()) {
      // overflow safety: drop
      read_buffer_pos_ = 0;
    }

    ssize_t n = ::read(fd_, read_buffer_.data() + read_buffer_pos_,
                       static_cast<int>(read_buffer_.size() - read_buffer_pos_));
    if (n <= 0) return;

    read_buffer_pos_ += static_cast<size_t>(n);

    // 1) RAW COMMAND PARSER (single-byte style)
    while (read_buffer_pos_ > 0) {
      uint8_t cmd = read_buffer_[0];

      if (cmd == CMD_SET_MODE_RAW && read_buffer_pos_ >= 2) {
        uint8_t new_mode = read_buffer_[1];
        RCLCPP_INFO(get_logger(),
          "🎮 RAW SET_MODE: %u → %u | armed=%u base_set=%s waypoint_set=%s",
          mode_, new_mode,
          arm_mode_,
          set_base_requested_ ? "YES" : "NO",
          have_waypoint_ ? "YES" : "NO");
        mode_ = new_mode;
        // RCLCPP_INFO(get_logger(), "🎮 RAW SET_MODE 0x20: mode=%u", (unsigned)new_mode);
        consume(2);
        continue;
      }

      if (cmd == 0x0A) {
        arm_mode_ = 1;
        if (!is_arming_valid_ && have_fix_) {
          armed_lat_e7_ = lat_e7_;
          armed_lon_e7_ = lon_e7_;
          is_arming_valid_ = true;

          RCLCPP_INFO(get_logger(),
            "📌 ARM GPS LATCHED: lat=%.7f lon=%.7f",
            armed_lat_e7_ / 1e7, armed_lon_e7_ / 1e7);
        }
        RCLCPP_INFO(get_logger(),
          "🟢 RAW ARM: armed=%u → 1 | base_set=%s waypoint_set=%s mode=%u",
          arm_mode_,
          set_base_requested_ ? "YES" : "NO",
          have_waypoint_ ? "YES" : "NO",
          mode_);
        // RCLCPP_INFO(get_logger(), "🟢 RAW ARM 0x0A");
        consume(1);
        continue;
      }

      if (cmd == 0x0B) {
        arm_mode_ = 0;
        is_arming_valid_ = false;
        RCLCPP_INFO(get_logger(),
          "🔴 RAW DISARM: armed=%u → 0 | base_set=%s waypoint_set=%s mode=%u",
          arm_mode_,
          set_base_requested_ ? "YES" : "NO",
          have_waypoint_ ? "YES" : "NO",
          mode_);
        // RCLCPP_INFO(get_logger(), "🔴 RAW DISARM 0x0B");
        consume(1);
        continue;
      }

      if (cmd == CMD_WAYPOINT_RAW && read_buffer_pos_ >= 9) {
        int32_t lat_e6 = rd_i32_le(&read_buffer_[1]);
        int32_t lon_e6 = rd_i32_le(&read_buffer_[5]);

        have_waypoint_ = true;
        wp_lat_e6_ = lat_e6;
        wp_lon_e6_ = lon_e6;

        RCLCPP_INFO(get_logger(),
          "📍 RAW WAYPOINT SET: lat=%.6f lon=%.6f | armed=%u mode=%u base_set=%s",
          lat_e6 / 1e6, lon_e6 / 1e6,
          arm_mode_, mode_,
          set_base_requested_ ? "YES" : "NO");

        // RCLCPP_INFO(get_logger(), "📍 RAW WAYPOINT 0x25: lat=%.6f lon=%.6f",
        //             lat_e6 / 1e6, lon_e6 / 1e6);
        consume(9);
        continue;
      }

      if (cmd == 0x26) {
        set_base_requested_ = true;
        RCLCPP_INFO(get_logger(),
          "🏠 RAW BASE SET | armed=%u mode=%u waypoint_set=%s",
          arm_mode_, mode_,
          have_waypoint_ ? "YES" : "NO");
        //RCLCPP_INFO(get_logger(), "🏠 RAW SET_BASE 0x26");
        consume(1);
        continue;
      }

      break; // not a raw command => move to framed parser
    }

    // 2) FRAMED EP PARSER (proper protocol)
    for (;;) {
      // Need at least header (6) + crc(2)
      if (read_buffer_pos_ < 8) break;

      // find SOF
      size_t sof = find_sof();
      if (sof > 0) consume(sof);
      if (read_buffer_pos_ < 8) break;

      // verify SOF
      if (read_buffer_[0] != EP_SOF1 || read_buffer_[1] != EP_SOF2) {
        consume(1);
        continue;
      }

      // header fields
      const uint8_t ver = read_buffer_[2];
      const uint8_t type = read_buffer_[3];
      const uint8_t seq  = read_buffer_[4];
      const uint8_t len  = read_buffer_[5];

      (void)seq;

      const size_t frame_len = static_cast<size_t>(6) + static_cast<size_t>(len) + 2;
      if (frame_len > 1024) { // sanity
        consume(2);
        continue;
      }

      if (read_buffer_pos_ < frame_len) break; // wait for full frame

      if (ver != EP_VER) {
        consume(2);
        continue;
      }

      // CRC check: over [VER..payload]
      const uint16_t calc = crc16_ccitt(&read_buffer_[2], static_cast<size_t>(4) +  static_cast<size_t>(len), 0x0000);
      const uint16_t pkt  = static_cast<uint16_t>(read_buffer_[frame_len - 2]) |
                            (static_cast<uint16_t>(read_buffer_[frame_len - 1]) << 8);
      if (calc != pkt) {
        // bad CRC => desync one byte
        consume(1);
        continue;
      }

      const uint8_t *payload = &read_buffer_[6];

      if (type == 0x0A && len == 0) {
        arm_mode_ = 1;
        RCLCPP_INFO(get_logger(),"Armed");
        if (!is_arming_valid_ && have_fix_) {
            armed_lat_e7_ = lat_e7_;
            armed_lon_e7_ = lon_e7_;
            is_arming_valid_ = true;

            RCLCPP_INFO(get_logger(),
              "📌 ARM GPS LATCHED (FRAMED): lat=%.7f lon=%.7f",
              armed_lat_e7_ / 1e7, armed_lon_e7_ / 1e7);
          }
          if(is_arming_valid_ && mode_ == EP_MODE_PURSUIT){
            start_nav_pub_->publish(std_msgs::msg::Empty());
          }
          else{
            RCLCPP_WARN(get_logger(),
              "Cannot start navigation: invalid arming or not in PURSUIT mode (armed=%u mode=%u)",
              is_arming_valid_ ? 1 : 0,
              mode_);
          }
      } 
      else if (type == 0X0B && len == 0) {
        arm_mode_ = 0;
        is_arming_valid_ = false;
        RCLCPP_INFO(get_logger(), "DisArmed");
        stop_nav_pub_->publish(std_msgs::msg::Empty());
      }
      else if (type == EP_MSG_MODE_HOTAS && len == 0) {
        mode_ = EP_MODE_HOTAS;
        mode_pursuit_ = false;
        mode_rtb_ = false;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE HOTAS (0x30), mode :%u ", mode_);
      }
      else if (type == EP_MSG_MODE_SHADOW && len == 0) {
        mode_ = EP_MODE_SHADOW;
        mode_pursuit_ = false;
        mode_rtb_ = false;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE SHADOW (0x31), mode :%u ", mode_);
      }
      else if (type == EP_MSG_MODE_PURSUIT && len == 0) {
        mode_ = EP_MODE_PURSUIT;
        mode_pursuit_ = true;
        mode_rtb_ = false;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE PURSUIT (0x32), mode :%u ", mode_);
      }
      else if (type == EP_MSG_MODE_BASE && len == 0) {
        mode_ = EP_MODE_BASE;
        mode_pursuit_ = false;
        mode_rtb_ = true;
        rtb_pub_->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE BASE (0x33), mode :%u ", mode_);
      }
      // Keep existing for backward compatibility
      else if (type == EP_MSG_SET_MODE && len == 1) {
        mode_ = payload[0];
        RCLCPP_INFO(get_logger(), "🎮 FRAMED SET_MODE: %u", mode_);
      }
      // else if (type == EP_MSG_SET_MODE && len == 1) {
      //   mode_ = payload[0];
      //    RCLCPP_INFO(get_logger(),
      //     "🎮 FRAMED SET_MODE: %u | armed=%u base_set=%s waypoint_set=%s",
      //     mode_, 
      //     arm_mode_,
      //     set_base_requested_ ? "YES" : "NO",
      //     have_waypoint_ ? "YES" : "NO");
      //   // RCLCPP_INFO(get_logger(), "🎮 FRAMED SET_MODE 0x22: mode=%u", (unsigned)mode_);
      // } 
      else if (type == EP_MSG_WAYPOINT && len == 8) {
        set_waypoints_ = true;
        int32_t lat_e6 = rd_i32_le(payload);
        int32_t lon_e6 = rd_i32_le(payload + 4);
        have_waypoint_ = true;
        wp_lat_e6_ = lat_e6;
        wp_lon_e6_ = lon_e6;
        RCLCPP_INFO(get_logger(),
          "📍 FRAMED WAYPOINT SET: lat=%.6f lon=%.6f | wp_lat=%d wp_lon=%d armed=%u mode=%u base_set=%s",
          lat_e6 / 1e6, lon_e6 / 1e6,
          wp_lat_e6_, wp_lon_e6_,
          arm_mode_, mode_,
          set_base_requested_ ? "YES" : "NO");

        geometry_msgs::msg::PoseStamped waypoint_poses;
        waypoint_poses.pose.position.x = wp_lat_e6_ / 1e6;
        waypoint_poses.pose.position.y = wp_lon_e6_ / 1e6;
        waypoint_poses.pose.position.z = 0.0;
        waypoint_poses.pose.orientation.w = 1.0;  
        goal_pose_pub_->publish(waypoint_poses);
        RCLCPP_INFO(get_logger(), "WAYPOINT lat=%.6f lon=%.6f",
                    waypoint_poses.pose.position.x, waypoint_poses.pose.position.y);
      } else if (type == EP_MSG_SET_BASE && len == 0) {
        set_base_requested_ = true;
        base_set_ = true;
        base_lat_e7_ = lat_e7_;
        base_lon_e7_ = lon_e7_;
        RCLCPP_INFO(get_logger(),
          "🏠 FRAMED BASE SET | armed=%u mode=%u waypoint_set=%s | base_lat=%.7f base_lon=%.7f",
          arm_mode_, mode_,
          have_waypoint_ ? "YES" : "NO",
          base_lat_e7_ / 1e7, base_lon_e7_ / 1e7);
        //RCLCPP_INFO(get_logger(), "🏠 FRAMED SET_BASE 0x24");
      } else if (type == EP_MSG_GOAL && len == 8) {
        // optional if you ever use it
        int32_t lat_e7 = rd_i32_le(payload);
        int32_t lon_e7 = rd_i32_le(payload + 4);
        RCLCPP_INFO(get_logger(), "🎯 FRAMED GOAL 0x21: lat=%.7f lon=%.7f",
                    lat_e7 / 1e7, lon_e7 / 1e7);
      }else if (type == 0x69 && len == 0) {
        RCLCPP_INFO(get_logger(), "Flush Waypoints received, all waypoints should be cleared");
        flush_wp_pub_->publish(std_msgs::msg::Empty());
      }
      else {
        RCLCPP_DEBUG(get_logger(), "FRAMED RX unknown type=0x%02X len=%u", type, (unsigned)len);
      }

      consume(frame_len);
    }
  }

  size_t find_sof() const
  {
    for (size_t i = 0; i + 1 < read_buffer_pos_; i++) {
      if (read_buffer_[i] == EP_SOF1 && read_buffer_[i + 1] == EP_SOF2) return i;
    }
    return read_buffer_pos_;
  }

  void consume(size_t nbytes)
  {
    if (nbytes == 0) return;
    if (nbytes >= read_buffer_pos_) {
      read_buffer_pos_ = 0;
      return;
    }
    std::memmove(read_buffer_.data(), read_buffer_.data() + nbytes, read_buffer_pos_ - nbytes);
    read_buffer_pos_ -= nbytes;
  }

  void write_all(const uint8_t *data, size_t len)
  {
    if (!data || len == 0 || fd_ < 0) return;

    size_t sent = 0;
    for (int tries = 0; tries < 6 && sent < len; tries++) {
      ssize_t w = ::write(fd_, data + sent, len - sent);
      if (w > 0) {
        sent += static_cast<size_t>(w);
        continue;
      }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        ::usleep(1000);
        continue;
      }
      RCLCPP_WARN(get_logger(), "UART write failed: %s", strerror(errno));
      ::close(fd_);
      fd_ = -1;
      break;
    }
  }

private:
  // Params
  std::string serial_port_;
  int baud_{115200};
  double rate_hz_{50.0};
  std::string navsat_topic_;
  std::string odom_topic_;
  std::string cmdvel_topic_;

  // Serial
  int fd_{-1};
  uint8_t seq_{0};

  // Telemetry fields
  uint8_t mode_{EP_MODE_HOTAS};
  uint8_t arm_mode_{0};

  int32_t lat_e7_{0};
  int32_t lon_e7_{0};
  int16_t linear_x_mm_s_{0};
  int16_t angular_z_mrad_s_{0};
  uint16_t heading_cdeg_{0};

  bool have_fix_{false};
  bool have_heading_{false};
  bool have_cmd_{false};

  // Parsed RX commands
  bool have_waypoint_{false};
  bool set_base_requested_{false};

  // Serial read buffer
  std::array<uint8_t, READ_BUFFER_SIZE> read_buffer_;
  size_t read_buffer_pos_;

  // RNG (kept in case you add pitch jitter later)
  std::mt19937 rng_;

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_nav_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_nav_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rtb_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flush_wp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

class WaypointManagerInterpolater : public rclcpp::Node
{
public:
  using NavAction = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  WaypointManagerInterpolater()
  : Node("waypoint_manager"), interpolation_res_(3.0),
    current_goal_handle_(nullptr), mission_active_(false),
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

    rtb_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/rtb", 10, std::bind(&WaypointManagerInterpolater::rtbCb, this, _1));

    flush_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/flush_waypoints", 10, std::bind(&WaypointManagerInterpolater::flushCb, this, _1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsat/fix", 10, std::bind(&WaypointManagerInterpolater::gpsCb, this, _1));
        
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/interpolated_waypoints", 10);

    nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_through_poses");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Waypoint Manager Interpolator READY");
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

  std::vector<geometry_msgs::msg::PoseStamped> goal_points_;      // ALL goals from RViz (permanent)
  std::vector<geometry_msgs::msg::PoseStamped> raw_goals_;        // Active mission goals
  std::vector<geometry_msgs::msg::PoseStamped> interpolated_path_;
  double interpolation_res_;

  GoalHandle::SharedPtr current_goal_handle_;
  bool mission_active_ = false;
  bool goal_rtb_mode_ = false;
  size_t last_reached_goal_idx_ = 0;
  size_t completed_goals_count_ = 0;
  const double GOAL_REACHED_DIST = 0.5;

  int32_t latitude_{0}, longitude_{0};
  bool gps_printed_{false};

  // void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  // {   
  //   // Convert them to map coordinates using armed position as reference
  //   if (set_waypoints_) {
  //     double wp_lat = msg->pose.position.x;  // Latitude in degrees
  //     double wp_lon = msg->pose.position.y;  // Longitude in degrees
  //     // double ref_lat = armed_lat_e7_ / 1e7;  // Reference latitude (degrees)
  //     // double ref_lon = armed_lon_e7_ / 1e7;  // Reference longitude (degrees)
  //     double ref_lat = base_lat_e7_ / 1e7;  // Reference latitude (degrees)
  //     double ref_lon = base_lon_e7_ / 1e7;  // Reference longitude (degrees)
  //     double x, y;
  //     RCLCPP_INFO(get_logger(),"ref_lat=%f ref_lon=%f", ref_lat, ref_lon);
  //     gpsToMap(wp_lat, wp_lon, ref_lat, ref_lon, x, y);

  //     geometry_msgs::msg::PoseStamped waypoint_pose;
  //     waypoint_pose.header.frame_id = "map";
  //     waypoint_pose.header.stamp = now();
  //     waypoint_pose.pose.position.x = x;  // Map X coordinate (meters)
  //     waypoint_pose.pose.position.y = y;  // Map Y coordinate (meters)
  //     waypoint_pose.pose.position.z = 0.0;
  //     waypoint_pose.pose.orientation.w = 1.0;
  //     waypoint_pose.pose.orientation.x = 0.0;
  //     waypoint_pose.pose.orientation.y = 0.0;
  //     waypoint_pose.pose.orientation.z = 0.0;

  //     goal_points_.push_back(waypoint_pose);
  //     raw_goals_.push_back(waypoint_pose);

  //     RCLCPP_INFO(get_logger(), "Waypoint converted and added to goal_points_ and raw_goals_: GPS(%.6f, %.6f) -> Map(%.6f, %.6f) | Total goals: %zu",
  //                 wp_lat, wp_lon, x, y, goal_points_.size());
  //     publishMarkers();
  //   } else {
  //     if (!set_waypoints_) {
  //       RCLCPP_WARN(get_logger(), "Waypoint command not received yet, cannot add goal");
  //     }
  //     if (!is_arming_valid_) {
  //       RCLCPP_WARN(get_logger(), "ARM GPS reference not set, cannot convert waypoint to map coordinates");
  //     }
  //   }
  // }
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!set_waypoints_) {
      RCLCPP_WARN(get_logger(), "Waypoint command not received yet, cannot add goal");
      return;
    }

    const std::string frame = msg->header.frame_id;

    const double in_x = msg->pose.position.x;
    const double in_y = msg->pose.position.y;

    RCLCPP_INFO(get_logger(),
                "GOAL RX | frame='%s' | x=%.6f y=%.6f",
                frame.c_str(), in_x, in_y);

    // Heuristic: GPS degrees look like lat ~ [-90..90], lon ~ [-180..180]
    // Local meters are usually small-ish, and frame_id is map/odom/base_link.
    const bool looks_like_gps =
        (std::abs(in_x) <= 90.0) &&
        (std::abs(in_y) <= 180.0) &&
        (std::abs(in_x) > 1.0 || std::abs(in_y) > 1.0); // degrees usually not near 0 unless you're at equator/prime meridian

    const bool frame_says_local =
        (frame == "map" || frame == "odom" || frame == "base_link" || frame == "");

    double out_map_x = 0.0;
    double out_map_y = 0.0;

    // Case A: LOCAL goal already in meters
    if (frame_says_local && !looks_like_gps) {
      out_map_x = in_x;
      out_map_y = in_y;

      RCLCPP_INFO(get_logger(),
                  "GOAL INTERPRETATION: LOCAL meters -> using directly (%.3f, %.3f)",
                  out_map_x, out_map_y);
    }
    // Case B: GPS goal packed into x/y (lat/lon)
    else {
      // if (!is_arming_valid_) {
      //   RCLCPP_WARN(get_logger(), "Base/ARM GPS reference not set, cannot convert GPS -> map");
      //   return;
      // }

      const double wp_lat = in_x;  // latitude degrees
      const double wp_lon = in_y;  // longitude degrees

      const double ref_lat = base_lat_e7_ / 1e7;
      const double ref_lon = base_lon_e7_ / 1e7;

      RCLCPP_INFO(get_logger(),
                  "GOAL INTERPRETATION: GPS degrees -> converting | ref_lat=%.7f ref_lon=%.7f | wp_lat=%.7f wp_lon=%.7f",
                  ref_lat, ref_lon, wp_lat, wp_lon);

      gpsToMap(wp_lat, wp_lon, ref_lat, ref_lon, out_map_x, out_map_y);

      RCLCPP_INFO(get_logger(),
                  "GPS->MAP result: Map(%.3f, %.3f) meters",
                  out_map_x, out_map_y);
    }

    geometry_msgs::msg::PoseStamped waypoint_pose;
    waypoint_pose.header.frame_id = "map";
    waypoint_pose.header.stamp = now();

    waypoint_pose.pose.position.x = out_map_x;
    waypoint_pose.pose.position.y = out_map_y;
    waypoint_pose.pose.position.z = 0.0;

    waypoint_pose.pose.orientation.w = 1.0;
    waypoint_pose.pose.orientation.x = 0.0;
    waypoint_pose.pose.orientation.y = 0.0;
    waypoint_pose.pose.orientation.z = 0.0;

    goal_points_.push_back(waypoint_pose);
    raw_goals_.push_back(waypoint_pose);

    RCLCPP_INFO(get_logger(),
                "Waypoint added: Map(%.3f, %.3f) | Total goals: %zu",
                out_map_x, out_map_y, goal_points_.size());

    publishMarkers();
  }

  void rtbCb(const std_msgs::msg::Empty::SharedPtr)
  { 
    if (!mode_rtb_) {
      RCLCPP_WARN(get_logger(), "RTB mode is not set");
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "RTB mode is set");
    }

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
    if (base_set_) {
      RCLCPP_INFO(get_logger(),"base is set.");
      double ref_lat = armed_lat_e7_ / 1e7;
      double ref_lon = armed_lon_e7_ / 1e7;
      double base_lat = base_lat_e7_ / 1e7;
      double base_lon = base_lon_e7_ / 1e7;
      double x, y;
      RCLCPP_INFO(get_logger(),"base_position.x=%6f, base_position.y=%.6f | armed_position.x=%.6f armed_position.y=%.6f",
                  base_lat, base_lon, ref_lat, ref_lon);
      gpsToMap(base_lat, base_lon, ref_lat, ref_lon, x, y);
      base_pose.pose.position.x = x;
      base_pose.pose.position.y = y;
      RCLCPP_INFO(get_logger(),"base_position.x=%6f, base_position.y=%.6f",base_pose.pose.position.x, base_pose.pose.position.y);
    } else {
      base_pose.pose.position.x = 0.0;
      base_pose.pose.position.y = 0.0;
      RCLCPP_INFO(get_logger(),"base_position.x=%6f, base_position.y=%.6f",base_pose.pose.position.x, base_pose.pose.position.y);
    }
    base_pose.pose.position.z = 0.0;
    base_pose.pose.orientation.w = 1.0;

    double home_dist = std::hypot(base_pose.pose.position.x - robot_pose.pose.position.x,
                                  base_pose.pose.position.y - robot_pose.pose.position.y);
    if (isPoseCollisionFree(base_pose) && home_dist > MIN_RTB_GOAL_DIST) {
        rtb_path.push_back(base_pose);
        RCLCPP_INFO(get_logger(), "🏠 RTB Datum added: (%.1f, %.1f) dist=%.1fm", 
                    base_pose.pose.position.x, base_pose.pose.position.y, home_dist);
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

  void flushCb(const std_msgs::msg::Empty::SharedPtr)
  {
    goal_points_.clear();
    raw_goals_.clear();
    RCLCPP_INFO(get_logger(), "Waypoints flushed");
    publishMarkers();
  }

  void startCb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!is_arming_valid_ || !mode_pursuit_) {
      RCLCPP_WARN(get_logger(), "Cannot start navigation: invalid arming or not in PURSUIT mode");
      return;
    }

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
    goal_rtb_mode_ = false;
    interpolated_path_.clear();
    raw_goals_.clear();
    goal_points_.clear();
    RCLCPP_INFO(get_logger(), "🔴 Naivgation stopped.");
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
    // if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
    //   RCLCPP_ERROR(get_logger(), "Nav2 unavailable");
    //   return;
    // }

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
      goal_rtb_mode_ = false;
      interpolated_path_.clear();
      publishMarkers();
    };

    nav_client_->async_send_goal(goal, options);
  }

  void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // latitude_ = msg->latitude;
    // longitude_ = msg->longitude;
    latitude_ = gpscb_lat_e7;
    longitude_ = gpscb_lon_e7;
    if (!gps_printed_) {
      RCLCPP_INFO(get_logger(), "GPS: lat=%d lon=%d", latitude_, longitude_);
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
      p.pose.orientation.w = std::cos(yaw / 2.0);
      p.pose.orientation.z = std::sin(yaw / 2.0);
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

  // converting raw lat long data to ENU format 
  // void gpsToMap(double lat, double lon,
  //               double ref_lat, double ref_lon,
  //               double &x, double &y)
  // {
  //   static constexpr double DEG2RAD = M_PI / 180.0;
  //   static constexpr double EARTH_RADIUS_M = 6378137.0;

  //   double dlat = (lat - ref_lat) * DEG2RAD;
  //   double dlon = (lon - ref_lon) * DEG2RAD;

  //   double lat_rad = ref_lat * DEG2RAD;

  //   x = dlon * std::cos(lat_rad) * EARTH_RADIUS_M; // East
  //   y = dlat * EARTH_RADIUS_M;                     // North
  //   RCLCPP_INFO(get_logger(), "x in map=%f y in map=%f", x, y);
  // }
  void gpsToMap(double lat, double lon,
              double ref_lat, double ref_lon,
              double &x, double &y)
  {
    static constexpr double DEG2RAD = M_PI / 180.0;
    static constexpr double EARTH_RADIUS_M = 6378137.0;

    // Guardrails so dumb inputs don't nuke your world
    if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0 ||
        std::abs(ref_lat) > 90.0 || std::abs(ref_lon) > 180.0) {
      x = 0.0;
      y = 0.0;
      return;
    }

    const double dlat = (lat - ref_lat) * DEG2RAD;
    const double dlon = (lon - ref_lon) * DEG2RAD;

    const double lat_rad = ref_lat * DEG2RAD;

    // East (x), North (y)
    x = dlon * std::cos(lat_rad) * EARTH_RADIUS_M;
    y = dlat * EARTH_RADIUS_M;
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose)
  {
    // Ensure ARM reference (map origin) is available
    if (!is_arming_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "getRobotPose(): map origin not set (ARM GPS missing)");
      return false;
    }

    // Use the lat/lng from when ARM command was received, converted to map frame
    // double current_lat = armed_lat_e7_ / 1e7;
    // double current_lon = armed_lon_e7_ / 1e7;
    // double ref_lat = armed_lat_e7_ / 1e7;
    // double ref_lon = armed_lon_e7_ / 1e7;

    // double x, y;
    // gpsToMap(current_lat, current_lon, ref_lat, ref_lon, x, y);

    const std::string frame = "map";

    const double in_x = pose.pose.position.x;
    const double in_y = pose.pose.position.y;

    // RCLCPP_INFO(get_logger(),
    //             "GOAL RX | frame='%s' | x=%.6f y=%.6f",
    //             frame.c_str(), in_x, in_y);

    // Heuristic: GPS degrees look like lat ~ [-90..90], lon ~ [-180..180]
    // Local meters are usually small-ish, and frame_id is map/odom/base_link.
    const bool looks_like_gps =
        (std::abs(in_x) <= 90.0) &&
        (std::abs(in_y) <= 180.0) &&
        (std::abs(in_x) > 1.0 || std::abs(in_y) > 1.0); // degrees usually not near 0 unless you're at equator/prime meridian

    const bool frame_says_local =
        (frame == "map" || frame == "odom" || frame == "base_link" || frame == "");

    double out_map_x = 0.0;
    double out_map_y = 0.0;

    // Case A: LOCAL goal already in meters
    if (frame_says_local && !looks_like_gps) {
      out_map_x = in_x;
      out_map_y = in_y;

      RCLCPP_INFO(get_logger(),
                  "GOAL INTERPRETATION: LOCAL meters -> using directly (%.3f, %.3f)",
                  out_map_x, out_map_y);
    }
    // Case B: GPS goal packed into x/y (lat/lon)
    else {
      // if (!is_arming_valid_) {
      //   RCLCPP_WARN(get_logger(), "Base/ARM GPS reference not set, cannot convert GPS -> map");
      //   return;
      // }

      const double robot_current_lat = in_x;  // latitude degrees
      const double robot_current_lon = in_y;  // longitude degrees

      const double ref_lat = base_lat_e7_ / 1e7;
      const double ref_lon = base_lon_e7_ / 1e7;

      RCLCPP_INFO(get_logger(),
                  "GOAL INTERPRETATION: GPS degrees -> converting | ref_lat=%.7f ref_lon=%.7f | robot_current_lat=%.7f robot_current_lon=%.7f",
                  ref_lat, ref_lon, robot_current_lat, robot_current_lon);

      gpsToMap(robot_current_lat, robot_current_lon, ref_lat, ref_lon, out_map_x, out_map_y);

      RCLCPP_INFO(get_logger(),
                  "GPS->MAP result: Map(%.3f, %.3f) meters",
                  out_map_x, out_map_y);
    }

    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = out_map_x;
    pose.pose.position.y = out_map_y;
    pose.pose.position.z = 0.0;
    RCLCPP_INFO(get_logger(), "current_position.x=%f, current_position.y=%f", out_map_x,out_map_y);

    // Orientation: neutral (or replace later with IMU heading)
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    return true;
  }


  // bool getRobotPose(geometry_msgs::msg::PoseStamped &pose)
  // {
  //   try {
  //     auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  //     pose.header.frame_id = "map";
  //     pose.header.stamp = now();
  //     pose.pose.position.x = tf.transform.translation.x;
  //     pose.pose.position.y = tf.transform.translation.y;
  //     pose.pose.position.z = 0.0;
  //     pose.pose.orientation = tf.transform.rotation;
  //     return true;
  //   } catch (...) {
  //     return false;
  //   }
  // }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr flush_sub_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto telemetry_node =
      std::make_shared<AgxEpTelemetrySender>();

  auto waypoint_node =
      std::make_shared<WaypointManagerInterpolater>();
  // Multi-threaded executor is IMPORTANT
  // - UART + timers
  // - Nav2 action client
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(telemetry_node);
  executor.add_node(waypoint_node);

  RCLCPP_INFO(
      telemetry_node->get_logger(),
      "✅ Telemetry + Waypoint Manager running in ONE process");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
