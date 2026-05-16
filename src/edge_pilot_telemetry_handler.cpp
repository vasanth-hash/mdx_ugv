/*
EXECUTION:
ef-software@ubuntu:~/ros2_ws$ ros2 run mdx_ugv edge_pilot_telemetry_handler --ros-args   -p serial_port:=/dev/ttyUSB0   -p baud:=115200   -p rate_hz:=100.0   -p navsat_topic:=/navsat/fix   -p odom_topic:=/odom   -p cmdvel_topic:=/cmd_vel
[INFO] [1769598656.519402438] [agx_ep_telemetry_sender]: AGX EP telemetry up. UART=/dev/ttyUSB0 @115200, tx_rate=100.0Hz. RX parsers: mode/set_base/waypoint/arm
[INFO] [1769598699.861298534] [agx_ep_telemetry_sender]: 🎮 FRAMED MODE SHADOW (0x31)
[INFO] [1769598702.871336626] [agx_ep_telemetry_sender]: 🎮 FRAMED MODE BASE (0x33)
[INFO] [1769598705.001493904] [agx_ep_telemetry_sender]: 🎮 FRAMED MODE PURSUIT (0x32)
[INFO] [1769598706.871590617] [agx_ep_telemetry_sender]: 🎮 FRAMED MODE HOTAS (0x30)
[INFO] [1769598714.921975446] [agx_ep_telemetry_sender]: 📍 FRAMED WAYPOINT SET: lat=17.465754 lon=78.366488 | armed=0 mode=0 base_set=NO
[INFO] [1769598721.082266950] [agx_ep_telemetry_sender]: Flush Waypoints received, all waypoints should be cleared
[INFO] [1769598731.583018667] [agx_ep_telemetry_sender]: Armed
[INFO] [1769598732.893099041] [agx_ep_telemetry_sender]: DisArmed
[INFO] [1769598751.784183030] [agx_ep_telemetry_sender]: 🏠 FRAMED BASE SET | armed=0 mode=0 waypoint_set=YES

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

using namespace std::chrono_literals;

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
      "AGX EP telemetry up. UART=%s @%d, tx_rate=%.1fHz. RX parsers: mode/set_base/waypoint/arm",
      serial_port_.c_str(), baud_, rate_hz_);
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
    have_fix_ = true;
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

      // handle commands from edge_pilot.c
      // if (type == EP_MSG_ARM_EVT && len == 1) {
      //   arm_mode_ = (payload[0] != 0U) ? 1U : 0U;
      //   RCLCPP_INFO(get_logger(),
      //     "🟢 FRAMED ARM_EVT: armed=%u | base_set=%s waypoint_set=%s mode=%u",
      //     arm_mode_,
      //     set_base_requested_ ? "YES" : "NO",
      //     have_waypoint_ ? "YES" : "NO",
      //     mode_);
      //   //RCLCPP_INFO(get_logger(), "🟢 FRAMED ARM_EVT 0x20: armed=%u", (unsigned)arm_mode_);
      // } 
      if (type == 0x0A && len == 0) {
        arm_mode_ = 1;
        RCLCPP_INFO(get_logger(),"Armed");
      } 
      else if (type == 0X0B && len == 0) {
        arm_mode_ = 0;
        RCLCPP_INFO(get_logger(), "DisArmed");
      }
      else if (type == EP_MSG_MODE_HOTAS && len == 0) {
        mode_ = EP_MODE_HOTAS;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE HOTAS (0x30)");
      }
      else if (type == EP_MSG_MODE_SHADOW && len == 0) {
        mode_ = EP_MODE_SHADOW;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE SHADOW (0x31)");
      }
      else if (type == EP_MSG_MODE_PURSUIT && len == 0) {
        mode_ = EP_MODE_PURSUIT;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE PURSUIT (0x32)");
      }
      else if (type == EP_MSG_MODE_BASE && len == 0) {
        mode_ = EP_MODE_BASE;
        RCLCPP_INFO(get_logger(), "🎮 FRAMED MODE BASE (0x33)");
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
        int32_t lat_e6 = rd_i32_le(payload);
        int32_t lon_e6 = rd_i32_le(payload + 4);
        have_waypoint_ = true;
        wp_lat_e6_ = lat_e6;
        wp_lon_e6_ = lon_e6;
        RCLCPP_INFO(get_logger(),
          "📍 FRAMED WAYPOINT SET: lat=%.6f lon=%.6f | armed=%u mode=%u base_set=%s",
          lat_e6 / 1e6, lon_e6 / 1e6,
          arm_mode_, mode_,
          set_base_requested_ ? "YES" : "NO");
        // RCLCPP_INFO(get_logger(), "📍 FRAMED WAYPOINT 0x23: lat=%.6f lon=%.6f",
        //             lat_e6 / 1e6, lon_e6 / 1e6);
      } else if (type == EP_MSG_SET_BASE && len == 0) {
        set_base_requested_ = true;
        RCLCPP_INFO(get_logger(),
          "🏠 FRAMED BASE SET | armed=%u mode=%u waypoint_set=%s",
          arm_mode_, mode_,
          have_waypoint_ ? "YES" : "NO");
        //RCLCPP_INFO(get_logger(), "🏠 FRAMED SET_BASE 0x24");
      } else if (type == EP_MSG_GOAL && len == 8) {
        // optional if you ever use it
        int32_t lat_e7 = rd_i32_le(payload);
        int32_t lon_e7 = rd_i32_le(payload + 4);
        RCLCPP_INFO(get_logger(), "🎯 FRAMED GOAL 0x21: lat=%.7f lon=%.7f",
                    lat_e7 / 1e7, lon_e7 / 1e7);
      }else if (type == 0x69 && len == 0) {
        RCLCPP_INFO(get_logger(), "Flush Waypoints received, all waypoints should be cleared");
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
  int32_t wp_lat_e6_{0};
  int32_t wp_lon_e6_{0};
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
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgxEpTelemetrySender>());
  rclcpp::shutdown();
  return 0;
}