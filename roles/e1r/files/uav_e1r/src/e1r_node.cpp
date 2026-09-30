#include "uav_e1r/time_gate.hpp"
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/driver/decoder/decoder_RSE1.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using Cloud = PointCloudT<PointXYZIRT>;
using Decoder = robosense::lidar::DecoderRSE1<Cloud>;
using namespace std::chrono_literals;

namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string read_line(const char *path) {
  std::ifstream f(path); std::string s; std::getline(f, s); return s;
}
int open_udp(int port) {
  if (port < 1024 || port > 65535) throw std::runtime_error("Invalid unprivileged UDP port");
  int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) throw std::runtime_error("Cannot open UDP socket");
  int bytes = 4 * 1024 * 1024;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
  // Exclusive ports prevent a second driver from silently sharing/capturing data.
  sockaddr_in addr{}; addr.sin_family = AF_INET;
  addr.sin_port = htons(uint16_t(port)); addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr))) {
    close(fd); throw std::runtime_error("Cannot bind UDP port " + std::to_string(port));
  }
  return fd;
}
template<class T> void store(std::vector<uint8_t> &data, size_t offset, T value) {
  std::memcpy(data.data() + offset, &value, sizeof(value));
}
}  // namespace

class E1RNode final : public rclcpp::Node {
 public:
  E1RNode() : Node("e1r_driver") {
    frame_ = declare_parameter<std::string>("frame_id", "e1r_nominal_lidar_frame");
    lidar_ip_ = declare_parameter<std::string>("lidar_ip", "192.168.144.95");
    status_path_ = declare_parameter<std::string>("clock_status_path", "/run/uav/time/ptp-status.json");
    const int msop_port = declare_parameter<int>("msop_port", 9007);
    const int difop_port = declare_parameter<int>("difop_port", 9107);
    gate_.difop_max_age_ns = int64_t(declare_parameter<double>("difop_timeout_sec", 1.0) * 1e9);
    gate_.timestamp_tolerance_ns = int64_t(declare_parameter<double>("timestamp_tolerance_sec", 2.0) * 1e9);
    if (inet_pton(AF_INET, lidar_ip_.c_str(), &lidar_addr_) != 1)
      throw std::runtime_error("Invalid E1R IPv4 address");
    boot_id_ = read_line("/proc/sys/kernel/random/boot_id");
    if (boot_id_.empty()) throw std::runtime_error("Cannot read host boot ID");
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/e1r/points", rclcpp::SensorDataQoS());
    raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/e1r/difop_raw", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    msop_fd_ = open_udp(msop_port);
    try { difop_fd_ = open_udp(difop_port); }
    catch (...) { close(msop_fd_); throw; }
    receive_timer_ = create_wall_timer(1ms, [this] { receive(); });
    clock_timer_ = create_wall_timer(200ms, [this] { refresh_clock(); });
    diagnostics_timer_ = create_wall_timer(1s, [this] { diagnostics(); });
    refresh_clock();
    RCLCPP_INFO(get_logger(), "Pinned RoboSense decoder; UTC-gated E1R %s UDP %d/%d, frame %s",
      lidar_ip_.c_str(), msop_port, difop_port, frame_.c_str());
  }

  ~E1RNode() override { if(msop_fd_ >= 0) close(msop_fd_); if(difop_fd_ >= 0) close(difop_fd_); }

 private:
  void invalidate(const std::string &reason) {
    last_reason_ = reason;
    decoder_.reset();
    full_scan_started_ = false;
  }

  void refresh_clock() {
    const int previous_offset = gate_.clock.utc_offset;
    try {
      std::ifstream f(status_path_);
      const auto j = nlohmann::json::parse(f);
      gate_.clock.valid = j.at("utc_offset_verified").get<bool>() &&
        j.at("ptp_timescale").get<bool>() && j.at("boot_id").get<std::string>() == boot_id_;
      gate_.clock.utc_offset = j.at("current_utc_offset").get<int>();
      gate_.clock.updated_steady_ns = j.at("updated_monotonic_ns").get<int64_t>();
      utc_offset_advertised_valid_ = j.at("utc_offset_valid").get<bool>();
      if (gate_.clock.utc_offset != previous_offset) invalidate("PTP UTC offset changed; reacquiring complete scan");
      auto reason = gate_.check_clock(steady_ns());
      if (!reason.empty()) invalidate(reason);
    } catch (const std::exception &) {
      gate_.clock.valid = false;
      invalidate("PTP clock status missing or malformed");
    }
  }

  void initialize_decoder() {
    robosense::lidar::RSDecoderParam params;
    params.use_lidar_clock = true;
    params.ts_first_point = true;
    params.dense_points = false;
    params.wait_for_difop = true;
    decoder_ = std::make_unique<Decoder>(params);
    decoder_->point_cloud_ = std::make_shared<Cloud>();
    decoder_->regCallback(
      [this](const robosense::lidar::Error &e) { ++decoder_errors_; last_reason_ = e.toString(); },
      [this](uint16_t, double stamp) { publish_cloud(stamp); });
    full_scan_started_ = false;
  }

  void receive() {
    if (decoder_ && last_msop_steady_ >= 0 && steady_ns() - last_msop_steady_ > 1000000000LL)
      invalidate("MSOP stream stale; reacquiring complete scan");
    // Process status first, so the current lock result gates subsequent points.
    drain(difop_fd_, true);
    drain(msop_fd_, false);
  }

  void drain(int fd, bool difop) {
    std::array<uint8_t, 1500> data{};
    for (int i = 0; i < 512; ++i) {
      sockaddr_in sender{}; socklen_t slen = sizeof(sender);
      const ssize_t n = recvfrom(fd, data.data(), data.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &slen);
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ++socket_errors_;
        break;
      }
      if (sender.sin_addr.s_addr != lidar_addr_.s_addr) { ++foreign_packets_; continue; }
      const auto steady = steady_ns();
      const auto wall = wall_ns();
      last_packet_steady_ = steady;  // Connection evidence survives malformed/timing-rejected data.
      uav_e1r::PacketTime packet;
      if (!uav_e1r::packet_time(data.data(), size_t(n), difop, packet)) {
        ++malformed_packets_; invalidate("Malformed E1R packet"); continue;
      }
      if (difop) {
        ++difop_packets_;
        gate_.observe_difop(packet, steady);
        std_msgs::msg::UInt8MultiArray raw;
        raw.data.assign(data.begin(), data.begin() + n);
        raw_pub_->publish(std::move(raw));
        auto reason = gate_.check_difop(wall, steady);
        if (!reason.empty()) invalidate(reason);
        // No IMU publication until the vendor units and IMU frame are verified.
      } else {
        ++msop_packets_;
        last_msop_ = packet;
        last_msop_steady_ = steady;
        auto reason = gate_.check_msop(packet, wall, steady);
        if (!reason.empty()) { ++rejected_packets_; invalidate(reason); continue; }
        if (last_accepted_raw_ns_ && packet.raw_ns < last_accepted_raw_ns_) {
          ++backward_packets_; invalidate("MSOP clock moved backwards");
        }
        if (last_accepted_raw_ns_ && packet.raw_ns - last_accepted_raw_ns_ > 1000000000LL)
          invalidate("MSOP stream gap; reacquiring complete scan");
        if (have_sequence_ && packet.sequence > last_sequence_ + 1) {
          missing_packets_ += packet.sequence - last_sequence_ - 1;
        }
        last_sequence_ = packet.sequence;
        have_sequence_ = true;
        last_accepted_raw_ns_ = packet.raw_ns;
        if (!decoder_) initialize_decoder();
        decoder_->processMsopPkt(data.data(), size_t(n));
      }
    }
  }

  void publish_cloud(double raw_stamp) {
    // Called synchronously at the vendor's frame boundary, before the first
    // packet of the next scan is decoded. Drop startup/recovery partial scans.
    auto cloud = decoder_->point_cloud_;
    decoder_->point_cloud_ = std::make_shared<Cloud>();
    if (!full_scan_started_) { full_scan_started_ = true; ++partial_scans_; return; }
    if (cloud->points.empty()) return;
    if (!gate_.check_msop(last_msop_, wall_ns(), steady_ns()).empty()) {
      full_scan_started_ = false; ++rejected_scans_; return;
    }
    const double utc_stamp = gate_.utc_seconds(raw_stamp);
    sensor_msgs::msg::PointCloud2 out;
    out.header.frame_id = frame_;
    out.header.stamp = rclcpp::Time(int64_t(std::llround(utc_stamp * 1e9)), RCL_SYSTEM_TIME);
    out.height = 1; out.width = uint32_t(cloud->points.size());
    out.is_dense = false; out.is_bigendian = false; out.point_step = 24;
    out.row_step = out.width * out.point_step;
    using F = sensor_msgs::msg::PointField;
    auto field = [&out](const std::string &name, uint32_t offset, uint8_t type) {
      F f; f.name = name; f.offset = offset; f.datatype = type; f.count = 1; out.fields.push_back(f);
    };
    field("x", 0, F::FLOAT32); field("y", 4, F::FLOAT32); field("z", 8, F::FLOAT32);
    field("intensity", 12, F::UINT8); field("ring", 14, F::UINT16); field("timestamp", 16, F::FLOAT64);
    out.data.resize(out.row_step);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto &p = cloud->points[i]; const size_t off = i * out.point_step;
      store(out.data, off, p.x); store(out.data, off + 4, p.y); store(out.data, off + 8, p.z);
      store(out.data, off + 12, p.intensity); store(out.data, off + 14, p.ring);
      // Every point retains the official decoder's microsecond firing offset.
      store(out.data, off + 16, gate_.utc_seconds(p.timestamp));
    }
    last_cloud_raw_stamp_ = raw_stamp;
    last_cloud_utc_stamp_ = utc_stamp;
    last_cloud_points_ = out.width;
    points_pub_->publish(std::move(out));
    ++published_clouds_; last_cloud_steady_ = steady_ns(); last_reason_.clear();
  }

  void diagnostics() {
    const auto steady = steady_ns();
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "e1r/driver"; d.hardware_id = "E1R@" + lidar_ip_;
    auto reason = gate_.check_msop(last_msop_, wall_ns(), steady);
    // Packet reception proves connection even when strict timestamp gates
    // withhold clouds. Only losing both packet streams is a connection error.
    const auto recent = [steady](int64_t received) {
      return received >= 0 && steady - received <= 3000000000LL;
    };
    const bool connected = recent(last_packet_steady_);
    if (!connected) { d.level = d.ERROR; d.message = "No E1R MSOP or DIFOP packets"; }
    else if (!reason.empty()) { d.level = d.WARN; d.message = reason; }
    else if (last_cloud_steady_ < 0 || steady - last_cloud_steady_ > 1000000000LL) {
      d.level = d.WARN; d.message = "gPTP valid; waiting for complete point cloud";
    } else { d.level = d.OK; d.message = "Streaming; gPTP timestamps normalized to UTC"; }
    auto value = [&d](const std::string &key, const auto &v) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = key;
      std::ostringstream s; s << std::setprecision(17) << v; kv.value = s.str(); d.values.push_back(kv);
    };
    value("driver_commit", "897b14d3bdb6186a75df27ba51b65b5bd5557723");
    value("connected", connected);
    value("packet_age_sec", last_packet_steady_ < 0 ? -1.0 : (steady - last_packet_steady_) * 1e-9);
    value("msop_packets", msop_packets_); value("difop_packets", difop_packets_);
    value("published_clouds", published_clouds_); value("last_cloud_points", last_cloud_points_);
    value("rejected_packets", rejected_packets_); value("rejected_scans", rejected_scans_);
    value("partial_scans_discarded", partial_scans_); value("backward_packets", backward_packets_);
    value("malformed_packets", malformed_packets_); value("foreign_packets", foreign_packets_);
    value("missing_packets_within_scan", missing_packets_);
    value("socket_errors", socket_errors_); value("decoder_errors", decoder_errors_);
    value("difop_time_mode", int(gate_.difop.mode)); value("difop_sync_status", int(gate_.difop.status));
    value("msop_time_mode", int(last_msop_.mode)); value("last_msop_raw_ptp_ns", last_msop_.raw_ns);
    value("last_difop_raw_ptp_ns", gate_.difop.raw_ns);
    value("current_utc_offset", gate_.clock.utc_offset);
    value("utc_offset_verified", gate_.clock.valid); value("utc_offset_advertised_valid", utc_offset_advertised_valid_);
    value("last_cloud_raw_ptp_sec", last_cloud_raw_stamp_); value("last_cloud_utc_sec", last_cloud_utc_stamp_);
    value("msop_age_sec", last_msop_steady_ < 0 ? -1.0 : (steady - last_msop_steady_) * 1e-9);
    value("difop_age_sec", gate_.difop_received_steady_ns < 0 ? -1.0 : (steady - gate_.difop_received_steady_ns) * 1e-9);
    value("last_rejection_reason", last_reason_);
    value("imu", "not published: units and frame unverified");
    array.status.push_back(std::move(d)); diagnostics_pub_->publish(std::move(array));
  }

  uav_e1r::TimeGate gate_;
  uav_e1r::PacketTime last_msop_;
  std::unique_ptr<Decoder> decoder_;
  bool full_scan_started_ = false, utc_offset_advertised_valid_ = false, have_sequence_ = false;
  uint16_t last_sequence_ = 0;
  std::string frame_, lidar_ip_, status_path_, boot_id_, last_reason_;
  in_addr lidar_addr_{};
  int msop_fd_ = -1, difop_fd_ = -1;
  int64_t last_packet_steady_ = -1, last_msop_steady_ = -1, last_cloud_steady_ = -1, last_accepted_raw_ns_ = 0;
  double last_cloud_raw_stamp_ = 0, last_cloud_utc_stamp_ = 0;
  uint64_t msop_packets_ = 0, difop_packets_ = 0, published_clouds_ = 0, last_cloud_points_ = 0;
  uint64_t rejected_packets_ = 0, rejected_scans_ = 0, partial_scans_ = 0, backward_packets_ = 0;
  uint64_t malformed_packets_ = 0, foreign_packets_ = 0, socket_errors_ = 0, decoder_errors_ = 0;
  uint64_t missing_packets_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr receive_timer_, clock_timer_, diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<E1RNode>()); }
  catch (const std::exception &e) { std::cerr << "E1R driver failed: " << e.what() << '\n'; return 1; }
  rclcpp::shutdown();
  return 0;
}
