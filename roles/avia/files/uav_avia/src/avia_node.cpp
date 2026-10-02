#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/timex.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_sdk.h>
#include "uav_avia/point.hpp"
#include "uav_avia/time.hpp"
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This Livox wire decoder requires a little-endian host.
#endif
using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;
using Cloud = sensor_msgs::msg::PointCloud2;
using Imu = sensor_msgs::msg::Imu;
using Custom = livox_ros_driver2::msg::CustomMsg;
using Diag = diagnostic_msgs::msg::DiagnosticStatus;
using uav_avia::ns_per_s;

namespace {
int64_t realtime_ns() { timespec t{};clock_gettime(CLOCK_REALTIME,&t);return static_cast<int64_t>(t.tv_sec)*ns_per_s+t.tv_nsec; }
int64_t second_fraction(int64_t ns) { return ((ns%ns_per_s)+ns_per_s)%ns_per_s; }
void sleep_monotonic_ns(int64_t ns) {
  if (ns<=0) return;
  timespec t{static_cast<time_t>(ns/ns_per_s),static_cast<long>(ns%ns_per_s)};
  while (clock_nanosleep(CLOCK_MONOTONIC,0,&t,&t)==EINTR) {}
}
builtin_interfaces::msg::Time to_msg(int64_t ns) {
  builtin_interfaces::msg::Time t;t.sec=static_cast<int32_t>(ns/ns_per_s);t.nanosec=static_cast<uint32_t>(ns%ns_per_s);return t;
}
const char* const skip_reasons[]={"none","Avia not streaming","host clock not synchronized","UTC out of range","send window missed"};
}  // namespace

class AviaNode : public rclcpp::Node {
 public:
  AviaNode() : Node("avia_driver") {
    code_=declare_parameter<std::string>("broadcast_code","3JEDNAP001S5701");
    ip_=declare_parameter<std::string>("lidar_ip","192.168.144.80");
    frame_=declare_parameter<std::string>("frame_id","avia_nominal_lidar_frame");
    imu_frame_=declare_parameter<std::string>("imu_frame_id","avia_imu");
    rate_=declare_parameter<double>("publish_rate_hz",10.0);
    timeout_=declare_parameter<double>("receipt_timeout_sec",2.0);
    utc_sync_=declare_parameter<bool>("utc_sync",true);
    utc_phase_=declare_parameter<double>("utc_sync_phase_sec",0.1);
    max_clock_error_=declare_parameter<double>("utc_sync_max_clock_error_sec",0.05);
    header_policy_=declare_parameter<std::string>("header_stamp","sensor_utc");
    const double lat_min=declare_parameter<double>("sensor_latency_min_sec",-0.001);
    const double lat_max=declare_parameter<double>("sensor_latency_max_sec",0.05);
    // PTP time minus UTC. 0 behind ptp4l on the system clock (it serves UTC,
    // announced as an arbitrary timescale); 37 behind a TAI (PTP-timescale) master.
    ptp_utc_offset_ns_=declare_parameter<int64_t>("ptp_utc_offset_sec",0)*ns_per_s;
    if (rate_<1 || rate_>50 || timeout_<=0 || code_.empty() || ip_.empty())
      throw std::invalid_argument("Invalid Avia configuration");
    // The Avia wants the UTC of a pulse 10-500 ms after that pulse's rising edge.
    // The host clock can be off by up to max_clock_error, so the send phase keeps
    // that margin on both sides: a label can then never land on the wrong pulse.
    lo_ns_=static_cast<int64_t>(std::llround((0.01+max_clock_error_)*1e9));
    hi_ns_=static_cast<int64_t>(std::llround((0.49-max_clock_error_)*1e9));
    phase_ns_=static_cast<int64_t>(std::llround(utc_phase_*1e9));
    lat_min_ns_=static_cast<int64_t>(std::llround(lat_min*1e9));
    lat_max_ns_=static_cast<int64_t>(std::llround(lat_max*1e9));
    if (max_clock_error_<=0 || phase_ns_<lo_ns_ || phase_ns_>hi_ns_ || lat_max_ns_<=lat_min_ns_ ||
        (header_policy_!="sensor_utc" && header_policy_!="host_receipt"))
      throw std::invalid_argument("Invalid Avia time synchronization configuration");
    cloud_pub_=create_publisher<Cloud>("/avia/points",rclcpp::SensorDataQoS().keep_last(2));
    // For LiDAR-inertial odometry (FAST-LIO): the Avia's own IMU and Livox's
    // per-point-time frame, both on the sensor clock. Reliable, as FAST-LIO
    // subscribes reliably (a best-effort publisher would never match it).
    imu_pub_=create_publisher<Imu>("/avia/imu",rclcpp::QoS(50));
    custom_pub_=create_publisher<Custom>("/avia/custom",rclcpp::QoS(5));
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    fields_=fields();buffer_.reserve(30000*uav_avia::point_step);
    instance_=this;
    if (!Init()) throw std::runtime_error("Livox SDK initialization failed (port already in use?)");
    initialized_=true;
    SetBroadcastCallback(broadcast);SetDeviceStateUpdateCallback(state);
    if (!Start()) { Uninit();initialized_=false;throw std::runtime_error("Livox discovery start failed"); }
    timer_=create_wall_timer(std::chrono::duration<double>(1.0/rate_),[this]{publish_cloud();});
    diag_timer_=create_wall_timer(1s,[this]{diagnostics();});
    RCLCPP_INFO(get_logger(),"Waiting for Avia %s at %s; UTC push %s (%.0f ms after each second, host clock error gate %.0f ms); headers: %s",
                code_.c_str(),ip_.c_str(),utc_sync_?"on":"off",utc_phase_*1e3,max_clock_error_*1e3,
                header_policy_=="sensor_utc"?"sensor UTC once validated, host receipt until then":"host receipt");
    if (utc_sync_) utc_thread_=std::thread([this]{utc_loop();});  // last: nothing may throw after this
  }
  ~AviaNode() override {
    stopping_=true;
    if (utc_thread_.joinable()) utc_thread_.join();
    if (initialized_) {
      if (connected_) { LidarStopSampling(handle_,nullptr,nullptr);std::this_thread::sleep_for(100ms); }
      Uninit();
    }
    instance_=nullptr;
  }
 private:
  inline static AviaNode* instance_=nullptr;
  bool initialized_=false;
  std::atomic<bool> stopping_{false},connected_{false},configuring_{false},sampling_{false};
  std::atomic<uint8_t> handle_{0};
  std::atomic<int> stage_{0},config_errors_{0};
  std::string code_,ip_,frame_,imu_frame_,header_policy_,firmware_="unknown";
  double rate_,timeout_,utc_phase_=0.1,max_clock_error_=0.05;
  bool utc_sync_=true;
  int64_t lo_ns_=0,hi_ns_=0,phase_ns_=0,lat_min_ns_=0,lat_max_ns_=0,ptp_utc_offset_ns_=0;
  std::thread utc_thread_;
  std::atomic<uint64_t> utc_sent_{0},utc_acked_{0},utc_rejected_{0},utc_errors_{0},utc_skipped_{0};
  std::atomic<long> clock_maxerror_us_{-1};
  std::atomic<int> utc_skip_reason_{0};
  std::atomic<bool> logged_ack_{false},logged_reject_{false};
  std::mutex mutex_;
  std::vector<uint8_t> buffer_;
  std::vector<sensor_msgs::msg::PointField> fields_;
  builtin_interfaces::msg::Time first_receipt_;
  int64_t first_sensor_ns_=0;
  bool first_trusted_=false;
  Steady::time_point first_steady_{},last_point_{},last_imu_{},last_diag_=Steady::now();
  uint64_t packets_=0,imu_packets_=0,slots_=0,valid_=0,clouds_=0,bad_=0,overflows_=0;
  uint64_t gap_events_=0,regressions_=0,previous_time_=0,raw_time_=0;
  uint64_t previous_packets_=0,previous_clouds_=0,previous_imu_=0;
  uint64_t imu_sensor_=0,imu_receipt_=0;           // under mutex_
  uint64_t customs_=0,custom_points_dropped_=0;    // executor thread only
  // Sensor-time evidence for the current diagnostic interval (under mutex_).
  int64_t lat_lo_=0,lat_hi_=0;
  double lat_sum_=0;
  uint64_t lat_n_=0,untrusted_utc_=0,bad_utc_=0;
  uint64_t clouds_sensor_=0,clouds_receipt_=0,prev_clouds_sensor_=0,prev_clouds_receipt_=0;
  bool last_validated_=false;
  int last_pps_=-1,last_ptp_=-1;
  uint32_t last_sync_=99;
  uint32_t status_=0;
  uint8_t timestamp_type_=255,previous_type_=255;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<Custom>::SharedPtr custom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_,diag_timer_;

  static std::vector<sensor_msgs::msg::PointField> fields() {
    using F=sensor_msgs::msg::PointField;std::vector<F> out;
    auto add=[&](const char* name,uint32_t off,uint8_t type){F f;f.name=name;f.offset=off;f.datatype=type;f.count=1;out.push_back(f);};
    add("x",0,F::FLOAT32);add("y",4,F::FLOAT32);add("z",8,F::FLOAT32);add("intensity",12,F::FLOAT32);
    add("tag",16,F::UINT8);add("line",17,F::UINT8);add("timestamp_type",18,F::UINT8);
    add("sensor_time_low",20,F::UINT32);add("sensor_time_high",24,F::UINT32);
    add("point_offset_ns",28,F::UINT32);add("status_code",32,F::UINT32);return out;
  }
  static void broadcast(const BroadcastDeviceInfo* info) {
    auto* n=instance_;if (!n || n->stopping_ || !info) return;
    if (info->dev_type!=kDeviceTypeLidarAvia || n->code_!=info->broadcast_code || n->ip_!=info->ip) return;
    uint8_t h=0;
    if (AddLidarToConnect(info->broadcast_code,&h)==kStatusSuccess) {
      n->handle_=h;SetDataCallback(h,data,n);
    }
  }
  static void state(const DeviceInfo* info,DeviceEvent event) {
    auto* n=instance_;if (!n || n->stopping_ || !info || n->code_!=info->broadcast_code) return;
    n->handle_=info->handle;
    if (event==kEventDisconnect) {
      n->connected_=false;n->sampling_=false;n->configuring_=false;
      std::lock_guard<std::mutex> lock(n->mutex_);n->buffer_.clear();n->previous_time_=0;
      RCLCPP_WARN(n->get_logger(),"Avia disconnected; waiting for rediscovery");return;
    }
    if (!n->connected_.exchange(true)) QueryDeviceInformation(info->handle,device_info,n);
    if (info->state==kLidarStateNormal && !n->sampling_ && !n->configuring_.exchange(true)) {
      n->stage_=0;n->configure_next();
    }
  }
  static void device_info(livox_status status,uint8_t,DeviceInformationResponse* response,void* context) {
    auto* n=static_cast<AviaNode*>(context);
    if (n->stopping_ || status!=kStatusSuccess || !response || response->ret_code!=0) return;
    const uint8_t* v=response->firmware_version;
    const std::string firmware=std::to_string(v[0])+"."+std::to_string(v[1])+"."+std::to_string(v[2])+"."+std::to_string(v[3]);
    {std::lock_guard<std::mutex> lock(n->mutex_);n->firmware_=firmware;}
    RCLCPP_INFO(n->get_logger(),"Avia firmware %s",firmware.c_str());
  }
  void configure_next() {
    if (stopping_ || !connected_) return;
    livox_status result=kStatusFailure;
    switch (stage_.load()) {
      case 0: result=SetCartesianCoordinate(handle_,configured,this);break;
      case 1: result=LidarSetPointCloudReturnMode(handle_,kFirstReturn,configured,this);break;
      // The built-in IMU at 200 Hz, published on /avia/imu.
      case 2: result=LidarSetImuPushFrequency(handle_,kImuFreq200Hz,configured,this);break;
      case 3: result=LidarStartSampling(handle_,configured,this);break;
      default: sampling_=true;configuring_=false;RCLCPP_INFO(get_logger(),"Avia streaming Cartesian single-return data");return;
    }
    if (result!=kStatusSuccess) configuration_failed();
  }
  void configuration_failed() {
    ++config_errors_;configuring_=false;sampling_=false;
    RCLCPP_ERROR(get_logger(),"Avia configuration failed at stage %d; service will reconnect",stage_.load());
  }
  static void configured(livox_status result,uint8_t,uint8_t response,void* context) {
    auto* n=static_cast<AviaNode*>(context);if (n->stopping_) return;
    if (result!=kStatusSuccess || response!=0) { n->configuration_failed();return; }
    ++n->stage_;n->configure_next();
  }

  // GPS-style synchronization: once per second, the UTC time of the PPS pulse
  // that began this second, sent utc_phase after it. The time comes from the
  // host clock, which chrony disciplines to the same receiver PPS; the kernel's
  // error bound gates every send. Deliberately not gated on the Avia's own PPS
  // flag: the push is harmless without a pulse, and the status word shows what
  // the Avia makes of it.
  void utc_loop() {
    int64_t last_labelled=-1;
    while (!stopping_) {
      int64_t wait=phase_ns_-second_fraction(realtime_ns());
      if (wait<=0) wait+=ns_per_s;
      // Relative, monotonic, and at most ~1 s: a clock step can never park the thread.
      sleep_monotonic_ns(std::min<int64_t>(wait,1100000000));
      if (stopping_) break;
      const int64_t now=realtime_ns();int64_t sec=0;
      if (!uav_avia::pulse_second(now,lo_ns_,hi_ns_,sec)) {
        if (second_fraction(now)>hi_ns_) skip(4);  // too late for this pulse; never label the next one early
        continue;
      }
      if (sec==last_labelled) continue;
      last_labelled=sec;
      push_utc(sec);
    }
  }
  void skip(int reason) {++utc_skipped_;utc_skip_reason_=reason;}
  void push_utc(int64_t sec) {
    if (!connected_ || !sampling_) {skip(1);return;}
    timex tx{};
    const int clock_state=adjtimex(&tx);  // modes 0: read only
    clock_maxerror_us_=tx.maxerror;
    if (clock_state==TIME_ERROR || (tx.status&STA_UNSYNC) || tx.maxerror>std::lround(max_clock_error_*1e6)) {skip(2);return;}
    uav_avia::UtcFields f{};
    if (!uav_avia::utc_fields(sec,f)) {skip(3);return;}
    LidarSetUtcSyncTimeRequest req{};
    req.year=f.year;req.month=f.month;req.day=f.day;req.hour=f.hour;req.mircrosecond=f.microsecond;
    if (LidarSetUtcSyncTime(handle_,&req,utc_acked,this)==kStatusSuccess) ++utc_sent_;
    else ++utc_errors_;
  }
  static void utc_acked(livox_status result,uint8_t,uint8_t response,void* context) {
    auto* n=static_cast<AviaNode*>(context);if (n->stopping_) return;
    if (result==kStatusSuccess && response==0) {
      ++n->utc_acked_;
      if (!n->logged_ack_.exchange(true)) RCLCPP_INFO(n->get_logger(),"Avia accepted the UTC time command");
    } else {
      ++n->utc_rejected_;
      if (!n->logged_reject_.exchange(true))
        RCLCPP_WARN(n->get_logger(),"Avia did not accept the UTC time command (SDK status %d, response %u)",static_cast<int>(result),response);
    }
  }

  static void data(uint8_t,LivoxEthPacket* packet,uint32_t count,void* context) {
    auto* n=static_cast<AviaNode*>(context);if (n->stopping_ || !packet) return;
    const auto received=Steady::now();const builtin_interfaces::msg::Time stamp=n->get_clock()->now();
    const int64_t receipt_ns=static_cast<int64_t>(stamp.sec)*ns_per_s+stamp.nanosec;
    if (packet->version==5 && packet->data_type==kImu && count==1) {n->imu(packet,received,stamp,receipt_ns);return;}
    std::lock_guard<std::mutex> lock(n->mutex_);
    if (packet->version!=5) {++n->bad_;return;}
    if (packet->data_type!=kExtendCartesian || count!=96) {++n->bad_;return;}
    const auto raw=uav_avia::read<uint64_t>(packet->timestamp);
    // Monotonic packet time for gap and regression checks: uptime or PTP
    // nanoseconds as they are, a UTC stamp decoded. PPS-only stamps (type 4)
    // restart at every edge, so they are not checked.
    int64_t sensor_ns=0;bool trusted=false,have_sensor=false;uint64_t mono=0;
    if (packet->timestamp_type==0) mono=raw;
    else if (packet->timestamp_type==uav_avia::timestamp_ptp) {
      mono=raw;sensor_ns=static_cast<int64_t>(raw)-n->ptp_utc_offset_ns_;have_sensor=true;
    } else if (packet->timestamp_type==uav_avia::timestamp_utc) {
      if (uav_avia::utc_stamp_ns(packet->timestamp,sensor_ns)) {mono=static_cast<uint64_t>(sensor_ns);have_sensor=true;}
      else ++n->bad_utc_;
    }
    if (have_sensor) {
      const int64_t latency=receipt_ns-sensor_ns;
      trusted=uav_avia::sensor_time_trusted(packet->timestamp_type,packet->err_code,latency,n->lat_min_ns_,n->lat_max_ns_);
      if (!n->lat_n_ || latency<n->lat_lo_) n->lat_lo_=latency;
      if (!n->lat_n_ || latency>n->lat_hi_) n->lat_hi_=latency;
      n->lat_sum_+=static_cast<double>(latency);++n->lat_n_;
      if (!trusted) ++n->untrusted_utc_;
    }
    if (mono && n->previous_time_ && packet->timestamp_type==n->previous_type_) {
      if (mono<=n->previous_time_) {++n->regressions_;n->buffer_.clear();}
      else if (mono-n->previous_time_>600000) ++n->gap_events_; // Expected ~400 us/packet, generous 1.5x diagnostic bound.
    }
    if (packet->timestamp_type!=n->previous_type_) n->buffer_.clear();
    n->previous_time_=mono;n->previous_type_=packet->timestamp_type;
    n->raw_time_=raw;n->timestamp_type_=packet->timestamp_type;n->status_=packet->err_code;
    n->last_point_=received;++n->packets_;n->slots_+=count;
    if (n->buffer_.size()/uav_avia::point_step+count>100000) {n->buffer_.clear();++n->overflows_;}
    if (n->buffer_.empty()) {
      n->first_receipt_=stamp;n->first_steady_=received;n->first_sensor_ns_=sensor_ns;n->first_trusted_=trusted;
    }
    for (uint32_t i=0;i<count;++i)
      if (uav_avia::append_point(n->buffer_,packet->data+i*sizeof(LivoxExtendRawPoint),i,raw,packet->timestamp_type,packet->err_code)) ++n->valid_;
  }
  // An IMU sample, stamped like the clouds: sensor UTC when its packet passes
  // the same trust checks, host receipt otherwise. The SDK reports g; ROS wants m/s^2.
  void imu(const LivoxEthPacket* packet,Steady::time_point received,const builtin_interfaces::msg::Time& receipt,int64_t receipt_ns) {
    int64_t sensor_ns=0;
    const bool have=uav_avia::point_time_ns(uav_avia::read<uint64_t>(packet->timestamp),packet->timestamp_type,0,ptp_utc_offset_ns_,sensor_ns);
    const bool sensor=have && header_policy_=="sensor_utc" &&
      uav_avia::sensor_time_trusted(packet->timestamp_type,packet->err_code,receipt_ns-sensor_ns,lat_min_ns_,lat_max_ns_);
    LivoxImuPoint p{};std::memcpy(&p,packet->data,sizeof(p));
    constexpr double g=9.80665;
    Imu msg;msg.header.frame_id=imu_frame_;msg.header.stamp=sensor?to_msg(sensor_ns):receipt;
    msg.orientation_covariance[0]=-1.0;  // no orientation estimate
    msg.angular_velocity.x=p.gyro_x;msg.angular_velocity.y=p.gyro_y;msg.angular_velocity.z=p.gyro_z;
    msg.linear_acceleration.x=p.acc_x*g;msg.linear_acceleration.y=p.acc_y*g;msg.linear_acceleration.z=p.acc_z*g;
    {std::lock_guard<std::mutex> lock(mutex_);++imu_packets_;last_imu_=received;++(sensor?imu_sensor_:imu_receipt_);}
    imu_pub_->publish(std::move(msg));
  }
  // Livox's CustomMsg for FAST-LIO: timebase = the frame's first point, every
  // point's offset from its packet stamp plus its slot in the packet. Only for
  // frames on trusted sensor time (the IMU is then on the same clock), and only
  // when something subscribes.
  void publish_custom(const Cloud& cloud,int64_t base_ns) {
    Custom out;out.header.frame_id=frame_;out.header.stamp=to_msg(base_ns);
    out.timebase=static_cast<uint64_t>(base_ns);out.lidar_id=0;out.points.reserve(cloud.width);
    const uint8_t* p=cloud.data.data();
    for (uint32_t i=0;i<cloud.width;++i,p+=uav_avia::point_step) {
      const uint64_t raw=uav_avia::read<uint32_t>(p+20)|(static_cast<uint64_t>(uav_avia::read<uint32_t>(p+24))<<32);
      int64_t t=0;
      if (!uav_avia::point_time_ns(raw,p[18],uav_avia::read<uint32_t>(p+28),ptp_utc_offset_ns_,t) ||
          t<base_ns || t-base_ns>ns_per_s) {++custom_points_dropped_;continue;}
      livox_ros_driver2::msg::CustomPoint q;
      q.offset_time=static_cast<uint32_t>(t-base_ns);
      q.x=uav_avia::read<float>(p);q.y=uav_avia::read<float>(p+4);q.z=uav_avia::read<float>(p+8);
      q.reflectivity=static_cast<uint8_t>(std::clamp(uav_avia::read<float>(p+12),0.0f,255.0f));
      q.tag=p[16];q.line=p[17];
      out.points.push_back(q);
    }
    out.point_num=static_cast<uint32_t>(out.points.size());
    custom_pub_->publish(std::move(out));++customs_;
  }
  void publish_cloud() {
    if (config_errors_.load()>0) throw std::runtime_error("Avia setup failed; restart required");
    Cloud msg;bool sensor=false;int64_t base_ns=0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_point_==Steady::time_point{} || Steady::now()-last_point_>std::chrono::duration<double>(timeout_)) {buffer_.clear();return;}
      msg.header.frame_id=frame_;
      // Sensor UTC of the cloud's first packet when that packet passed every
      // check; otherwise the host receipt time of that packet.
      sensor=header_policy_=="sensor_utc" && !buffer_.empty() && first_trusted_;
      base_ns=first_sensor_ns_;
      msg.header.stamp=buffer_.empty()?static_cast<builtin_interfaces::msg::Time>(get_clock()->now()):
                                       (sensor?to_msg(first_sensor_ns_):first_receipt_);
      // Never emit a backlog as a fresh scan after the executor has stalled.
      if (!buffer_.empty() && Steady::now()-first_steady_>500ms) {buffer_.clear();++overflows_;return;}
      if (!buffer_.empty()) ++(sensor?clouds_sensor_:clouds_receipt_);
      msg.data.swap(buffer_);++clouds_;
    }
    msg.height=1;msg.width=msg.data.size()/uav_avia::point_step;msg.fields=fields_;
    msg.point_step=uav_avia::point_step;msg.row_step=msg.width*msg.point_step;
    msg.is_bigendian=false;msg.is_dense=true;
    if (sensor && msg.width>0 && custom_pub_->get_subscription_count()>0) publish_custom(msg,base_ns);
    cloud_pub_->publish(std::move(msg));
  }
  static void value(Diag& d,const std::string& key,const std::string& val) {
    diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=val;d.values.push_back(kv);
  }
  template<class T> static void value(Diag& d,const std::string& key,T val) {value(d,key,std::to_string(val));}
  static double age(Steady::time_point p,Steady::time_point now) {
    return p==Steady::time_point{}?-1.0:std::chrono::duration<double>(now-p).count();
  }
  void diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray out;out.header.stamp=get_clock()->now();
    Diag stream,clock;stream.name="avia/driver";clock.name="avia/clock";
    stream.hardware_id=code_;clock.hardware_id=code_;
    std::lock_guard<std::mutex> lock(mutex_);const auto now=Steady::now();
    const auto point_age=age(last_point_,now),imu_age=age(last_imu_,now);
    const double dt=std::chrono::duration<double>(now-last_diag_).count();
    const double packet_hz=(packets_-previous_packets_)/dt,cloud_hz=(clouds_-previous_clouds_)/dt;
    const bool fresh=connected_ && point_age>=0 && point_age<timeout_;
    const uint32_t faults=status_&0xc0001dffU;
    stream.level=!connected_?Diag::ERROR:(!fresh || faults || cloud_hz<rate_*.7 || packet_hz<2000?Diag::WARN:Diag::OK);
    stream.message=!fresh?"Waiting for Avia point data":(stream.level==Diag::OK?"Receiving point clouds":"Stream rate or device status warning");
    value(stream,"ip",ip_);value(stream,"firmware",firmware_);value(stream,"frame_id",frame_);value(stream,"connected",connected_.load());
    value(stream,"point_packet_hz",packet_hz);value(stream,"cloud_hz",cloud_hz);
    value(stream,"imu_packet_hz",(imu_packets_-previous_imu_)/dt);value(stream,"point_age_sec",point_age);value(stream,"imu_age_sec",imu_age);
    value(stream,"point_slots_total",slots_);value(stream,"valid_points_total",valid_);value(stream,"zero_returns_removed",slots_-valid_);
    value(stream,"imu_published_sensor_time",imu_sensor_);value(stream,"imu_published_receipt_time",imu_receipt_);
    value(stream,"lio_frames_published",customs_);value(stream,"lio_points_dropped",custom_points_dropped_);
    value(stream,"unsupported_packets",bad_);value(stream,"buffer_drops",overflows_);value(stream,"packet_gap_events",gap_events_);value(stream,"timestamp_regressions",regressions_);value(stream,"status_code",status_);

    // Clock: which time the headers carry, and the evidence for it.
    const uint32_t sync=uav_avia::status_time_sync(status_);
    const bool pps=uav_avia::status_pps_ok(status_),ptp=uav_avia::status_ptp_ok(status_);
    const uint64_t by_sensor=clouds_sensor_-prev_clouds_sensor_,by_receipt=clouds_receipt_-prev_clouds_receipt_;
    const bool validated=fresh && header_policy_=="sensor_utc" && by_sensor>0 && by_receipt==0;
    std::string why;
    if (!fresh) why="No fresh sensor time evidence";
    else if (validated) why=timestamp_type_==uav_avia::timestamp_ptp?"Sensor time from PTP":"Sensor UTC from PPS + pushed TOD";
    else if (header_policy_!="sensor_utc") why="Host receipt timestamps (configured)";
    else if (timestamp_type_==0) why="Host receipt timestamps: the Avia is unsynchronized (no PTP lock, no PPS)";
    else if (timestamp_type_==4) why="Host receipt timestamps: PPS only, no UTC accepted yet";
    else if (timestamp_type_==uav_avia::timestamp_ptp && !(ptp && sync==uav_avia::time_sync_ptp))
      why="Host receipt timestamps: PTP stamps without a live PTP lock (sync mode "+std::to_string(sync)+")";
    else if (timestamp_type_==uav_avia::timestamp_utc && !(pps && sync==uav_avia::time_sync_gps))
      why="Host receipt timestamps: UTC stamps without PPS + GPS sync (sync mode "+std::to_string(sync)+")";
    else if (untrusted_utc_) why="Host receipt timestamps: sensor time failed the receipt-latency check";
    else why="Host receipt timestamps";
    clock.level=validated?Diag::OK:Diag::WARN;
    clock.message=why;
    value(clock,"header_stamp_policy",header_policy_);
    value(clock,"header_stamp_source",std::string(by_sensor>0?(by_receipt>0?"mixed":"sensor_utc"):"host_receipt"));
    value(clock,"timing_validated",std::string(validated?"true":"false"));
    value(clock,"fusion_ready",std::string("false"));  // extrinsics are still nominal
    value(clock,"timestamp_type",timestamp_type_);value(clock,"time_sync_status",sync);
    value(clock,"pps_status",pps?1U:0U);value(clock,"ptp_status",ptp?1U:0U);
    value(clock,"ptp_utc_offset_sec",ptp_utc_offset_ns_/ns_per_s);
    value(clock,"utc_push",std::string(utc_sync_?"on":"off"));
    value(clock,"utc_commands_sent",utc_sent_.load());value(clock,"utc_commands_acked",utc_acked_.load());
    value(clock,"utc_commands_rejected",utc_rejected_.load());value(clock,"utc_command_errors",utc_errors_.load());
    value(clock,"utc_pushes_skipped",utc_skipped_.load());
    value(clock,"utc_last_skip_reason",std::string(skip_reasons[utc_skip_reason_.load()]));
    value(clock,"host_clock_maxerror_us",clock_maxerror_us_.load());
    if (lat_n_) {
      value(clock,"sensor_latency_ms_min",lat_lo_/1e6);value(clock,"sensor_latency_ms_mean",lat_sum_/lat_n_/1e6);
      value(clock,"sensor_latency_ms_max",lat_hi_/1e6);
    }
    value(clock,"stamped_packets_untrusted",untrusted_utc_);value(clock,"utc_stamps_invalid_total",bad_utc_);
    value(clock,"packet_timestamp_raw",raw_time_);value(clock,"nominal_extrinsics",std::string("true"));
    if (fresh) {
      if (static_cast<int>(pps)!=last_pps_) {
        RCLCPP_INFO(get_logger(),"Avia PPS input: %s",pps?"present":"absent");last_pps_=pps;
      }
      if (static_cast<int>(ptp)!=last_ptp_) {
        RCLCPP_INFO(get_logger(),"Avia PTP lock: %s",ptp?"yes":"no");last_ptp_=ptp;
      }
      if (sync!=last_sync_) {
        RCLCPP_INFO(get_logger(),"Avia time sync mode %u (0 none, 1 PTP, 2 GPS, 3 PPS, 4 abnormal); packet timestamp type %u",
                    sync,static_cast<unsigned>(timestamp_type_));
        last_sync_=sync;
      }
      if (validated!=last_validated_) {
        if (validated) RCLCPP_INFO(get_logger(),"Headers now carry sensor time (%s; receipt latency %.2f-%.2f ms)",why.c_str(),lat_lo_/1e6,lat_hi_/1e6);
        else RCLCPP_WARN(get_logger(),"Headers back on host receipt time: %s",why.c_str());
        last_validated_=validated;
      }
    }
    out.status={stream,clock};diag_pub_->publish(out);
    previous_packets_=packets_;previous_clouds_=clouds_;previous_imu_=imu_packets_;last_diag_=now;
    prev_clouds_sensor_=clouds_sensor_;prev_clouds_receipt_=clouds_receipt_;
    lat_n_=0;lat_sum_=0;untrusted_utc_=0;
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {auto node=std::make_shared<AviaNode>();rclcpp::spin(node);}
  catch(const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("avia_driver"),"%s",e.what());result=1;}
  if(rclcpp::ok()) rclcpp::shutdown();
  return result;
}
