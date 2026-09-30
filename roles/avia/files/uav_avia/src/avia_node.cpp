#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <livox_sdk.h>
#include "uav_avia/point.hpp"
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This Livox wire decoder requires a little-endian host.
#endif
using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;
using Cloud = sensor_msgs::msg::PointCloud2;
using Diag = diagnostic_msgs::msg::DiagnosticStatus;

class AviaNode : public rclcpp::Node {
 public:
  AviaNode() : Node("avia_driver") {
    code_=declare_parameter<std::string>("broadcast_code","3JEDNAP001S5701");
    ip_=declare_parameter<std::string>("lidar_ip","192.168.144.80");
    frame_=declare_parameter<std::string>("frame_id","avia_nominal_lidar_frame");
    rate_=declare_parameter<double>("publish_rate_hz",10.0);
    timeout_=declare_parameter<double>("receipt_timeout_sec",2.0);
    if (rate_<1 || rate_>50 || timeout_<=0 || code_.empty() || ip_.empty())
      throw std::invalid_argument("Invalid Avia configuration");
    cloud_pub_=create_publisher<Cloud>("/avia/points",rclcpp::SensorDataQoS().keep_last(2));
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    fields_=fields();buffer_.reserve(30000*uav_avia::point_step);
    instance_=this;
    if (!Init()) throw std::runtime_error("Livox SDK initialization failed (port already in use?)");
    initialized_=true;
    SetBroadcastCallback(broadcast);SetDeviceStateUpdateCallback(state);
    if (!Start()) { Uninit();initialized_=false;throw std::runtime_error("Livox discovery start failed"); }
    timer_=create_wall_timer(std::chrono::duration<double>(1.0/rate_),[this]{publish_cloud();});
    diag_timer_=create_wall_timer(1s,[this]{diagnostics();});
    RCLCPP_INFO(get_logger(),"Waiting for Avia %s at %s; headers use first packet host receipt time; synchronization unverified",code_.c_str(),ip_.c_str());
  }
  ~AviaNode() override {
    stopping_=true;
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
  std::string code_,ip_,frame_;
  double rate_,timeout_;
  std::mutex mutex_;
  std::vector<uint8_t> buffer_;
  std::vector<sensor_msgs::msg::PointField> fields_;
  builtin_interfaces::msg::Time first_receipt_;
  Steady::time_point first_steady_{},last_point_{},last_imu_{},last_diag_=Steady::now();
  uint64_t packets_=0,imu_packets_=0,slots_=0,valid_=0,clouds_=0,bad_=0,overflows_=0;
  uint64_t gap_events_=0,regressions_=0,previous_time_=0,raw_time_=0;
  uint64_t previous_packets_=0,previous_clouds_=0,previous_imu_=0;
  uint32_t status_=0;
  uint8_t timestamp_type_=255,previous_type_=255;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
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
    n->connected_=true;
    if (info->state==kLidarStateNormal && !n->sampling_ && !n->configuring_.exchange(true)) {
      n->stage_=0;n->configure_next();
    }
  }
  void configure_next() {
    if (stopping_ || !connected_) return;
    livox_status result=kStatusFailure;
    switch (stage_.load()) {
      case 0: result=SetCartesianCoordinate(handle_,configured,this);break;
      case 1: result=LidarSetPointCloudReturnMode(handle_,kFirstReturn,configured,this);break;
      // Receive IMU packets for transport health only. No IMU ROS topic is asserted yet.
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
  static void data(uint8_t,LivoxEthPacket* packet,uint32_t count,void* context) {
    auto* n=static_cast<AviaNode*>(context);if (n->stopping_ || !packet) return;
    const auto received=Steady::now();const builtin_interfaces::msg::Time stamp=n->get_clock()->now();
    std::lock_guard<std::mutex> lock(n->mutex_);
    if (packet->version!=5) {++n->bad_;return;}
    if (packet->data_type==kImu && count==1) {++n->imu_packets_;n->last_imu_=received;return;}
    if (packet->data_type!=kExtendCartesian || count!=96) {++n->bad_;return;}
    const auto raw=uav_avia::read<uint64_t>(packet->timestamp);
    if (n->previous_time_ && packet->timestamp_type==n->previous_type_ && (packet->timestamp_type==0 || packet->timestamp_type==1)) {
      if (raw<=n->previous_time_) {++n->regressions_;n->buffer_.clear();}
      else if (raw-n->previous_time_>600000) ++n->gap_events_; // Expected ~400 us/packet, generous 1.5x diagnostic bound.
    }
    if (packet->timestamp_type!=n->previous_type_) n->buffer_.clear();
    n->previous_time_=raw;n->previous_type_=packet->timestamp_type;
    n->raw_time_=raw;n->timestamp_type_=packet->timestamp_type;n->status_=packet->err_code;
    n->last_point_=received;++n->packets_;n->slots_+=count;
    if (n->buffer_.size()/uav_avia::point_step+count>100000) {n->buffer_.clear();++n->overflows_;}
    if (n->buffer_.empty()) {n->first_receipt_=stamp;n->first_steady_=received;}
    for (uint32_t i=0;i<count;++i)
      if (uav_avia::append_point(n->buffer_,packet->data+i*sizeof(LivoxExtendRawPoint),i,raw,packet->timestamp_type,packet->err_code)) ++n->valid_;
  }
  void publish_cloud() {
    if (config_errors_.load()>0) throw std::runtime_error("Avia setup failed; restart required");
    Cloud msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_point_==Steady::time_point{} || Steady::now()-last_point_>std::chrono::duration<double>(timeout_)) {buffer_.clear();return;}
      msg.header.frame_id=frame_;
      msg.header.stamp=buffer_.empty()?static_cast<builtin_interfaces::msg::Time>(get_clock()->now()):first_receipt_;
      // Never emit a backlog as a fresh scan after the executor has stalled.
      if (!buffer_.empty() && Steady::now()-first_steady_>500ms) {buffer_.clear();++overflows_;return;}
      msg.data.swap(buffer_);++clouds_;
    }
    msg.height=1;msg.width=msg.data.size()/uav_avia::point_step;msg.fields=fields_;
    msg.point_step=uav_avia::point_step;msg.row_step=msg.width*msg.point_step;
    msg.is_bigendian=false;msg.is_dense=true;cloud_pub_->publish(std::move(msg));
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
    value(stream,"ip",ip_);value(stream,"frame_id",frame_);value(stream,"connected",connected_.load());
    value(stream,"point_packet_hz",packet_hz);value(stream,"cloud_hz",cloud_hz);
    value(stream,"imu_packet_hz",(imu_packets_-previous_imu_)/dt);value(stream,"point_age_sec",point_age);value(stream,"imu_age_sec",imu_age);
    value(stream,"point_slots_total",slots_);value(stream,"valid_points_total",valid_);value(stream,"zero_returns_removed",slots_-valid_);
    value(stream,"unsupported_packets",bad_);value(stream,"buffer_drops",overflows_);value(stream,"packet_gap_events",gap_events_);value(stream,"timestamp_regressions",regressions_);value(stream,"status_code",status_);
    clock.level=Diag::WARN;  // Timing is independent of connection severity.
    clock.message=fresh?"Host receipt timestamps; sensor synchronization not validated":"No fresh sensor time evidence";
    value(clock,"header_stamp_policy",std::string("first_packet_host_receipt"));value(clock,"fusion_ready",std::string("false"));
    value(clock,"timestamp_type",timestamp_type_);value(clock,"time_sync_status",(status_>>14)&7U);
    value(clock,"packet_timestamp_raw",raw_time_);value(clock,"nominal_extrinsics",std::string("true"));
    out.status={stream,clock};diag_pub_->publish(out);
    previous_packets_=packets_;previous_clouds_=clouds_;previous_imu_=imu_packets_;last_diag_=now;
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
