#include "maggrad_protocol/decoder.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <maggrad_protocol/AkSample.h>
#include <maggrad_protocol/ErrorRecord.h>
#include <maggrad_protocol/IcmSample.h>
#include <maggrad_protocol/RoundSample.h>
#include <maggrad_protocol/StatsRecord.h>
#include <maggrad_protocol/TmagSample.h>
#include <ros/ros.h>
#include <serial_frame_bridge/RawFrame.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using Clock = std::chrono::steady_clock;
using maggrad_protocol::Decoder;
using maggrad_protocol::Expected;
using maggrad_protocol::Round;

std::vector<maggrad_protocol::SensorId> readIds(ros::NodeHandle& nh, const char* name) {
  XmlRpc::XmlRpcValue values;
  if (!nh.getParam(name, values)) return {};
  if (values.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument(std::string(name) + " must be a list of [u8, u8]");
  std::vector<maggrad_protocol::SensorId> ids;
  for (int i = 0; i < values.size(); ++i) {
    const auto& pair = values[i];
    if (pair.getType() != XmlRpc::XmlRpcValue::TypeArray || pair.size() != 2 ||
        pair[0].getType() != XmlRpc::XmlRpcValue::TypeInt ||
        pair[1].getType() != XmlRpc::XmlRpcValue::TypeInt ||
        int(pair[0]) < 0 || int(pair[0]) > 255 || int(pair[1]) < 0 || int(pair[1]) > 255)
      throw std::invalid_argument(std::string(name) + " entries must be [u8, u8]");
    ids.emplace_back(static_cast<uint8_t>(int(pair[0])), static_cast<uint8_t>(int(pair[1])));
  }
  return ids;
}
Expected readExpected(ros::NodeHandle& nh) {
  Expected expected;
  expected.ak = readIds(nh, "expected_ak");
  expected.tmag = readIds(nh, "expected_tmag");
  nh.param("expect_icm", expected.icm, false);
  return expected;
}

template <typename T>
void field(diagnostic_msgs::DiagnosticStatus& status, const char* name, T value) {
  diagnostic_msgs::KeyValue kv;
  kv.key = name;
  kv.value = std::to_string(value);
  status.values.push_back(kv);
}

uint8_t mapAkSensorId(uint8_t /*bus*/, uint8_t /*mask*/) {
  // TODO: Confirm AK bus+mask -> logical ID mapping; do not infer addresses.
  return 0xFF;
}

uint8_t mapTmagSensorId(uint8_t /*ch_mask*/, uint8_t /*addr*/) {
  // TODO: Confirm TMAG ch_mask+addr -> logical ID mapping; do not infer addresses.
  return 0xFF;
}

maggrad_protocol::RoundSample message(const serial_frame_bridge::RawFrame& frame, const Round& r) {
  maggrad_protocol::RoundSample msg;
  msg.header = frame.header;
  msg.round_seq = r.seq;
  msg.device_tick_ms = r.tick_ms;
  msg.device_tick_unwrapped_ms = r.unwrapped_tick_ms;
  msg.has_phase = r.has_phase;
  for (int i = 0; i < 3; ++i) {
    msg.phase_age_us[i] = r.phase_age_us[i];
    msg.phase_valid[i] = r.has_phase && r.phase_age_us[i] != 0xffffffffu;
  }
  for (const auto& a : r.ak) {
    maggrad_protocol::AkSample m;
    m.read_done_offset_us = a.offset_us; m.bus = a.bus; m.mask = a.mask;
    m.sensor_id = mapAkSensorId(a.bus, a.mask);
    m.hx = a.hx; m.hy = a.hy; m.hz = a.hz;
    m.drdy = a.drdy; m.err = a.err; m.dor = a.dor;
    msg.ak.push_back(m);
  }
  for (const auto& t : r.tmag) {
    maggrad_protocol::TmagSample m;
    m.read_done_offset_us = t.offset_us; m.ch_mask = t.ch_mask; m.addr = t.addr;
    m.sensor_id = mapTmagSensorId(t.ch_mask, t.addr);
    m.x = t.x; m.y = t.y; m.z = t.z;
    m.drdy = t.drdy; m.overflow = t.overflow;
    msg.tmag.push_back(m);
  }
  msg.has_icm = r.has_icm;
  if (r.has_icm) {
    msg.icm.read_done_offset_us = r.icm.offset_us;
    msg.icm.ax = r.icm.ax; msg.icm.ay = r.icm.ay; msg.icm.az = r.icm.az;
    msg.icm.gx = r.icm.gx; msg.icm.gy = r.icm.gy; msg.icm.gz = r.icm.gz;
    msg.icm.temp = r.icm.temp;
  }
  for (const auto& e : r.errors) {
    maggrad_protocol::ErrorRecord m;
    m.source = e.source; m.code = e.code; m.detail = e.detail;
    msg.errors.push_back(m);
  }
  msg.has_stats = r.has_stats;
  if (r.has_stats) {
    msg.stats.ak_frames = r.stats.ak_frames; msg.stats.tmag_frames = r.stats.tmag_frames;
    msg.stats.icm_frames = r.stats.icm_frames; msg.stats.skipped = r.stats.skipped;
    msg.stats.errors = r.stats.errors;
        msg.stats.device_id = r.stats.device_id;
  }
  msg.complete = r.complete;
  msg.missing_sensor_mask = r.missing_sensor_mask;
  return msg;
}

class ProtocolNode {
 public:
  ProtocolNode() : private_("~"), decoder_(readExpected(private_)), start_(Clock::now()),
                   last_report_(start_) {
    std::string topic;
    private_.param<std::string>("raw_frame_topic", topic, "serial_frame_bridge/raw_frames");
    private_.param<std::string>("sample_topic", sample_topic_, "maggrad/sample");
    const auto diagnostics_topic = private_.param<std::string>("diagnostics_topic", "diagnostics");
    publisher_ = nh_.advertise<maggrad_protocol::RoundSample>(sample_topic_, 5);
    diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 5);
    subscriber_ = nh_.subscribe(topic, 512, &ProtocolNode::onFrame, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &ProtocolNode::onDiagnostics, this);
    ROS_INFO_STREAM("MagGrad protocol subscribing to " << topic << ", publishing " << sample_topic_);
  }

 private:
  void onFrame(const serial_frame_bridge::RawFrame::ConstPtr& frame) {
    Round round;
    std::string error;
    if (!decoder_.decode(frame->data, round, &error)) {
      ROS_WARN_STREAM_THROTTLE(5.0, "Discarding malformed ROUND: " << error);
      return;
    }
    last_received_ = Clock::now();
    received_ = true;
    if (round.has_stats) { stats_ = round.stats; have_stats_ = true; }
    publisher_.publish(message(*frame, round));
  }

  void onDiagnostics(const ros::WallTimerEvent&) {
    const auto now = Clock::now();
    const auto& c = decoder_.counters();
    const double elapsed = std::chrono::duration<double>(now - last_report_).count();
    const double age = std::chrono::duration<double>(now - (received_ ? last_received_ : start_)).count();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = ros::this_node::getName() + ": protocol";
    status.hardware_id = "maggrad_v2";
    if (age > 1.0 || !received_) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "No recent valid rounds";
    } else if (age > 0.1 || c.malformed > last_bad_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Stale rounds or malformed frames";
    } else if (decoder_.expected().ak.empty() && decoder_.expected().tmag.empty() &&
               !decoder_.expected().icm) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Expected sensors not configured";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "Receiving rounds";
    }
    field(status, "rate_hz", elapsed > 0 ? (c.frames - last_count_) / elapsed : 0.0);
    field(status, "last_round_age_s", age);
    field(status, "frames", c.frames); field(status, "malformed", c.malformed);
    field(status, "crc_errors", c.crc_errors); field(status, "length_errors", c.length_errors);
    field(status, "version_errors", c.version_errors); field(status, "record_errors", c.record_errors);
    field(status, "seq_holes", c.seq_holes); field(status, "seq_repeats", c.seq_repeats);
    field(status, "seq_out_of_order", c.seq_out_of_order);
    field(status, "suspected_restarts", c.suspected_restarts);
    field(status, "tick_out_of_order", c.tick_out_of_order);
    field(status, "duplicate_sensors", c.duplicate_sensors);
    if (have_stats_) {
      field(status, "firmware_skipped", stats_.skipped);
      field(status, "firmware_errors", stats_.errors);
    }
    diagnostic_msgs::DiagnosticArray report;
    report.header.stamp = ros::Time::now();
    report.status.push_back(status);
    diagnostics_.publish(report);
    last_report_ = now; last_count_ = c.frames; last_bad_ = c.malformed;
  }

  ros::NodeHandle nh_, private_;
  Decoder decoder_;
  ros::Publisher publisher_, diagnostics_;
  ros::Subscriber subscriber_;
  ros::WallTimer timer_;
  std::string sample_topic_;
  Clock::time_point start_, last_report_, last_received_;
  bool received_ = false, have_stats_ = false;
  maggrad_protocol::Stats stats_;
  uint64_t last_count_ = 0, last_bad_ = 0;
};
}  // namespace

#ifndef MAGGRAD_PROTOCOL_TEST
int main(int argc, char** argv) {
  ros::init(argc, argv, "maggrad_protocol");
  try { ProtocolNode node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL_STREAM("Invalid protocol configuration: " << e.what()); return 1; }
  return 0;
}
#endif
