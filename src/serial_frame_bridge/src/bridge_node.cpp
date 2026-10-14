#include "serial_frame_bridge/serial_bridge.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <serial_frame_bridge/RawFrame.h>
#include <std_msgs/UInt8MultiArray.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <utility>

namespace {
class BridgeNode {
 public:
  BridgeNode() : private_nh_("~"),
                 bridge_(private_nh_.param<std::string>("port", "/dev/maggrad"),
                         static_cast<unsigned int>(std::max(1, private_nh_.param("baud_rate", 115200))),
                         [this](serial_frame_bridge::FrameParser::Frame frame) { enqueue(std::move(frame)); }),
                 queue_limit_(static_cast<std::size_t>(std::max(1, private_nh_.param("frame_queue_size", 512)))),
                 batch_limit_(static_cast<std::size_t>(std::max(1, private_nh_.param("publish_batch_size", 512)))) {
    raw_pub_ = private_nh_.advertise<serial_frame_bridge::RawFrame>("raw_frames", 5);
    diag_pub_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    tx_sub_ = private_nh_.subscribe("tx_bytes", 10, &BridgeNode::onTx, this);
    publish_timer_ = private_nh_.createWallTimer(ros::WallDuration(0.001), &BridgeNode::publishFrames, this);
    diag_timer_ = private_nh_.createWallTimer(ros::WallDuration(1), &BridgeNode::publishDiagnostics, this);
    bridge_.start();
  }

 private:
  struct QueuedFrame {
    ros::Time received_at;
    serial_frame_bridge::FrameParser::Frame data;
  };

  void enqueue(serial_frame_bridge::FrameParser::Frame frame) {
    const auto received_at = ros::Time::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.size() == queue_limit_) {
      frames_.pop_front();
      ++rx_dropped_;
    }
    frames_.push_back({received_at, std::move(frame)});
  }

  void onTx(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    bridge_.send(msg->data);
  }

  void publishFrames(const ros::WallTimerEvent&) {
    std::deque<QueuedFrame> batch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const std::size_t count = std::min(batch_limit_, frames_.size());
      for (std::size_t i = 0; i < count; ++i) {
        batch.push_back(std::move(frames_.front()));
        frames_.pop_front();
      }
    }
    for (auto& frame : batch) {
      serial_frame_bridge::RawFrame msg;
      msg.header.stamp = frame.received_at;
      msg.data = std::move(frame.data);
      raw_pub_.publish(msg);
    }
  }

  void publishDiagnostics(const ros::WallTimerEvent&) {
    const auto s = bridge_.stats();
    uint64_t dropped;
    std::size_t queued;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dropped = rx_dropped_;
      queued = frames_.size();
    }
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = ros::this_node::getName() + ": serial bridge";
    status.hardware_id = private_nh_.param<std::string>("port", "/dev/maggrad");
    status.level = s.connected ? diagnostic_msgs::DiagnosticStatus::OK
                               : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = s.connected ? "connected" : "disconnected/retrying";
    auto add = [&](const std::string& key, uint64_t value) {
      diagnostic_msgs::KeyValue entry;
      entry.key = key;
      entry.value = std::to_string(value);
      status.values.push_back(std::move(entry));
    };
    add("frames", s.frames);
    add("crc_errors", s.crc_errors);
    add("header_errors", s.header_errors);
    add("resync_bytes", s.resync_bytes);
    add("read_errors", s.read_errors);
    add("write_errors", s.write_errors);
    add("connection_attempts", s.reconnects);
    add("rx_queue_dropped", dropped);
    add("rx_queue_size", queued);
    add("tx_dropped", s.tx_dropped);
    array.status.push_back(std::move(status));
    diag_pub_.publish(array);
  }

  ros::NodeHandle private_nh_;
  serial_frame_bridge::SerialBridge bridge_;
  const std::size_t queue_limit_;
  const std::size_t batch_limit_;
  std::mutex mutex_;
  std::deque<QueuedFrame> frames_;
  uint64_t rx_dropped_ = 0;
  ros::Publisher raw_pub_, diag_pub_;
  ros::Subscriber tx_sub_;
  ros::WallTimer publish_timer_, diag_timer_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_frame_bridge");
  BridgeNode node;
  ros::spin();
  return 0;
}
