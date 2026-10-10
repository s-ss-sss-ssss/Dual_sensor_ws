#pragma once

#include "serial_frame_bridge/frame_parser.h"

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace serial_frame_bridge {

class SerialBridge {
 public:
  using Handler = FrameParser::Handler;
  struct Stats {
    bool connected;
    uint64_t reconnects;
    uint64_t read_errors;
    uint64_t write_errors;
    uint64_t tx_dropped;
    uint64_t frames;
    uint64_t crc_errors;
    uint64_t header_errors;
    uint64_t resync_bytes;
  };

  SerialBridge(std::string port, unsigned int baud_rate, Handler on_frame);
  ~SerialBridge();
  SerialBridge(const SerialBridge&) = delete;
  SerialBridge& operator=(const SerialBridge&) = delete;
  void start();
  void stop();
  void send(std::vector<uint8_t> bytes);
  Stats stats() const;

 private:
  void connect();
  void schedule_reconnect();
  void disconnect();
  void read();
  void write();

  const std::string port_name_;
  const unsigned int baud_rate_;
  Handler on_frame_;
  boost::asio::io_service io_;
  boost::asio::serial_port port_;
  boost::asio::deadline_timer retry_;
  std::unique_ptr<boost::asio::io_service::work> work_;
  std::thread thread_;
  FrameParser parser_;

  std::deque<std::vector<uint8_t>> tx_queue_;

  std::size_t tx_bytes_ = 0;
  std::mutex pending_mutex_;
  std::size_t pending_messages_ = 0;
  std::size_t pending_bytes_ = 0;
  bool writing_ = false;
  std::atomic<bool> started_{false};
  bool stopping_ = false;
  uint64_t generation_ = 0;
  unsigned int retry_index_ = 0;
  std::atomic<bool> connected_{false};
  std::atomic<uint64_t> reconnects_{0}, read_errors_{0}, write_errors_{0}, tx_dropped_{0};
  std::atomic<uint64_t> frames_{0}, crc_errors_{0}, header_errors_{0}, resync_bytes_{0};
};

}  // namespace serial_frame_bridge
