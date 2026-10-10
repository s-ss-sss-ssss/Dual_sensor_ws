#include "serial_frame_bridge/serial_bridge.h"

#include <algorithm>
#include <utility>

namespace serial_frame_bridge {
namespace {
constexpr std::size_t kMaxTxBytes = 256 * 1024;
constexpr std::size_t kMaxTxMessages = 128;
}  // namespace

SerialBridge::SerialBridge(std::string port, unsigned int baud_rate, Handler on_frame)
    : port_name_(std::move(port)), baud_rate_(baud_rate), on_frame_(std::move(on_frame)),
      port_(io_), retry_(io_) {}

SerialBridge::~SerialBridge() { stop(); }

void SerialBridge::start() {
  if (started_.exchange(true)) return;
  io_.reset();
  stopping_ = false;
  retry_index_ = 0;
  work_.reset(new boost::asio::io_service::work(io_));
  io_.post([this] { connect(); });
  thread_ = std::thread([this] { io_.run(); });
}

void SerialBridge::stop() {
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!started_.exchange(false)) return;
    io_.post([this] {
      stopping_ = true;
      boost::system::error_code ignored;
      retry_.cancel(ignored);
      disconnect();
      work_.reset();
    });
  }
  thread_.join();
}

void SerialBridge::send(std::vector<uint8_t> bytes) {
  if (bytes.empty()) return;
  const auto size = bytes.size();
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (!started_) return;
  if (pending_messages_ >= kMaxTxMessages || size > kMaxTxBytes - pending_bytes_) {
      ++tx_dropped_;
      return;
  }
  ++pending_messages_;
  pending_bytes_ += size;
  io_.post([this, bytes = std::move(bytes), size]() mutable {
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      --pending_messages_;
      pending_bytes_ -= size;
    }
    if (stopping_ || !connected_) {
      ++tx_dropped_;
      return;
    }
    while (!tx_queue_.empty() &&
           (tx_queue_.size() >= kMaxTxMessages ||
            tx_bytes_ + bytes.size() > kMaxTxBytes)) {
      tx_bytes_ -= tx_queue_.front().size();
      tx_queue_.pop_front();
      ++tx_dropped_;
    }
    if (tx_bytes_ + bytes.size() > kMaxTxBytes) {
      ++tx_dropped_;
      return;
    }
    tx_bytes_ += bytes.size();
    tx_queue_.push_back(std::move(bytes));
    write();
  });
}

SerialBridge::Stats SerialBridge::stats() const {
  return {connected_.load(), reconnects_.load(), read_errors_.load(), write_errors_.load(),
          tx_dropped_.load(), frames_.load(), crc_errors_.load(), header_errors_.load(),
          resync_bytes_.load()};
}

void SerialBridge::disconnect() {
  connected_ = false;
  ++generation_;
  boost::system::error_code ignored;
  port_.cancel(ignored);
  port_.close(ignored);
  parser_.reset();
  // Never replay opaque commands after a connection loss.
  tx_dropped_ += tx_queue_.size() + (writing_ ? 1 : 0);
  tx_queue_.clear();
  tx_bytes_ = 0;
  writing_ = false;
}

void SerialBridge::schedule_reconnect() {
  if (stopping_) return;
  disconnect();
  const int delays_ms[] = {500, 1000, 2000, 5000};
  const int delay = delays_ms[std::min<unsigned int>(retry_index_++, 3)];
  retry_.expires_from_now(boost::posix_time::milliseconds(delay));
  retry_.async_wait([this](const boost::system::error_code& ec) {
    if (!ec && !stopping_) connect();
  });
}

void SerialBridge::connect() {
  if (stopping_) return;
  ++reconnects_;
  boost::system::error_code ec;
  port_.open(port_name_, ec);
  if (!ec) port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_), ec);
  if (!ec) port_.set_option(boost::asio::serial_port_base::character_size(8), ec);
  if (!ec) port_.set_option(boost::asio::serial_port_base::parity(
                                boost::asio::serial_port_base::parity::none), ec);
  if (!ec) port_.set_option(boost::asio::serial_port_base::stop_bits(
                                boost::asio::serial_port_base::stop_bits::one), ec);
  if (!ec) port_.set_option(boost::asio::serial_port_base::flow_control(
                                boost::asio::serial_port_base::flow_control::none), ec);
  if (ec) {
    schedule_reconnect();
    return;
  }
  connected_ = true;
  retry_index_ = 0;
  read();
  write();
}

void SerialBridge::read() {
  const auto generation = generation_;
  auto buffer = std::make_shared<std::array<uint8_t, 4096>>();
  port_.async_read_some(boost::asio::buffer(*buffer),
                        [this, buffer, generation](const boost::system::error_code& ec, std::size_t n) {
    if (generation != generation_ || stopping_ || !connected_) return;
    if (ec) {
      ++read_errors_;
      schedule_reconnect();
      return;
    }
    parser_.feed(buffer->data(), n, on_frame_);
    const auto& s = parser_.stats();
    frames_ = s.frames;
    crc_errors_ = s.crc_errors;
    header_errors_ = s.header_errors;
    resync_bytes_ = s.resync_bytes;
    read();
  });
}

void SerialBridge::write() {
  if (writing_ || !connected_ || tx_queue_.empty()) return;
  auto bytes = std::make_shared<std::vector<uint8_t>>(std::move(tx_queue_.front()));
  tx_bytes_ -= bytes->size();
  tx_queue_.pop_front();
  writing_ = true;
  const auto generation = generation_;
  boost::asio::async_write(port_, boost::asio::buffer(*bytes),
                           [this, bytes, generation](const boost::system::error_code& ec, std::size_t) {
    if (generation != generation_) return;
    writing_ = false;
    if (stopping_ || !connected_) return;
    if (ec) {
      ++write_errors_;
      ++tx_dropped_;
      schedule_reconnect();
      return;
    }
    write();
  });
}

}  // namespace serial_frame_bridge
