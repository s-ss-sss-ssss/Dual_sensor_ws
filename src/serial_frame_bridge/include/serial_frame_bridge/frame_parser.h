#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace serial_frame_bridge {

class FrameParser {
 public:
  static constexpr std::size_t kMaxPayload = 4080;
  static constexpr std::size_t kMaxBuffer = 256 * 1024;
  using Frame = std::vector<uint8_t>;
  using Handler = std::function<void(Frame)>;

  struct Stats {
    uint64_t frames = 0;
    uint64_t crc_errors = 0;
    uint64_t header_errors = 0;
    uint64_t resync_bytes = 0;
  };

  void feed(const uint8_t* data, std::size_t size, const Handler& handler);
  void reset();
  const Stats& stats() const { return stats_; }
  std::size_t buffered_bytes() const { return buffer_.size(); }

 private:
  std::vector<uint8_t> buffer_;
  Stats stats_;
};

}  // namespace serial_frame_bridge
