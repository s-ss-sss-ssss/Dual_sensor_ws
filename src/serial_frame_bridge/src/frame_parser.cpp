#include "serial_frame_bridge/frame_parser.h"

#include <algorithm>
#include <iterator>

namespace serial_frame_bridge {
constexpr std::size_t FrameParser::kMaxPayload;
constexpr std::size_t FrameParser::kMaxBuffer;
namespace {
uint16_t crc16(const uint8_t* data, std::size_t size) {
  uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}
}  // namespace

void FrameParser::reset() { buffer_.clear(); }

void FrameParser::feed(const uint8_t* data, std::size_t size, const Handler& handler) {
  // Process bounded chunks even when a caller supplies an arbitrarily large read.
  while (size) {
    const std::size_t count = std::min(size, kMaxBuffer - buffer_.size());
    buffer_.insert(buffer_.end(), data, data + count);
    data += count;
    size -= count;

    std::size_t pos = 0;
    while (pos < buffer_.size()) {
      const uint8_t sync[] = {0xa5, 0x5a};
      auto start = std::search(buffer_.begin() + pos, buffer_.end(),
                               std::begin(sync), std::end(sync));
      if (start == buffer_.end()) {
        const std::size_t keep = buffer_.back() == 0xa5 ? 1 : 0;
        stats_.resync_bytes += buffer_.size() - pos - keep;
        pos = buffer_.size() - keep;
        break;
      }
      stats_.resync_bytes += static_cast<std::size_t>(start - (buffer_.begin() + pos));
      pos = static_cast<std::size_t>(start - buffer_.begin());
      if (buffer_.size() - pos < 14) break;
      const std::size_t payload = buffer_[pos + 12] | (static_cast<std::size_t>(buffer_[pos + 13]) << 8);
      if (buffer_[pos + 2] != 2 || buffer_[pos + 3] != 0x10 || payload > kMaxPayload) {
        ++stats_.header_errors;
        ++stats_.resync_bytes;
        ++pos;
        continue;
      }
      const std::size_t total = 16 + payload;
      if (buffer_.size() - pos < total) break;
      const uint16_t expected = buffer_[pos + total - 2] |
          (static_cast<uint16_t>(buffer_[pos + total - 1]) << 8);
      if (crc16(buffer_.data() + pos + 2, total - 4) != expected) {
        ++stats_.crc_errors;
        ++stats_.resync_bytes;
        ++pos;
        continue;
      }
      ++stats_.frames;
      handler(Frame(buffer_.begin() + pos, buffer_.begin() + pos + total));
      pos += total;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
  }
}

}  // namespace serial_frame_bridge
