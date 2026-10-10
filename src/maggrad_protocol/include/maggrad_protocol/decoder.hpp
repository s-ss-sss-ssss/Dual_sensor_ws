#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace maggrad_protocol {

using SensorId = std::pair<uint8_t, uint8_t>;

struct Ak {
  uint32_t offset_us = 0;
  uint8_t bus = 0, mask = 0;
  int16_t hx = 0, hy = 0, hz = 0;
  uint8_t drdy = 0, err = 0, dor = 0;
};
struct Tmag {
  uint32_t offset_us = 0;
  uint8_t ch_mask = 0, addr = 0;
  int16_t x = 0, y = 0, z = 0;
  uint8_t drdy = 0, overflow = 0;
};
struct Icm {
  uint32_t offset_us = 0;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;
};
struct Error {
  uint8_t source = 0;
  uint16_t code = 0;
  uint32_t detail = 0;
};
struct Stats {
  uint32_t ak_frames = 0, tmag_frames = 0, icm_frames = 0, skipped = 0, errors = 0;
    uint32_t device_id = 0;
};
struct Round {
  uint32_t seq = 0, tick_ms = 0;
  uint64_t unwrapped_tick_ms = 0;
  bool has_phase = false, has_icm = false, has_stats = false, complete = false;
  std::array<uint32_t, 3> phase_age_us{{0xffffffffu, 0xffffffffu, 0xffffffffu}};
  std::vector<Ak> ak;
  std::vector<Tmag> tmag;
  Icm icm;
  std::vector<Error> errors;
  Stats stats;
  uint64_t missing_sensor_mask = 0;
};
struct Counters {
  uint64_t frames = 0, malformed = 0, crc_errors = 0, length_errors = 0;
  uint64_t version_errors = 0, record_errors = 0, duplicate_sensors = 0;
  uint64_t seq_holes = 0, seq_repeats = 0, seq_out_of_order = 0;
  uint64_t tick_out_of_order = 0, suspected_restarts = 0;
};
struct Expected {
  std::vector<SensorId> ak, tmag;
  bool icm = false;
};

// Decodes one complete wire frame (not a byte stream). Invalid frames never
// advance sequence or time state; error counters do advance.
class Decoder {
 public:
  explicit Decoder(Expected expected = {});
  bool decode(const uint8_t* data, size_t size, Round& out, std::string* error = nullptr);
  bool decode(const std::vector<uint8_t>& data, Round& out, std::string* error = nullptr) {
    return decode(data.data(), data.size(), out, error);
  }
  const Counters& counters() const { return counters_; }
  const Expected& expected() const { return expected_; }
  void reset_tracking();  // call after a known device reconnect/restart

 private:
  Expected expected_;
  Counters counters_;
  bool have_seq_ = false, have_tick_ = false;
  uint32_t last_seq_ = 0, last_tick_ = 0;
  uint64_t unwrapped_tick_ = 0;
};

}  // namespace maggrad_protocol
