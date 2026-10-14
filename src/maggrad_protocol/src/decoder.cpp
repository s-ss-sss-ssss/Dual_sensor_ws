#include "maggrad_protocol/decoder.hpp"

#include <algorithm>
#include <stdexcept>

namespace maggrad_protocol {
namespace {
uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t u32(const uint8_t* p) { return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16); }
int16_t i16(const uint8_t* p) { return static_cast<int16_t>(u16(p)); }
uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(p[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
void validate_ids(const std::vector<SensorId>& ids) {
  for (size_t i = 0; i < ids.size(); ++i)
    if (std::find(ids.begin(), ids.begin() + i, ids[i]) != ids.begin() + i)
      throw std::invalid_argument("duplicate expected sensor identity");
}
}  // namespace

Decoder::Decoder(Expected expected) : expected_(std::move(expected)) {
  if (expected_.ak.size() + expected_.tmag.size() + size_t(expected_.icm) > 64)
    throw std::invalid_argument("missing_sensor_mask supports at most 64 sensors");
  validate_ids(expected_.ak);
  validate_ids(expected_.tmag);
}

void Decoder::reset_tracking() {
  have_seq_ = have_tick_ = false;
  last_seq_ = last_tick_ = 0;
  unwrapped_tick_ = 0;
}

bool Decoder::decode(const uint8_t* data, size_t size, Round& out, std::string* error) {
  auto fail = [&](uint64_t& category, const char* reason) {
    ++counters_.malformed;
    ++category;
    if (error) *error = reason;
    return false;
  };
  if (!data || size < 16 || data[0] != 0xa5 || data[1] != 0x5a)
    return fail(counters_.length_errors, "missing or truncated ROUND header");
  if (data[2] != 2 || data[3] != 0x10)
    return fail(counters_.version_errors, "unsupported version or frame type");
  const size_t payload_len = u16(data + 12);
  if (payload_len > 4080 || size != 16 + payload_len)
    return fail(counters_.length_errors, "invalid frame length");
  if (crc16(data + 2, 12 + payload_len) != u16(data + 14 + payload_len))
    return fail(counters_.crc_errors, "CRC-16/CCITT-FALSE mismatch");

  Round round;
  round.seq = u32(data + 4);
  round.tick_us = u32(data + 8);
  uint64_t duplicates = 0;
  const size_t end = 14 + payload_len;
  for (size_t pos = 14; pos < end;) {
    if (end - pos < 3) return fail(counters_.record_errors, "truncated TLV header");
    const uint8_t kind = data[pos];
    const size_t len = u16(data + pos + 1);
    pos += 3;
    if (len > end - pos) return fail(counters_.record_errors, "TLV exceeds payload");
    const uint8_t* p = data + pos;
    pos += len;
    size_t required = 0;
    switch (kind) {
      case 1: required = 15; break;
      case 2: required = 14; break;
      case 3: required = 18; break;
      case 4: required = 12; break;
      case 0xe0: required = 7; break;
      case 0xf0: required = 24; break;
      default: continue;
    }
    if (len != required) return fail(counters_.record_errors, "incorrect known TLV length");
    if (kind == 1) {
      Ak item;
      item.offset_us = u32(p); item.bus = p[4]; item.mask = p[5];
      item.hx = i16(p + 6); item.hy = i16(p + 8); item.hz = i16(p + 10);
      item.drdy = p[12]; item.err = p[13]; item.dor = p[14];
      auto it = std::find_if(round.ak.begin(), round.ak.end(), [&](const Ak& a) {
        return a.bus == item.bus && a.mask == item.mask;
      });
      if (it == round.ak.end()) round.ak.push_back(item);
      else { *it = item; ++duplicates; }
    } else if (kind == 2) {
      Tmag item;
      item.offset_us = u32(p); item.ch_mask = p[4]; item.addr = p[5];
      item.x = i16(p + 6); item.y = i16(p + 8); item.z = i16(p + 10);
      item.drdy = p[12]; item.overflow = p[13];
      auto it = std::find_if(round.tmag.begin(), round.tmag.end(), [&](const Tmag& t) {
        return t.ch_mask == item.ch_mask && t.addr == item.addr;
      });
      if (it == round.tmag.end()) round.tmag.push_back(item);
      else { *it = item; ++duplicates; }
    } else if (kind == 3) {
      if (round.has_icm) ++duplicates;
      round.has_icm = true;
      round.icm.offset_us = u32(p);
      round.icm.ax = i16(p + 4); round.icm.ay = i16(p + 6); round.icm.az = i16(p + 8);
      round.icm.gx = i16(p + 10); round.icm.gy = i16(p + 12);
      round.icm.gz = i16(p + 14); round.icm.temp = i16(p + 16);
    } else if (kind == 4) {
      round.has_phase = true;
      for (int i = 0; i < 3; ++i) round.phase_age_us[i] = u32(p + 4 * i);
    } else if (kind == 0xe0) {
      round.errors.push_back(Error{p[0], u16(p + 1), u32(p + 3)});
    } else {
      round.has_stats = true;
      round.stats = Stats{u32(p), u32(p + 4), u32(p + 8), u32(p + 12), u32(p + 16), u32(p + 20)};
    }
  }

  size_t bit = 0;
  for (const auto& id : expected_.ak) {
    if (std::none_of(round.ak.begin(), round.ak.end(), [&](const Ak& a) {
      return a.bus == id.first && a.mask == id.second;
    })) round.missing_sensor_mask |= uint64_t(1) << bit;
    ++bit;
  }
  for (const auto& id : expected_.tmag) {
    if (std::none_of(round.tmag.begin(), round.tmag.end(), [&](const Tmag& t) {
      return t.ch_mask == id.first && t.addr == id.second;
    })) round.missing_sensor_mask |= uint64_t(1) << bit;
    ++bit;
  }
  if (expected_.icm && !round.has_icm) round.missing_sensor_mask |= uint64_t(1) << bit;
  round.complete = round.missing_sensor_mask == 0;

  if (!have_seq_) { have_seq_ = true; last_seq_ = round.seq; }
  else {
    const uint32_t delta = round.seq - last_seq_;
    if (!delta) ++counters_.seq_repeats;
    else if (delta < 0x80000000u) { counters_.seq_holes += delta - 1; last_seq_ = round.seq; }
    else {
      ++counters_.seq_out_of_order;
      if (round.seq < last_seq_ && round.tick_us < last_tick_) ++counters_.suspected_restarts;
    }
  }
  if (!have_tick_) { have_tick_ = true; last_tick_ = round.tick_us; unwrapped_tick_ = round.tick_us; }
  else {
    const uint32_t delta = round.tick_us - last_tick_;
    if (delta < 0x80000000u) { unwrapped_tick_ += delta; last_tick_ = round.tick_us; }
    else ++counters_.tick_out_of_order;
  }
  round.unwrapped_tick_us = unwrapped_tick_;
  counters_.duplicate_sensors += duplicates;
  ++counters_.frames;
  out = std::move(round);
  if (error) error->clear();
  return true;
}
}  // namespace maggrad_protocol
