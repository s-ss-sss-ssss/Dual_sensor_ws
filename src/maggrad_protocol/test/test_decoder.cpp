#include "maggrad_protocol/decoder.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using Bytes = std::vector<uint8_t>;
void u16(Bytes& b, uint16_t x) { b.push_back(x & 255); b.push_back(x >> 8); }
void u32(Bytes& b, uint32_t x) { u16(b, x & 65535); u16(b, x >> 16); }
Bytes tlv(uint8_t kind, const Bytes& body) {
  Bytes b{kind}; u16(b, body.size()); b.insert(b.end(), body.begin(), body.end()); return b;
}
void append(Bytes& b, const Bytes& other) { b.insert(b.end(), other.begin(), other.end()); }
Bytes frame(uint32_t seq, uint32_t tick, const Bytes& payload = {}) {
  Bytes b{0xa5, 0x5a, 2, 0x10}; u32(b, seq); u32(b, tick); u16(b, payload.size());
  append(b, payload);
  uint16_t crc = 0xffff;
  for (size_t i = 2; i < b.size(); ++i) {
    crc ^= uint16_t(b[i]) << 8;
    for (int j = 0; j < 8; ++j)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  u16(b, crc); return b;
}
int main() {
  using namespace maggrad_protocol;
  Expected expected;
  expected.ak = {{2, 3}, {1, 1}};
  expected.tmag = {{8, 0x35}}; expected.icm = true;
  Decoder d(expected); Round r;
  Bytes ak; u32(ak, 123); ak.insert(ak.end(), {2, 3});
  u16(ak, 0xffff); u16(ak, 4); u16(ak, 0xfffb);
  ak.insert(ak.end(), {1, 0, 0});
  Bytes tmag; u32(tmag, 456); tmag.insert(tmag.end(), {8, 0x35});
  u16(tmag, 0xfffe); u16(tmag, 7); u16(tmag, 9); tmag.insert(tmag.end(), {1, 0});
  Bytes icm; u32(icm, 789);
  for (int x : {1, 2, 3, 4, 5, 6, -7}) u16(icm, uint16_t(x));
  Bytes phase; u32(phase, 0); u32(phase, 0xffffffffu); u32(phase, 42);
  Bytes err{4}; u16(err, 1); u32(err, 99);
  Bytes stats; for (uint32_t x = 1; x <= 5; ++x) u32(stats, x);
    u32(stats, 0x89abcdefu);
  Bytes all;
  for (const auto& rec : {tlv(1, ak), tlv(2, tmag), tlv(3, icm), tlv(4, phase),
                          tlv(0xe0, err), tlv(0xf0, stats), tlv(0x55, {1, 2})}) append(all, rec);
  assert(d.decode(frame(0xfffffffeu, 0xfffffffeu, all), r));
  assert(r.ak.size() == 1 && r.ak[0].hx == -1 && r.ak[0].offset_us == 123);
  assert(r.tmag[0].x == -2 && r.icm.temp == -7 && r.errors[0].detail == 99);
  assert(r.has_phase && r.phase_age_us[1] == 0xffffffffu && r.has_stats);
    assert(r.stats.ak_frames == 1 && r.stats.tmag_frames == 2 && r.stats.icm_frames == 3);
    assert(r.stats.skipped == 4 && r.stats.errors == 5 && r.stats.device_id == 0x89abcdefu);
  assert(!r.complete && r.missing_sensor_mask == 2);
  Bytes complete = all;
  Bytes second_ak = ak; second_ak[4] = 1; second_ak[5] = 1;
  append(complete, tlv(1, second_ak));
  assert(d.decode(frame(0xffffffffu, 0xffffffffu, complete), r));
  assert(r.complete && r.missing_sensor_mask == 0 && r.ak.size() == 2);
  Bytes duplicates; append(duplicates, tlv(1, ak)); ak[0] = 77; append(duplicates, tlv(1, ak));
  append(duplicates, tlv(3, icm)); append(duplicates, tlv(3, icm));
  assert(d.decode(frame(1, 2, duplicates), r));
  assert(r.ak.size() == 1 && r.ak[0].offset_us == 77);
  assert(d.counters().duplicate_sensors == 2 && d.counters().seq_holes == 1);
  assert(r.unwrapped_tick_us == 0x100000002ull);
  assert(d.decode(frame(1, 2), r) && d.counters().seq_repeats == 1);
  assert(d.decode(frame(0, 1), r) && d.counters().seq_out_of_order == 1);
  assert(d.counters().tick_out_of_order == 1 && d.counters().suspected_restarts == 1);
  auto bad = frame(2, 3);
  bad.back() ^= 1; assert(!d.decode(bad, r) && d.counters().crc_errors == 1);
  bad = frame(2, 3); bad[2] = 1; assert(!d.decode(bad, r) && d.counters().version_errors == 1);
  bad = frame(2, 3); bad[3] = 2; assert(!d.decode(bad, r));
  bad = frame(2, 3); bad[12] = 1; assert(!d.decode(bad, r) && d.counters().length_errors == 1);
  bad = frame(2, 3, tlv(1, Bytes(14, 0)));
  assert(!d.decode(bad, r) && d.counters().record_errors == 1);
  assert(!d.decode(frame(2, 3, Bytes{1, 15}), r));
  assert(!d.decode(frame(2, 3, Bytes{1, 15, 0, 0}), r));
  assert(d.decode(frame(2, 3), r) && r.unwrapped_tick_us == 0x100000003ull);
  assert(d.counters().seq_holes == 1);
  Bytes max_payload;
  append(max_payload, tlv(0x55, Bytes(4077, 0)));
  assert(d.decode(frame(3, 4, max_payload), r));
  append(max_payload, tlv(0x56, {}));
  assert(!d.decode(frame(4, 5, max_payload), r));
  try { Expected invalid; invalid.ak = {{2, 3}, {2, 3}}; Decoder ignored(invalid); assert(false); }
  catch (const std::invalid_argument&) {}
  {
      Decoder stats_decoder;
      Round stats_round;
      assert(stats_decoder.decode(frame(1, 1, tlv(0xf0, stats)), stats_round));
      assert(stats_round.has_stats && stats_round.stats.device_id == 0x89abcdefu);
      for (size_t length : {size_t(20), size_t(23), size_t(25)}) {
        assert(!stats_decoder.decode(frame(2, 2, tlv(0xf0, Bytes(length, 0))), stats_round));
        assert(stats_round.seq == 1 && stats_round.stats.device_id == 0x89abcdefu);
      }
      assert(stats_decoder.counters().record_errors == 3);
      for (uint32_t id : {uint32_t(0), uint32_t(0xffffffffu)}) {
        Bytes body(stats.begin(), stats.begin() + 20);
        u32(body, id);
        assert(stats_decoder.decode(frame(2, 2, tlv(0xf0, body)), stats_round));
        assert(stats_round.has_stats && stats_round.stats.device_id == id);
      }
      assert(stats_decoder.counters().seq_holes == 0);
      assert(stats_decoder.decode(frame(3, 3), stats_round));
      assert(!stats_round.has_stats && stats_round.stats.device_id == 0);
    }
  {
    Decoder timing;
    Round timed;
    assert(timing.decode(frame(1, 1234567), timed));
    assert(timed.tick_us == 1234567 && timed.unwrapped_tick_us == 1234567);
    assert(timing.decode(frame(2, 1234890), timed));
    assert(timed.unwrapped_tick_us == 1234890);
    assert(timing.decode(frame(3, 1234890), timed));
    assert(timed.unwrapped_tick_us == 1234890 && timing.counters().tick_out_of_order == 0);
    assert(timing.decode(frame(4, 1234889), timed));
    assert(timed.tick_us == 1234889 && timed.unwrapped_tick_us == 1234890);
    assert(timing.counters().tick_out_of_order == 1);
    timing.reset_tracking();
    assert(timing.decode(frame(1, 10), timed));
    assert(timed.unwrapped_tick_us == 10);
    assert(timing.decode(frame(2, 0x80000009u), timed));
    assert(timed.unwrapped_tick_us == 0x80000009ull);
    assert(timing.decode(frame(3, 9), timed));
    assert(timed.unwrapped_tick_us == 0x80000009ull);
    assert(timing.counters().tick_out_of_order == 2);
    timing.reset_tracking();
    assert(timing.decode(frame(1, 0xfffffff0u, tlv(1, ak)), timed));
    assert(!timed.has_phase);
    assert(timed.unwrapped_tick_us + timed.ak[0].offset_us == 0x10000003dull);
    auto invalid = frame(2, 0x10);
    invalid.back() ^= 1;
    assert(!timing.decode(invalid, timed));
    assert(timing.decode(frame(2, 0x10), timed));
    assert(timed.tick_us == 0x10 && timed.unwrapped_tick_us == 0x100000010ull);
    assert(timing.counters().seq_holes == 0);
  }
  std::cout << "decoder tests passed\n";
}
