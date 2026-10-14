// Exercise the production message conversion without a ROS master.
#define MAGGRAD_PROTOCOL_TEST
#include "../src/protocol_node.cpp"

#include <cassert>
#include <iostream>

int main() {
  // No address pair may accidentally become logical ID 0 before mapping is confirmed.
  for (unsigned int first = 0; first < 256; ++first) {
    for (unsigned int second = 0; second < 256; ++second) {
      assert(mapAkSensorId(first, second) == 0xFF);
      assert(mapTmagSensorId(first, second) == 0xFF);
    }
  }
  serial_frame_bridge::RawFrame frame;
  frame.header.stamp = ros::Time(123, 456);
  frame.header.frame_id = "board_a";
  Round round;
  round.tick_us = 0xfffffff0u;
  round.unwrapped_tick_us = 0x1fffffff0ull;
  maggrad_protocol::Ak ak;
  ak.offset_us = 123; ak.bus = 2; ak.mask = 8;
  ak.hx = -1; ak.hy = 2; ak.hz = -3;
  ak.drdy = 1; ak.err = 2; ak.dor = 3;
  round.ak.push_back(ak);
  maggrad_protocol::Tmag tmag;
  tmag.offset_us = 456; tmag.ch_mask = 4; tmag.addr = 0x35;
  tmag.x = -4; tmag.y = 5; tmag.z = -6;
  tmag.drdy = 1; tmag.overflow = 2;
  round.tmag.push_back(tmag);
  const auto msg = message(frame, round);
  assert(msg.header.stamp == frame.header.stamp && msg.header.frame_id == "board_a");
  assert(msg.device_tick_us == round.tick_us);
  assert(msg.device_tick_unwrapped_us == round.unwrapped_tick_us);
  assert(!msg.has_phase && !msg.phase_valid[0] && !msg.phase_valid[1] && !msg.phase_valid[2]);
  assert(msg.ak.size() == 1 && msg.tmag.size() == 1);
  const auto& a = msg.ak[0];
  assert(a.sensor_id == 0xFF && a.bus == 2 && a.mask == 8 && a.read_done_offset_us == 123);
  assert(msg.device_tick_unwrapped_us + a.read_done_offset_us == 0x20000006bull);
  assert(a.hx == -1 && a.hy == 2 && a.hz == -3 && a.drdy == 1 && a.err == 2 && a.dor == 3);
  const auto& t = msg.tmag[0];
  assert(t.sensor_id == 0xFF && t.ch_mask == 4 && t.addr == 0x35 && t.read_done_offset_us == 456);
  assert(t.x == -4 && t.y == 5 && t.z == -6 && t.drdy == 1 && t.overflow == 2);
  std::cout << "Protocol message conversion tests passed\n";
}
