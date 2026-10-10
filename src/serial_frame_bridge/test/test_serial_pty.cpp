#include "serial_frame_bridge/serial_bridge.h"

#include <gtest/gtest.h>
#include <pty.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

TEST(SerialBridge, ReadsSplitsWritesAndReconnectsWithoutHardware) {
  int master = -1, slave = -1;
  char name[128] = {};
  ASSERT_EQ(0, openpty(&master, &slave, name, nullptr, nullptr));
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::vector<uint8_t>> received;
  serial_frame_bridge::SerialBridge bridge(name, 115200, [&](std::vector<uint8_t> bytes) {
    std::lock_guard<std::mutex> lock(mutex);
    received.push_back(std::move(bytes));
    cv.notify_one();
  });
  bridge.start();
  auto wait_connected = [&] {
    for (int i = 0; i < 200; ++i) {
      if (bridge.stats().connected) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  ASSERT_TRUE(wait_connected());
  // Empty payload ROUND frame, CRC-16/CCITT-FALSE of bytes [2..13].
  std::vector<uint8_t> expected = {0xa5, 0x5a, 2, 0x10, 1, 0, 0, 0,
                                   2, 0, 0, 0, 0, 0, 0, 0};
  // Compute test CRC independently to avoid depending on a hard-coded checksum.
  uint16_t crc = 0xffff;
  for (std::size_t i = 2; i < 14; ++i) {
    crc ^= static_cast<uint16_t>(expected[i]) << 8;
    for (int j = 0; j < 8; ++j)
      crc = crc & 0x8000 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  }
  expected[14] = crc & 255;
  expected[15] = crc >> 8;
  ASSERT_EQ(5, ::write(master, expected.data(), 5));
  ASSERT_EQ(11, ::write(master, expected.data() + 5, 11));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&] { return !received.empty(); }));
    EXPECT_EQ(expected, received.front());
  }
  bridge.send({0x11, 0x22, 0x33});
  pollfd pfd{master, POLLIN, 0};
  ASSERT_EQ(1, ::poll(&pfd, 1, 2000));
  uint8_t reply[3] = {};
  ASSERT_EQ(3, ::read(master, reply, 3));
  EXPECT_EQ((std::vector<uint8_t>{0x11, 0x22, 0x33}),
            (std::vector<uint8_t>(reply, reply + 3)));
  bridge.stop();
  bridge.start();
  ASSERT_TRUE(wait_connected());
  bridge.send({0x44});
  ASSERT_EQ(1, ::poll(&pfd, 1, 2000));
  uint8_t restarted_reply = 0;
  ASSERT_EQ(1, ::read(master, &restarted_reply, 1));
  EXPECT_EQ(0x44, restarted_reply);
  ::close(master);
  ::close(slave);
  // A closed PTY should be reported as disconnected; retries need no device.
  for (int i = 0; i < 200 && bridge.stats().connected; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(bridge.stats().connected);
  const auto dropped = bridge.stats().tx_dropped;
  bridge.send({0x55});
  for (int i = 0; i < 100 && bridge.stats().tx_dropped == dropped; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GT(bridge.stats().tx_dropped, dropped);
  bridge.stop();
}
