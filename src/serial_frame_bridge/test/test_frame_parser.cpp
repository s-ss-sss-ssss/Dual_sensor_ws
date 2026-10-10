#include "serial_frame_bridge/frame_parser.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

using serial_frame_bridge::FrameParser;
namespace {
std::vector<uint8_t> frame(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> bytes = {0xa5, 0x5a, 2, 0x10, 1, 0, 0, 0, 2, 0, 0, 0,
                                static_cast<uint8_t>(payload.size() & 255),
                                static_cast<uint8_t>(payload.size() >> 8)};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  uint16_t crc = 0xffff;
  for (std::size_t i = 2; i < bytes.size(); ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = crc & 0x8000 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  }
  bytes.push_back(crc & 255);
  bytes.push_back(crc >> 8);
  return bytes;
}
}  // namespace

TEST(FrameParser, SplitAtEveryByteAndStickyFrames) {
  auto a = frame({0xa5, 0x5a, 0, 42});
  auto b = frame({});
  for (std::size_t split = 0; split <= a.size(); ++split) {
    FrameParser parser;
    std::vector<FrameParser::Frame> found;
    auto collect = [&](FrameParser::Frame f) { found.push_back(std::move(f)); };
    parser.feed(a.data(), split, collect);
    std::vector<uint8_t> rest(a.begin() + split, a.end());
    rest.insert(rest.end(), b.begin(), b.end());
    parser.feed(rest.data(), rest.size(), collect);
    ASSERT_EQ(2u, found.size());
    EXPECT_EQ(a, found[0]);
    EXPECT_EQ(b, found[1]);
  }
}

TEST(FrameParser, NoiseInvalidHeadersCrcAndNestedSync) {
  FrameParser parser;
  auto bad_crc = frame({1});
  bad_crc.back() ^= 0x80;
  auto bad_version = frame({});
  bad_version[2] = 3;
  auto bad_length = frame({});
  bad_length[12] = 0xff;
  bad_length[13] = 0xff;
  auto good = frame({3});
  std::vector<uint8_t> input = {8, 0xa5};
  input.insert(input.end(), bad_version.begin(), bad_version.end());
  input.insert(input.end(), bad_length.begin(), bad_length.end());
  input.insert(input.end(), bad_crc.begin(), bad_crc.end());
  input.insert(input.end(), good.begin(), good.end());
  std::vector<FrameParser::Frame> found;
  parser.feed(input.data(), input.size(), [&](FrameParser::Frame f) { found.push_back(std::move(f)); });
  ASSERT_EQ(1u, found.size());
  EXPECT_EQ(good, found.front());
  EXPECT_EQ(1u, parser.stats().crc_errors);
  EXPECT_GE(parser.stats().header_errors, 2u);
  EXPECT_GT(parser.stats().resync_bytes, 0u);
}

TEST(FrameParser, MaxPayloadAndBufferBound) {
  FrameParser parser;
  auto maximal = frame(std::vector<uint8_t>(FrameParser::kMaxPayload, 0x5a));
  std::vector<FrameParser::Frame> found;
  std::vector<uint8_t> noise(FrameParser::kMaxBuffer + 100, 0x7f);
  noise.push_back(0xa5);
  parser.feed(noise.data(), noise.size(), [&](FrameParser::Frame f) { found.push_back(std::move(f)); });
  EXPECT_LE(parser.buffered_bytes(), FrameParser::kMaxBuffer);
  parser.feed(maximal.data() + 1, maximal.size() - 1,
              [&](FrameParser::Frame f) { found.push_back(std::move(f)); });
  ASSERT_EQ(1u, found.size());
  EXPECT_EQ(maximal, found.front());
  EXPECT_EQ(0u, parser.buffered_bytes());
  parser.reset();
  EXPECT_EQ(0u, parser.buffered_bytes());
}
