// Exercise the production accumulator without a ROS master or wall-clock sleeps.
#define MAGGRAD_STATISTICS_TEST
#include "../src/statistics_node.cpp"

#include <cassert>
#include <iostream>

namespace {
std::string value(const Status& status, const std::string& key) {
  for (const auto& entry : status.values) if (entry.key == key) return entry.value;
  assert(false && "Missing diagnostic field");
  return {};
}
}

int main() {
  assert(isBridgeDiagnostic("serial_frame_bridge"));
  assert(isBridgeDiagnostic("/board_a/serial_frame_bridge: serial bridge"));
  assert(isBridgeDiagnostic("/board_b/renamed_bridge: serial bridge"));
  assert(!isBridgeDiagnostic("/board_a/maggrad_protocol: protocol"));
  assert(!isBridgeDiagnostic("serial bridge"));
  const auto start = Clock::time_point{};
  const auto later = start + std::chrono::seconds(2);
  Statistics stats;
  assert(stats.report(start, 1, 3, true).level == Status::ERROR);

  maggrad_protocol::RoundSample sample;
  sample.complete = true;
  sample.has_phase = true;
  sample.phase_valid.fill(true);
  maggrad_protocol::AkSample ak;
  ak.bus = 1; ak.mask = 8; ak.drdy = 1;
  sample.ak.push_back(ak);
  maggrad_protocol::TmagSample tmag;
  tmag.ch_mask = 2; tmag.addr = 35; tmag.drdy = 1;
  sample.tmag.push_back(tmag);
  sample.has_icm = true;
  sample.has_stats = true;
  sample.stats.device_id = 42;
  sample.stats.ak_frames = 100;
  stats.observe(sample, start);
  auto report = stats.report(later, 2, 3, true);
  assert(report.level == Status::OK);
  assert(value(report, "round_hz") == "0.5");
  assert(value(report, "total.rounds") == "1");
  assert(value(report, "period.complete_ratio") == "1");
  assert(value(report, "period.ak.id.1.8") == "1");
  assert(value(report, "total.tmag.id.2.35") == "1");
  assert(value(report, "total.icm.records") == "1");
  assert(value(report, "firmware.42.device_id") == "42");
  assert(value(report, "firmware.42.ak_frames") == "100");
  assert(value(report, "firmware.42.age_s") == "2");
  assert(stats.report(later, 2, 3, false).level == Status::WARN);
  assert(value(stats.report(later, 2, 3, false), "complete_semantics").find("NOT hardware") != std::string::npos);
  assert(stats.report(later, 2, 1, true).level == Status::ERROR);

  stats.resetPeriod();
  sample.complete = false;
  sample.ak[0].drdy = 0; sample.ak[0].err = 1; sample.ak[0].dor = 1;
  sample.tmag[0].overflow = 1;
  sample.phase_valid[1] = false;
  maggrad_protocol::ErrorRecord error;
  error.source = 3; error.code = 12; error.detail = 99;
  sample.errors.push_back(error);
  sample.stats.errors = 2;
  stats.observe(sample, later);
  report = stats.report(later, 2, 3, true);
  assert(report.level == Status::WARN);
  assert(value(report, "period.rounds") == "1");
  assert(value(report, "total.rounds") == "2");
  assert(value(report, "total.complete_ratio") == "0.5");
  for (const auto* key : {"ak.not_drdy", "ak.err", "ak.dor", "tmag.overflow",
                          "phase.invalid", "phase.invalid.1", "err.records",
                          "err.source.3.code.12.detail.99", "firmware.error_updates"})
    assert(value(report, std::string("period.") + key) == "1");

  stats.resetPeriod();
  sample.ak.clear(); sample.tmag.clear(); sample.errors.clear();
  sample.complete = true; sample.has_phase = false; sample.has_icm = false;
  sample.stats.device_id = 43; sample.stats.errors = 0;
  stats.observe(sample, later);
  report = stats.report(later, 1, 3, true);
  assert(report.level == Status::OK);
  assert(value(report, "firmware.devices") == "2");
  assert(value(report, "period.ak.id.1.8") == "0");
  assert(value(report, "total.ak.id.1.8") == "2");
  assert(value(report, "period.phase.missing") == "1");
  assert(value(report, "period.phase.invalid") == "0");
  assert(value(report, "period.icm.records") == "0");

  // Unchanged cumulative firmware errors must not warn in every later interval.
  stats.resetPeriod();
  sample.stats.device_id = 42; sample.stats.errors = 2;
  stats.observe(sample, later);
  assert(stats.report(later, 1, 3, true).level == Status::OK);
  stats.resetPeriod();
  report = stats.report(later, 0, 3, true);
  assert(value(report, "round_hz") == "0");
  assert(value(report, "period.complete_ratio") == "0");
  assert(value(report, "total.rounds") == "4");
  std::cout << "Statistics accumulator tests passed\n";
  return 0;
}
