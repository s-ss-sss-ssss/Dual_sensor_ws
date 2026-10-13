#include <diagnostic_msgs/DiagnosticArray.h>
#include <maggrad_protocol/RoundSample.h>
#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using Clock = std::chrono::steady_clock;
using Status = diagnostic_msgs::DiagnosticStatus;
using Counts = std::map<std::string, uint64_t>;

double seconds(Clock::time_point now, Clock::time_point then) {
  return std::chrono::duration<double>(now - then).count();
}

template <typename T>
void field(Status& status, const std::string& key, const T& value) {
  diagnostic_msgs::KeyValue entry;
  entry.key = key;
  std::ostringstream text;
  text << value;
  entry.value = text.str();
  status.values.push_back(entry);
}

struct Firmware {
  maggrad_protocol::StatsRecord stats;
  Clock::time_point received;
};

struct Statistics {
  Counts total, period;
  std::map<uint32_t, Firmware> firmware;
  Clock::time_point last_received;
  bool received = false;

  Statistics() {
    for (const auto* key : {"rounds", "complete", "ak.records", "tmag.records",
                            "icm.records", "ak.drdy", "ak.not_drdy", "ak.err", "ak.dor",
                            "tmag.drdy", "tmag.not_drdy", "tmag.overflow", "err.records",
                            "phase.missing", "phase.invalid", "stats.records",
                            "firmware.error_updates"}) {
      total[key] = 0;
      period[key] = 0;
    }
  }

  void add(const std::string& key, uint64_t n = 1) {
    total[key] += n;
    period[key] += n;
  }

  void observe(const maggrad_protocol::RoundSample& sample, Clock::time_point now) {
    received = true;
    last_received = now;
    add("rounds");
    if (sample.complete) add("complete");
    for (const auto& a : sample.ak) {
      const auto id = "ak.id." + std::to_string(a.bus) + "." + std::to_string(a.mask);
      add("ak.records"); add(id);
      add(a.drdy ? "ak.drdy" : "ak.not_drdy");
      if (a.err) add("ak.err");
      if (a.dor) add("ak.dor");
    }
    for (const auto& t : sample.tmag) {
      const auto id = "tmag.id." + std::to_string(t.ch_mask) + "." + std::to_string(t.addr);
      add("tmag.records"); add(id);
      add(t.drdy ? "tmag.drdy" : "tmag.not_drdy");
      if (t.overflow) add("tmag.overflow");
    }
    if (sample.has_icm) add("icm.records");  // ICM has no sensor ID in RoundSample.
    if (!sample.has_phase) add("phase.missing");
    else {
      for (std::size_t i = 0; i < sample.phase_valid.size(); ++i) {
        if (!sample.phase_valid[i]) { add("phase.invalid"); add("phase.invalid." + std::to_string(i)); }
      }
    }
    for (const auto& e : sample.errors) {
      add("err.records");
      add("err.source." + std::to_string(e.source) + ".code." + std::to_string(e.code) +
          ".detail." + std::to_string(e.detail));
    }
    if (sample.has_stats) {
      add("stats.records");
      const auto old = firmware.find(sample.stats.device_id);
      if ((old == firmware.end() && (sample.stats.errors || sample.stats.skipped)) ||
          (old != firmware.end() && (sample.stats.errors > old->second.stats.errors ||
                                    sample.stats.skipped > old->second.stats.skipped)))
        add("firmware.error_updates");
      firmware[sample.stats.device_id] = {sample.stats, now};
    }
  }

  uint64_t count(const Counts& counts, const std::string& key) const {
    const auto it = counts.find(key);
    return it == counts.end() ? 0 : it->second;
  }

  Status report(Clock::time_point now, double elapsed, double stale_timeout,
                bool expected_configured) const {
    Status status;
    status.hardware_id = "maggrad_v2";
    const double age = received ? seconds(now, last_received) : -1.0;
    bool errors = false;
    for (const auto* key : {"ak.not_drdy", "ak.err", "ak.dor", "tmag.not_drdy",
                            "tmag.overflow", "err.records", "phase.invalid",
                            "firmware.error_updates"})
      errors = errors || count(period, key) != 0;
    errors = errors || (expected_configured && count(period, "complete") < count(period, "rounds"));
    if (!received || age > stale_timeout) {
      status.level = Status::ERROR;
      status.message = "No recent RoundSample data";
    } else if (errors || !expected_configured) {
      status.level = Status::WARN;
      status.message = errors ? "Errors in report interval" : "Expected sensors not configured";
    } else {
      status.level = Status::OK;
      status.message = "Receiving rounds";
    }
    field(status, "round_hz", elapsed > 0 ? count(period, "rounds") / elapsed : 0.0);
    field(status, "last_round_age_s", age);
    field(status, "report_elapsed_s", elapsed);
    field(status, "expected_configured", expected_configured);
    field(status, "complete_semantics", expected_configured ? "decoder expected-sensor completeness" :
          "message complete only; NOT hardware completeness (configure decoder expected sensors)");
    for (const auto* scope : {"period", "total"}) {
      const auto& counts = std::string(scope) == "period" ? period : total;
      const auto rounds = count(counts, "rounds");
      field(status, std::string(scope) + ".complete_ratio", rounds ?
            double(count(counts, "complete")) / rounds : 0.0);
      for (const auto& entry : total)
        field(status, std::string(scope) + "." + entry.first, count(counts, entry.first));
    }
    field(status, "firmware.devices", firmware.size());
    for (const auto& entry : firmware) {
      const auto prefix = "firmware." + std::to_string(entry.first) + ".";
      const auto& s = entry.second.stats;
      field(status, prefix + "device_id", s.device_id);
      field(status, prefix + "ak_frames", s.ak_frames);
      field(status, prefix + "tmag_frames", s.tmag_frames);
      field(status, prefix + "icm_frames", s.icm_frames);
      field(status, prefix + "skipped", s.skipped);
      field(status, prefix + "errors", s.errors);
      field(status, prefix + "age_s", seconds(now, entry.second.received));
    }
    return status;
  }

  void resetPeriod() { period.clear(); }
};

#ifndef MAGGRAD_STATISTICS_TEST
struct Upstream {
  Status status;
  Clock::time_point received;
};

class StatisticsNode {
 public:
  StatisticsNode() : private_("~"), last_report_(Clock::now()) {
    const auto sample_topic = parameter<std::string>("sample_topic", "/maggrad/sample");
    const auto diagnostics_topic = parameter<std::string>("diagnostics_topic", "/diagnostics");
    const auto interval = parameter<double>("report_interval", 1.0);
    stale_timeout_ = parameter<double>("stale_timeout", 2.0);
    const auto queue_size = parameter<int>("queue_size", 512);
    expected_configured_ = parameter<bool>("expected_configured", false);
    if (!std::isfinite(interval) || interval <= 0 || !std::isfinite(stale_timeout_) ||
        stale_timeout_ <= 0 || queue_size <= 0)
      throw std::invalid_argument("report_interval, stale_timeout and queue_size must be finite positive values");
    if (sample_topic.empty() || diagnostics_topic.empty())
      throw std::invalid_argument("sample_topic and diagnostics_topic must not be empty");
    name_ = ros::this_node::getName() + ": statistics";
    publisher_ = private_.advertise<diagnostic_msgs::DiagnosticArray>("statistics", 1);
    if (nh_.resolveName(diagnostics_topic) == publisher_.getTopic())
      throw std::invalid_argument("diagnostics_topic must not resolve to own ~statistics topic");
    samples_ = nh_.subscribe(sample_topic, queue_size, &StatisticsNode::onSample, this);
    diagnostics_ = nh_.subscribe(diagnostics_topic, queue_size, &StatisticsNode::onDiagnostics, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(interval), &StatisticsNode::onReport, this);
    ROS_INFO_STREAM("MagGrad statistics: samples=" << samples_.getTopic() << " diagnostics="
                    << diagnostics_.getTopic() << " output=" << publisher_.getTopic());
  }

 private:
  template <typename T>
  T parameter(const std::string& key, const T& fallback) {
    T value = fallback;
    if (private_.hasParam(key) && !private_.getParam(key, value))
      throw std::invalid_argument("Invalid parameter type: " + key);
    return value;
  }

  void onSample(const maggrad_protocol::RoundSample::ConstPtr& sample) {
    statistics_.observe(*sample, Clock::now());
  }

  void onDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& report) {
    for (const auto& status : report->status) {
      if (status.name == name_) continue;
      const bool bridge = status.name == "serial_frame_bridge";
      const bool protocol = status.name.size() >= 10 &&
                            status.name.compare(status.name.size() - 10, 10, ": protocol") == 0;
      if (!bridge && !protocol) continue;
      // Forward decoder sequence counters, never infer loss from RoundSample sequence IDs.
      const auto old = upstream_.find(status.name);
      for (const auto& kv : status.values) {
        if (kv.key == "malformed" || kv.key == "crc_errors" || kv.key == "header_errors" ||
            kv.key == "length_errors" || kv.key == "version_errors" || kv.key == "record_errors" ||
            kv.key == "seq_holes" || kv.key == "seq_repeats" || kv.key == "seq_out_of_order" ||
            kv.key == "suspected_restarts" || kv.key == "tick_out_of_order" ||
            kv.key == "duplicate_sensors" || kv.key == "read_errors" || kv.key == "write_errors" ||
            kv.key == "rx_queue_dropped" || kv.key == "tx_dropped") {
          std::string previous = "0";
          if (old != upstream_.end())
            for (const auto& v : old->second.status.values) if (v.key == kv.key) previous = v.value;
          try {
            if (std::stoull(kv.value) > std::stoull(previous)) upstream_errors_ = true;
          } catch (const std::exception&) { upstream_errors_ = true; }
        }
      }
      upstream_[status.name] = {status, Clock::now()};
    }
  }

  void onReport(const ros::WallTimerEvent&) {
    const auto now = Clock::now();
    auto status = statistics_.report(now, seconds(now, last_report_), stale_timeout_, expected_configured_);
    status.name = name_;
    field(status, "upstream.sources", upstream_.size());
    bool have_bridge = false, have_protocol = false;
    for (const auto& entry : upstream_) {
      have_bridge = have_bridge || entry.first == "serial_frame_bridge";
      have_protocol = have_protocol || entry.first != "serial_frame_bridge";
      const auto prefix = "upstream." + entry.first + ".";
      const auto age = seconds(now, entry.second.received);
      field(status, prefix + "age_s", age);
      field(status, prefix + "level", int(entry.second.status.level));
      field(status, prefix + "message", entry.second.status.message);
      for (const auto& value : entry.second.status.values) field(status, prefix + value.key, value.value);
      if (age > stale_timeout_ || entry.second.status.level != Status::OK) upstream_errors_ = true;
    }
    field(status, "upstream.bridge.available", have_bridge);
    field(status, "upstream.protocol.available", have_protocol);
    if ((upstream_errors_ || !have_bridge || !have_protocol) && status.level != Status::ERROR) {
      status.level = Status::WARN;
      status.message += "; upstream errors/stale/unavailable";
    }
    diagnostic_msgs::DiagnosticArray report;
    report.header.stamp = ros::Time::now();
    report.status.push_back(status);
    publisher_.publish(report);
    std::ostringstream summary;
    summary << status.message;
    for (const auto& kv : status.values) {
      if (kv.key == "round_hz" || kv.key == "last_round_age_s" ||
          kv.key == "total.rounds" || kv.key == "period.complete_ratio" ||
          kv.key == "period.ak.records" || kv.key == "period.tmag.records" ||
          kv.key == "period.icm.records" || kv.key == "period.err.records" ||
          kv.key == "period.ak.err" || kv.key == "period.ak.dor" ||
          kv.key == "period.tmag.overflow" || kv.key == "period.phase.invalid" ||
          kv.key == "period.firmware.error_updates" ||
          kv.key.compare(0, 9, "firmware.") == 0 ||
          kv.key.find(".seq_holes") != std::string::npos ||
          kv.key.find(".crc_errors") != std::string::npos ||
          kv.key.find(".rx_queue_dropped") != std::string::npos)
        summary << " " << kv.key << "=" << kv.value;
    }
    if (status.level == Status::ERROR) { ROS_ERROR_STREAM(summary.str()); }
    else if (status.level == Status::WARN) { ROS_WARN_STREAM(summary.str()); }
    else { ROS_INFO_STREAM(summary.str()); }
    statistics_.resetPeriod();
    upstream_errors_ = false;
    last_report_ = now;
  }

  ros::NodeHandle nh_, private_;
  ros::Publisher publisher_;
  ros::Subscriber samples_, diagnostics_;
  ros::WallTimer timer_;
  Statistics statistics_;
  std::map<std::string, Upstream> upstream_;
  Clock::time_point last_report_;
  std::string name_;
  double stale_timeout_ = 2.0;
  bool expected_configured_ = false, upstream_errors_ = false;
};
#endif
}  // namespace

#ifndef MAGGRAD_STATISTICS_TEST
int main(int argc, char** argv) {
  ros::init(argc, argv, "maggrad_statistics");
  try { StatisticsNode node; ros::spin(); }
  catch (const std::exception& e) {
    ROS_FATAL_STREAM("Invalid statistics configuration: " << e.what());
    return 1;
  }
  return 0;
}
#endif
