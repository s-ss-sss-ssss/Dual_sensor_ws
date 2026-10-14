std_msgs/Header header
string device_id
bool identity_verified
bool has_device_id
uint32 firmware_device_id
uint32 round_seq
uint32 device_tick_us # Unchanged round-start TIM2 counter from RoundSample.
uint64 device_tick_unwrapped_us # Unchanged host-expanded microsecond time.
bool has_phase
uint32[3] phase_age_us
bool[3] phase_valid
sensor_array_bringup/SensorCorrectionSample[] ak
sensor_array_bringup/SensorCorrectionSample[] tmag
bool has_icm
maggrad_protocol/IcmSample icm
maggrad_protocol/ErrorRecord[] errors
bool has_stats
maggrad_protocol/StatsRecord stats
# These describe the original protocol round, not sensor_correction coverage.
bool complete
uint64 missing_sensor_mask
uint32 skipped_unknown_id
uint32 skipped_missing_sensor_correction
