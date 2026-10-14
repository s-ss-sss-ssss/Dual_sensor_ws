std_msgs/Header header
uint32 round_seq
uint32 device_tick_us # Round-start TIM2 counter, 1 us/tick, modulo 2^32.
uint64 device_tick_unwrapped_us # Host-expanded device time in microseconds.
bool has_phase
uint32[3] phase_age_us
bool[3] phase_valid
maggrad_protocol/AkSample[] ak
maggrad_protocol/TmagSample[] tmag
bool has_icm
maggrad_protocol/IcmSample icm
maggrad_protocol/ErrorRecord[] errors
bool has_stats
maggrad_protocol/StatsRecord stats
bool complete
uint64 missing_sensor_mask
