# maggrad_protocol (ROS Noetic / C++14)

Second-stage decoder for complete v2 ROUND wire frames. `protocol_node` subscribes to
`serial_frame_bridge/RawFrame` on `~raw_frame_topic` (default
`/serial_frame_bridge/raw_frames`), verifies sync, version, type, exact size
(maximum 4080 payload bytes), CRC-16/CCITT-FALSE and every TLV boundary/known
length, then publishes `maggrad_protocol/RoundSample` on `~sample_topic`
(default `/maggrad/sample`). It also publishes protocol statistics to
`/diagnostics` once per second. This package has **no serial API**; the bridge
owns byte-stream framing, resynchronization, connection and queue diagnostics.
Build the bridge first (or build both packages with `catkin build`); the
protocol decoder itself can also be tested independently without ROS.

```bash
# In the configured Noetic workspace:
catkin build maggrad_protocol
source devel/setup.bash
roslaunch maggrad_protocol protocol.launch
# For an independent, ROS-free decoder test:
g++ -std=c++14 -UNDEBUG -I src/maggrad_protocol/include \
  src/maggrad_protocol/src/decoder.cpp src/maggrad_protocol/test/test_decoder.cpp \
  -o /tmp/maggrad_decoder_test && /tmp/maggrad_decoder_test
```

Configure `config/expected_sensors.yaml` with the **actual** hardware IDs:
`expected_ak: [[bus, mask], ...]`, `expected_tmag: [[ch_mask, addr], ...]`,
`expect_icm: true/false`. These are exact transmitted byte pairs (not a count
of devices). The protocol does not fix addresses, so defaults are empty and
false. Configured IDs must be unique within each type and their total cannot
exceed 64. Bits in `missing_sensor_mask` correspond to configured AK entries,
then TMAG entries, then optional ICM. `complete` means every expected ID was
present; with empty expectations it is trivially true. Unexpected IDs are
retained in the message. Duplicate AK/TMAG IDs and ICM records keep the last
value and increment `duplicate_sensors`. PHASE and STATS are optional, ERR can
repeat. Missing PHASE uses invalid ages `0xFFFFFFFF` and false validity bits;
when present, each `0xFFFFFFFF` age is likewise invalid. `header` is copied
from the bridge (host receipt), **not** calculated from device time. Raw
signed counts and unsigned microsecond read-completion offsets are unchanged.

STATS records must contain exactly 24 bytes: the five original `uint32`
counts followed by a little-endian `uint32 device_id` at body offset 20.
The MCU/board identifier is published unchanged as `stats.device_id` in
`/maggrad/sample` when `has_stats` is true; it is not a per-sensor identifier
and is not cached into rounds without STATS. Legacy 20-byte STATS records
are rejected, dropping the entire ROUND. Rebuild message consumers because
adding this field changes the ROS message MD5.

The ROS-independent `include/maggrad_protocol/decoder.hpp` and
`src/decoder.cpp` accept one *complete* frame per call. Unknown TLVs are
skipped; any malformed record drops the entire frame without advancing
sequence/tick baselines or duplicate counts. Sequence deltas are modulo
`2^32`: forward gaps count missing rounds, repeated/old sequence values are
counted without advancing the baseline. Tick expansion accepts forward
modular deltas less than `2^31`; ambiguous/backward ticks retain the last
unwrapped time and count `tick_out_of_order`. Backward sequence and tick
observed together count `suspected_restarts`, but this is a heuristic: a real
device reset or a gap longer than half the tick range needs an explicit
`Decoder::reset_tracking()` (or node restart) before its timestamps can be
trusted. `seq_holes` do not identify whether a drop occurred on-device, in
transport, or in the bridge. This adapter cannot detect a reconnect by itself.

An older Python decoder and its tests remain in the package as historical
reference, but are not installed or used by the C++ node. The C++ synthetic
frame test covers complete and sparse rounds, duplicates, malformed records,
CRC/length/version/type rejection, max payload, unknown TLVs, seq/tick wraps,
and restart suspicion. There is no claim of sustained 500 Hz operation or
hardware validation.
