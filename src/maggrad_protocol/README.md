# maggrad_protocol (ROS Noetic / C++14)

Second-stage decoder for complete v2 ROUND wire frames. `protocol_node` subscribes to
`serial_frame_bridge/RawFrame` on `~raw_frame_topic` (default
`/serial_frame_bridge/raw_frames`), verifies sync, version, type, exact size
(maximum 4080 payload bytes), CRC-16/CCITT-FALSE and every TLV boundary/known
length, then publishes `maggrad_protocol/RoundSample` on `~sample_topic`
(default `/maggrad/sample`). It also publishes protocol statistics to
`~diagnostics_topic` (default `diagnostics`, resolving to `/diagnostics` at
root) once per second. This package has **no serial API**; the bridge
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

## Sensor ID interface

`AkSample` and `TmagSample` each append a `uint8 sensor_id`; no `valid` field
is added. Raw AK `bus`/`mask` and TMAG `ch_mask`/`addr`, counts and status bytes
remain unchanged. The C++ adapter reserves separate
`mapAkSensorId(bus, mask)` and `mapTmagSensorId(ch_mask, addr)` functions in
`src/protocol_node.cpp`. **TODO: the mappings are not confirmed. Both functions
return `0xFF` for every input**, meaning unknown/unmapped; no physical address
mapping is inferred. The historical Python adapter also fills `0xFF`.
Expected-sensor completeness and per-address statistics still use raw byte
pairs, not this placeholder logical ID. The wire protocol is unchanged.
Rebuild all message consumers: this changes the MD5 of `AkSample`,
`TmagSample` and the containing `RoundSample`.

## Namespaces and topic configuration

`protocol.launch` and `joint_debug.launch` inherit the namespace from an
enclosing `<group ns="...">` or the `ROS_NAMESPACE` environment variable.
There is no internal namespace wrapper to override the parent's namespace.
Topic defaults are
relative, so root-level single-device topic names remain unchanged while
instances in distinct namespaces are isolated. Explicit absolute topic
arguments are supported and intentionally bypass the namespace.

| Launch argument | Default (relative to instance namespace) |
| --- | --- |
| `raw_frame_topic` | `serial_frame_bridge/raw_frames` |
| `sample_topic` | `maggrad/sample` |
| `diagnostics_topic` | `diagnostics` |
| `tx_bytes_topic` (joint only) | `serial_frame_bridge/tx_bytes` |
| `statistics_topic` (joint only) | `maggrad_statistics/statistics` |

The protocol node reads private `raw_frame_topic`, `sample_topic`, and
`diagnostics_topic` parameters, resolving their values in the node's namespace
(not its private namespace). Joint launch forwards TX configuration to the
bridge and remaps the statistics output separately from its diagnostics input.
Diagnostic names include the full node name plus `: protocol`,
`: serial bridge`, or `: statistics`, avoiding collisions when merging arrays.
The statistics node also recognizes the legacy `serial_frame_bridge` name.
Use per-instance diagnostics topics for per-device statistics; explicitly
sharing a diagnostics topic makes each statistics node observe all sources.

```bash
# Separate terminals; use the actual stable port for each device:
ROS_NAMESPACE=board_a roslaunch maggrad_protocol joint_debug.launch port:=/dev/ttyACM0
ROS_NAMESPACE=board_b roslaunch maggrad_protocol joint_debug.launch port:=/dev/ttyACM1
# board_a input/output: /board_a/serial_frame_bridge/raw_frames,
# /board_a/maggrad/sample, /board_a/diagnostics,
# /board_a/serial_frame_bridge/tx_bytes, /board_a/maggrad_statistics/statistics
```

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
hardware validation of sustained sensor sampling. `test_protocol_message` checks
both placeholder mappings over all 65,536 byte pairs and verifies conversion
preserves raw addresses/counts/status while filling `sensor_id=0xFF`.
`test_maggrad_statistics` covers both legacy and fully qualified bridge
diagnostic names.

## 联合调试与统计

一次启动串口桥接、协议解码和独立统计节点：

```bash
cd /home/zhang/embedded_array_ws_dual
source devel/setup.bash
roslaunch maggrad_protocol joint_debug.launch
# 其他板卡可覆盖稳定串口路径与波特率：
roslaunch maggrad_protocol joint_debug.launch port:=/dev/ttyACM1 baud_rate:=115200
```

默认串口是本次识别到的
`/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_315E39623233-if00`，
当前对应 `/dev/ttyACM1`。其他 STM32 板卡的序列号不同，需要覆盖 `port`。
请勿同时运行其他读取同一串口的程序。launch 不发送下行控制命令。

`statistics_node` 每秒输出控制台摘要，同时在
`/maggrad_statistics/statistics` 发布 `diagnostic_msgs/DiagnosticArray`，完整字段包括：

- 周期 ROUND 频率、周期/累计帧数、消息 `complete` 比例和最后接收年龄。
- AK/TMAG/ICM 周期/累计记录数，以及 AK `(bus, mask)`、TMAG `(ch_mask, addr)` 的出现次数。
- DRDY/non-DRDY、AK ERR/DOR、TMAG overflow、固件 ERR 分类和 PHASE 缺失/无效统计。
- 最近固件 STATS，按 `device_id` 展示 AK/TMAG/ICM 计数、skipped/errors 和 STATS 年龄。
- 桥接/解码诊断及其年龄，包括 CRC、队列丢弃、序号缺口、重复和乱序等计数。

```bash
rostopic echo /maggrad_statistics/statistics
# 也可单独启动统计节点（上游诊断需汇入 /diagnostics）：
rosrun maggrad_protocol statistics_node _report_interval:=2.0 _stale_timeout:=3.0
```

联合 launch 将桥接的私有诊断和解码诊断汇入 `/diagnostics`，统计输出使用独立话题，避免反馈循环。
没有样本或样本超时为 ERROR；本周期错误、上游诊断异常/缺失或未配置预期传感器为 WARN。
固件累计 errors/skipped 仅在首次非零或增长时触发相应周期告警；历史计数继续保留。
PHASE 缺失单独计数（协议允许省略），PHASE 无效值触发 WARN。
ROUND Hz 是协议轮次速率，**不是传感器有效采样频率**。

默认 expected 列表为空，`complete=true` 不能证明传感器正常。
先按硬件填写预期 ID，再显式开启完整性统计语义：

```bash
roslaunch maggrad_protocol joint_debug.launch \
  expected_sensors_file:=/absolute/path/to/expected_sensors.yaml \
  expected_configured:=true
```

`expected_configured` 只是统计节点的声明，不会自动配置解码器，必须与所加载的 YAML 一致。
可覆盖参数还包括 `raw_frame_topic`、`sample_topic`、`diagnostics_topic`、
`tx_bytes_topic`、`statistics_topic`、
`frame_queue_size`、`publish_batch_size`、`statistics_queue_size`、`report_interval` 和 `stale_timeout`。

### 2026-10-10 实物检查结果

完整的中文实验现象、可能原因和下一步验证方法见
[联合调试实验记录](docs/joint_debug_2026-10-10.md)。

STM32 CDC 串口可读取 v2 ROUND，直接约 2 秒探测校验通过 17,175 帧，CRC 错误为 0。
只有 PHASE 和 STATS，没有 AK/TMAG/ICM 记录；PHASE 三路均为 `0xFFFFFFFF`。
固件 `device_id=3063479653`，三种传感器累计帧计数均为 0。
随后有时限的联合 launch 测试得到约 8.3 kHz ROUND，固件累计 errors 为 5176，
桥接 CRC 错误和 RX 队列丢弃均为 0，但解码器存在序号缺口。
这是当时的观测，不是硬件健康或持续吞吐保证。

结论：串口及帧协议已通，传感器采集尚未正常工作。建议下位机检查传感器供电、
总线初始化、器件地址/片选、读取错误和 PHASE 输入。STATS 未提供错误原因，
仅凭这些计数无法确定具体故障。序号缺口不能区分固件、传输与桥接/ROS 链路损失；
高频短帧下应继续检查固件空轮次发送和 ROS 发布/订阅缓冲，不宜直接认定串口丢包。

新增统计测试与既有测试可运行：

```bash
catkin build serial_frame_bridge maggrad_protocol
# 在各包的构建目录中运行 CTest（兼容本机旧版 CTest）：
cd build/maggrad_protocol && ctest --output-on-failure
# 另一个终端，从工作区根目录：
cd build/serial_frame_bridge && ctest --output-on-failure
```
