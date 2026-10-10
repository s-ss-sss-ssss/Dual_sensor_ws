# MagGrad v2 高频数据 ROS 接收方案

## 架构

Linux 使用 udev 将 CDC ACM 设备映射为 `/dev/maggrad`。C++14 + Boost.Asio `serial_port::async_read_some()` 在独立 I/O 线程持续读取；每次返回的任意字节片段进入持久流解析器。协议解析和时间展开组成不依赖 ROS 的核心库，完整 ROUND 帧经有界 SPSC 队列交给 ROS Noetic 节点。ROS 2 迁移时复用核心库，只替换 roscpp/catkin 适配层为 rclcpp/ament。调试用 `tools/usb_read.py` 采用同一 v2 规则。

```text
/dev/maggrad → async_read_some → v2 ROUND 解析/CRC → 记录解码
               → 有界整轮队列 → /maggrad/sample 与 /diagnostics
```

I/O 回调只读取、解析和入队，不做 ROS 发布、磁盘写入或阻塞日志。串口参数设为 115200 8N1；CDC 实际速率不由 line coding 限制。热拔插按 0.5、1、2、5 秒退避重连，清空半帧和设备时间映射。

## v2 解析与聚合

帧头固定 14 字节：`A5 5A | version=2 | type=0x10 | round_seq:u32 | tick_ms:u32 | payload_len:u16`。等到 `16 + payload_len` 字节齐全后校验 CRC-16/CCITT-FALSE（覆盖同步字和 CRC 之外的字节）。限制 payload 最大 4080 字节；CRC、版本、类型或长度不合法时从当前同步字后一字节重新查找。串口读取可在任意位置拆分，也可含多轮。解析缓存设置 256 KiB 上限。

CRC 成功后按 `type:u8 | len:u16 | body` 遍历 payload；任何记录越过 payload 边界都丢弃该轮，未知记录类型按长度跳过。AK、TMAG、ICM body 的前 4 字节是相对轮首的读取完成偏移 `read_done_offset_us`，其后才是原有传感器字段。PHASE、ERR、STATS 无偏移。详见[通信协议](host_device_communication_protocol.md)。

每个有效 ROUND 帧已经是一整轮，不再以 PHASE 结尾组装。传感器记录缺失可由未就绪、未启用或访问错误造成。消息中保留实际收到的记录、ERR、STATS、PHASE 和各传感器有效位；根据运行时启用的传感器清单计算 `complete` 与缺失位图。若同一标识重复，保留最后一条并累计诊断计数。相邻 `round_seq` 按模 `2^32` 检查空洞；空洞表示整轮没有到达，不能用邻轮补齐。

推荐 `/maggrad/sample` 字段：`header`、`round_seq`、`device_tick_ms`、`device_tick_unwrapped_ms`、`phase_age_us[3]`、`phase_valid[3]`、AK/TMAG 数组及每条 `read_done_offset_us`、可选 ICM、ERR 数组、可选 STATS、`complete`、`missing_sensor_mask`。上位机保留原始计数，不在驱动层换算磁场单位或坐标系。ROS 1 与 ROS 2 采用相同消息语义。

## 时间、队列与诊断

`tick_ms` 是轮首设备毫秒时间，可按正向回绕展开为 `uint64`；`read_done_offset_us` 使用 TIM2 无符号差，回绕后仍为轮内微秒偏移。主机记录整轮到达时的 steady/system 时间，用滑动窗口估计设备到主机时间映射。初始约 20 轮可使用到达时间并标记时间同步未就绪。该偏移是读取完成时间，不代表传感器内部转换时刻。

SPSC 队列默认容纳 512 轮；满时丢最旧轮并计数。ROS 1 用 1 ms WallTimer 批量发布，publisher queue 深度 5；ROS 2 的高频 topic 用 Sensor Data QoS。诊断每秒发布连接状态、实际轮频、CRC/长度/版本错误、重同步字节、`round_seq` 空洞、记录长度错误、队列丢轮、重连、重复传感器、固件 STATS 中的 `skipped` 与 `errors`。超过 100 ms 无有效轮报警，超过 1 s 或断线报错。

## 验证

纯解析器测试覆盖全传感器、缺失/未就绪、ERR 与 STATS 同轮、最大 payload、TIM2 和序号回绕、未知类型、任意位置拆包、粘包、噪声、CRC 损坏与重同步。台架分别运行默认和全传感器配置，检查 30 分钟实际轮频、CRC 与序号空洞、固件发送失败计数和主机队列丢轮。暂停订阅者与热拔插后确认 I/O 持续或恢复，且不拼接旧半帧。ROS 1 和 ROS 2 用同一录制流比对消息字段。
