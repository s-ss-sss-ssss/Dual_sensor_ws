# MagGrad Dual V1 上下位机通信协议 v2

## 配置与传输

默认编译配置为 TMAG 二进制输出；AK 和 ICM 输出默认关闭，可用 `SENSOR_OUTPUT_AK=1`、`SENSOR_OUTPUT_ICM=1` 启用。`SENSOR_OUTPUT_FORMAT=0` 选择原有 CSV 调试模式。

二进制遥测走 USB CDC 字节流，每次主循环生成恰好一个应用协议帧。USB 包和串口 `read()` 边界都不代表帧边界。二进制模式下 `printf` 初始化与诊断日志只走 USART1，不混入 USB 数据。USART1 为 115200 8N1。USB CDC 串口设置的波特率不限制 USB 实际吞吐。

## ROUND 帧

所有多字节数都是小端，有符号数为二进制补码。

| 偏移 | 长度 | 字段 | 内容 |
| ---: | ---: | --- | --- |
| 0 | 2 | sync | `A5 5A` |
| 2 | 1 | version | `2` |
| 3 | 1 | type | `0x10` (ROUND) |
| 4 | 4 | round_seq | 每轮递增的 `u32`，发送或构帧失败仍递增 |
| 8 | 4 | tick_ms | 轮首 `HAL_GetTick()`，`u32` |
| 12 | 2 | payload_len | 所有记录的总字节数，`u16` |
| 14 | N | payload | 连续记录 |
| 14+N | 2 | crc16 | CRC-16/CCITT-FALSE，`u16` |

帧长为 `16 + payload_len`。CRC 多项式 `0x1021`，初值 `0xFFFF`，不反转，末异或 `0`；覆盖字节 `[2:14+payload_len]`。缓冲区容量为 4096 字节，因此当前 payload 最大为 4080 字节。接收端应设置不小于此值的合理上限。协议不发送 TIM2 的绝对计数。

`round_seq` 和 `tick_ms` 均按 `2^32` 回绕。若一轮构帧失败，固件丢弃整个轮帧；USB 发送失败也不会重发。后续帧的序号空洞可用于发现丢轮。STATS 的 `errors` 包括传感器访问、构帧和 USB 发送失败累计次数。

## Payload 记录

每条记录为 `record_type:u8 | record_len:u16le | record_body:record_len bytes`。接收端按 `record_len` 前进；未知类型可直接跳过。仅 AK、TMAG、ICM 记录在原有传感器内容前加 `read_done_offset_us:u32le`，即追加记录时 TIM2 计数减去轮首计数。TIM2 为 1 μs/tick 的 32 位自由运行计数器，减法按 `2^32` 回绕。该偏移代表传感器读取完成时间，非传感器内部转换完成时间。PHASE、ERR、STATS 不带此偏移。

| 类型 | 名称 | `record_len` | `record_body`，按顺序 |
| ---: | --- | ---: | --- |
| `0x01` | AK | 15 | `offset_us:u32, bus:u8, mask:u8, hx:i16, hy:i16, hz:i16, drdy:u8, err:u8, dor:u8` |
| `0x02` | TMAG | 14 | `offset_us:u32, ch_mask:u8, addr:u8, x:i16, y:i16, z:i16, drdy:u8, overflow:u8` |
| `0x03` | ICM | 18 | `offset_us:u32, ax:i16, ay:i16, az:i16, gx:i16, gy:i16, gz:i16, temp:i16` |
| `0x04` | PHASE | 12 | `ch1_age_us:u32, ch2_age_us:u32, ch3_age_us:u32` |
| `0xE0` | ERR | 7 | `source:u8, code:u16, detail:u32` |
| `0xF0` | STATS | 24 | `ak_frames:u32, tmag_frames:u32, icm_frames:u32, skipped:u32, errors:u32, device_id:u32` |

传感器未就绪时该轮省略对应记录并增加 `skipped`。访问失败时可追加 ERR，`source` 的 1/2/3 分别指 AK/TMAG/ICM；`code` 1 指通道选择错误，2 指读取错误。`source=4, code=1` 表示此前 USB 发送失败，`detail` 是累计发送失败次数，在下一次成功发送的轮中可见。ERR 当前不含具体传感器实例。PHASE 每轮一条，记录轮首采样时三路最近上升沿的年龄；从未捕获到边沿的通道为 `0xFFFFFFFF`。STATS 约每秒附加一次，计数从主循环开始累计。

### 设备识别码

`device_id` 在 `main()` 初始化区段计算一次，保存在全局变量 `g_device_id`，随每条 STATS 记录发送。它是 STM32 出厂 96 位 UID 的 CRC-32/ISO-HDLC：输入依次为 `HAL_GetUIDw0()`、`HAL_GetUIDw1()`、`HAL_GetUIDw2()`，每个 32 位字按小端展开，共 12 字节。多项式为 `0x04C11DB7`（反射实现为 `0xEDB88320`），初值 `0xFFFFFFFF`，输入/输出反转，末异或 `0xFFFFFFFF`；上位机可用 `zlib.crc32(struct.pack("<III", uid_w0, uid_w1, uid_w2))` 复算。

同一芯片重新烧录或重启后识别码保持一致；它标识 MCU 所在设备，不区分同板的多个传感器。CRC32 是压缩标识，存在碰撞可能，多设备部署时应检查识别码是否重复。

本次扩展保留 ROUND 协议版本 `2`，STATS 原有前 20 字节不变，末尾偏移 20 增加 4 字节 `device_id`。上位机需支持 `record_len=24。仓库接收脚本能够将新识别码识别为 8 位十六进制，并通过话题发布。

## 接收

在持久字节缓存中搜索同步字，读取 14 字节帧头并限制 `payload_len`，等到 `16 + payload_len` 字节齐全后验证版本、类型和 CRC，再逐条检查 `record_len` 是否留在 payload 范围内。校验失败时从当前同步字后一字节重新搜索；保留末尾半帧等待下次读取。整轮以一帧直接交给上层，不需要以 PHASE 记录聚合。参考实现：`python tools/usb_read.py --port COM5 --format binary --duration 10`。

协议版本 1 的接收程序必须更新。USB CDC 接收回调目前没有下行命令解析。CSV 模式仍输出原有 AK、TMAG、ICM、PHASE 文本行。
