# AK 与 TMAG 传感器到 sensor_id 的映射

AK 与 TMAG 分别使用独立的 ID1～ID12 编号。上位机应使用传感器类型和 `sensor_id` 共同区分数据，例如 AK ID1 与 TMAG ID1 是不同传感器。

## AK 编号规则

AK 数据中的 `bus` 表示 MCU 的 I²C 总线编号，`mask` 表示该总线上 TCA9548A 的通道选择位掩码。上位机使用 `(bus, mask)` 映射到固定的 `sensor_id`：

- I2C1 的 CH1～CH6 对应 ID1～ID6。
- I2C2 的 CH1～CH6 对应 ID7～ID12。

这里 CHn 指 TCA9548A 的硬件通道 n，`mask = 1 << n`。因此 **CH1 对应 `0x02`，不是 `0x01`**；`0x01` 对应 CH0，不在本映射范围内。

该规则与固件 [`AK09973D_STATIC_CONFIG`](../Core/Inc/sensor_ak09973d.h) 中的 12 路配置一致。

## AK 完整映射表

| 连接位置 | bus（十进制） | mask（十六进制） | mask（十进制） | sensor_id |
| --- | ---: | ---: | ---: | ---: |
| I2C1 CH1 | 1 | `0x02` | 2 | 1 |
| I2C1 CH2 | 1 | `0x04` | 4 | 2 |
| I2C1 CH3 | 1 | `0x08` | 8 | 3 |
| I2C1 CH4 | 1 | `0x10` | 16 | 4 |
| I2C1 CH5 | 1 | `0x20` | 32 | 5 |
| I2C1 CH6 | 1 | `0x40` | 64 | 6 |
| I2C2 CH1 | 2 | `0x02` | 2 | 7 |
| I2C2 CH2 | 2 | `0x04` | 4 | 8 |
| I2C2 CH3 | 2 | `0x08` | 8 | 9 |
| I2C2 CH4 | 2 | `0x10` | 16 | 10 |
| I2C2 CH5 | 2 | `0x20` | 32 | 11 |
| I2C2 CH6 | 2 | `0x40` | 64 | 12 |

## AK 上位机转换

对表内合法输入，设通道号为 `channel`，则：

```text
mask = 1 << channel
sensor_id = (bus - 1) * 6 + channel
```

Python 参考实现：

```python
def ak_sensor_id(bus: int, mask: int) -> int:
    channel_by_mask = {
        0x02: 1,
        0x04: 2,
        0x08: 3,
        0x10: 4,
        0x20: 5,
        0x40: 6,
    }
    if bus not in (1, 2) or mask not in channel_by_mask:
        raise ValueError(f"Invalid AK location: bus={bus}, mask={mask:#04x}")
    return (bus - 1) * 6 + channel_by_mask[mask]
```

`mask` 应为解码后的整数；例如 `ak_sensor_id(2, 0x10)` 返回 `10`。零掩码、多通道组合掩码、CH0、CH7 及其他总线编号均应视为无效输入。

## AK 使用约定

- `sensor_id` 是设备内的固定连接位置编号，不是芯片唯一序列号，也不等同于 STATS 中的 `device_id`。不同设备上的 ID1 可以同时存在。
- 编号不依赖记录在帧内的排列顺序。某个传感器未初始化成功、未就绪或读取失败时，其余传感器仍保持原编号，不重新连续编号。
- 本映射由上位机根据现有 AK 记录中的 `bus` 和 `mask` 计算，无需新增通信字段或修改协议版本。

## TMAG 编号规则

TMAG 全部连接在 I2C3，上位机使用 `(ch_mask, addr)` 映射到固定的 `sensor_id`。`ch_mask = 1 << channel`，其中 `channel` 是 TCA9548A 硬件通道号；`addr` 是该通道下 TMAG 的 7 位 I²C 地址。

按 CH1～CH4 顺序编号，每个通道内部按 GND、VCC、SDA 顺序排列：

- GND：`addr=0x34`，通道内第 1 颗。
- VCC：`addr=0x35`，通道内第 2 颗；对应固件中的 `A2_3V3` 配置。
- SDA：`addr=0x36`，通道内第 3 颗。

该规则与固件 [`sensor_tmag3001.h`](../Core/Inc/sensor_tmag3001.h) 中 CH1～CH4、每通道三颗的配置一致。

## TMAG 完整映射表

| 连接位置（均在 I2C3） | ch_mask（十六进制） | ch_mask（十进制） | addr（7 位，十六进制） | sensor_id |
| --- | ---: | ---: | ---: | ---: |
| CH1 GND | `0x02` | 2 | `0x34` | 1 |
| CH1 VCC | `0x02` | 2 | `0x35` | 2 |
| CH1 SDA | `0x02` | 2 | `0x36` | 3 |
| CH2 GND | `0x04` | 4 | `0x34` | 4 |
| CH2 VCC | `0x04` | 4 | `0x35` | 5 |
| CH2 SDA | `0x04` | 4 | `0x36` | 6 |
| CH3 GND | `0x08` | 8 | `0x34` | 7 |
| CH3 VCC | `0x08` | 8 | `0x35` | 8 |
| CH3 SDA | `0x08` | 8 | `0x36` | 9 |
| CH4 GND | `0x10` | 16 | `0x34` | 10 |
| CH4 VCC | `0x10` | 16 | `0x35` | 11 |
| CH4 SDA | `0x10` | 16 | `0x36` | 12 |

## TMAG 上位机转换

对表内合法输入，设通道号为 `channel`，则：

```text
sensor_id = (channel - 1) * 3 + (addr - 0x34) + 1
```

Python 参考实现：

```python
def tmag_sensor_id(ch_mask: int, addr: int) -> int:
    channel_by_mask = {0x02: 1, 0x04: 2, 0x08: 3, 0x10: 4}
    if ch_mask not in channel_by_mask or addr not in (0x34, 0x35, 0x36):
        raise ValueError(
            f"Invalid TMAG location: ch_mask={ch_mask:#04x}, addr={addr:#04x}"
        )
    return (channel_by_mask[ch_mask] - 1) * 3 + (addr - 0x34) + 1
```

例如 `tmag_sensor_id(0x04, 0x34)` 返回 `4`，`tmag_sensor_id(0x10, 0x36)` 返回 `12`。输入应使用解码后的整数，`addr` 不包含 I²C 读写位，不应左移一位。

零掩码、多通道组合掩码、CH1～CH4 之外的通道及表外地址均应视为无效输入。缺失或读取失败的传感器不引起其余传感器重新编号。本映射不新增通信字段，不修改协议版本。
