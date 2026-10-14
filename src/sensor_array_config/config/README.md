# Sensor Array Config

这个目录存放 `sensor_array_config` 读取的整板硬件配置。每个子目录代表一块具体硬件板，例如 `ak09973d`、`qmc6309`、`tmag3001`。

- `<name>/board.json`：整板配置入口，声明要加载哪些子配置文件。
- `<name>/magnetometer.json`：磁传感器型号、量程、单位换算、数量、分组和 frame。
- `<name>/array.json`：磁阵列几何位置 `d_list` 和安装方向修正 `R_CORR`。
- `<name>/affine.json`：每个磁传感器的 affine 标定参数 `D_i`、`e_i`。
- `<name>/imu.json`：IMU 缩放、frame、位置和轴变换。

代码入口保持不变：

```python
from sensor_array_config import get_hardware_config, list_hardware_configs

hw = get_hardware_config("ak09973d")
```

## Deployment 加载接口（离线）

```python
from sensor_array_config import load_deployment, get_hardware_config_from_directory

resolved = load_deployment("deployment.yaml")
# {'schema_version': 1, 'devices': [resolved_device, ...]}
# 外部目录也可以单独加载，返回经过 validate_hardware_config 的 HardwareConfig：
hw = get_hardware_config_from_directory("/absolute/path/to/custom_board")
```

此接口只解析、引用和校验配置，不启动节点、不打开串口、不应用任何校准。
`config/<name>/board.json` + `files` 子配置结构保持不变。
示例 deployment YAML（可放在任意目录）：

```yaml
schema_version: 1
devices:
  - device_id: left
    namespace: /arrays/left
    port: /dev/serial/by-id/usb-board-left
    hardware_config: qmc6309
  - device_id: right
    namespace: /arrays/right
    port: /dev/serial/by-id/usb-board-right
    hardware_config: tmag3001
    # 可选：直接指向外部 board.json 所在目录，不再追加 hardware_config。
    # config_dir: ./boards/custom_right
    baud_rate: 115200
    frame_queue_size: 512
    publish_batch_size: 512
    # expected_sensors_file: ./expected_right.yaml
    core_board_id: "00123"
    firmware_device_id: 42
    topics:
      sample: maggrad/sample          # 相对设备 namespace
      diagnostics: /shared/diagnostics # 绝对话题
```

### 输入约束与输出

- 顶层仅接受 `schema_version`（必须为整数 `1`）和非空 `devices` 列表。
- 每设备必填 `device_id`、`namespace`、`port`、`hardware_config`，均为非空字符串，拒绝首尾空白。
- `namespace` 可输入相对名称，输出为规范化绝对非根命名空间；重复 `/` 与末尾 `/` 会被规范化。
- `port` 必须为绝对串口路径，不接受 `auto` 或相对路径；输出为 realpath。离线加载不要求串口存在。
- `hardware_config` 保留输入的配置名称。无 `config_dir` 时按原 loader 规则查找 `CONFIG_ROOT / hardware_config.lower()`；有 `config_dir` 时直接读取该目录的 `board.json`。目录与 `expected_sensors_file` 的相对路径均相对于顶层 YAML 所在目录，输出为绝对 realpath；支持 `~` 用户目录展开。`expected_sensors_file` 若提供，必须为现有文件，但不解析其内容。
- `baud_rate` 为正整数，默认 `115200`；`frame_queue_size`、`publish_batch_size` 为正整数，默认均为 `512`。不接受 bool、浮点数、数字字符串或 null。
- 可选 `core_board_id` 为非空字符串（纯数字 ID 请加引号），`firmware_device_id` 为 `0..4294967295` 的整数。这些字段仅为元数据，**不代表在线身份核验**。
- `topics` 只接受下表四个 key，值可为相对 namespace 的名称或绝对名称。拒绝 `~` 私有名称、根名称、空白、标点等非法 ROS 名称；规范化重复/末尾 `/`。
- 拒绝顶层、设备及 `topics` 的未知字段，拒绝重复 YAML key 和非字符串 mapping key。严格依赖 PyYAML `SafeLoader`，没有简化解析 fallback，也不接受对象构造标签或 YAML merge key。
- 所有设备的 ID、规范化 namespace、串口 realpath 必须唯一。展开后的所有话题必须无碰撞（包括同设备不同 key、跨设备数据/诊断碰撞）；仅允许多个设备的 `diagnostics` 共用同一话题。
- 实际加载所有硬件子配置并复用 `validate_hardware_config`；缺文件抛出 `FileNotFoundError`，deployment 字段/碰撞错误抛出 `ValueError`，YAML 语法错误抛出 `yaml.YAMLError`。

| `topics` key | 默认绝对话题 |
| --- | --- |
| `raw_frames` | `<namespace>/serial_frame_bridge/raw_frames` |
| `tx_bytes` | `<namespace>/serial_frame_bridge/tx_bytes` |
| `sample` | `<namespace>/maggrad/sample` |
| `diagnostics` | `<namespace>/diagnostics` |

每个 resolved device 包含上述解析后的必填字段、三个默认整数、`config_dir`、完整 `topics`，以及 `configuration_files` 映射：`board`、`magnetometer`、`array`、`affine`、`imu` 五个绝对路径。子配置文件名取自 `board.json.files`，不假定固定文件名。

输出还保留硬件配置中的历史 `firmware_protocol` 和 `calibration_status`，**不会擅自将协议改为 v2**。`calibration_applied: false`、`identity_verified: false` 与 `configuration_note` 明确表示这里只引用配置、未应用校准、未在线核验。可选 ID 和 `expected_sensors_file` 仅在输入提供时出现在结果中。结果均为可序列化的普通 dict/list/标量，不嵌入 `HardwareConfig` 对象。

### 测试

从包目录运行（无需 ROS master、串口或机器人）：

```bash
python3 -m unittest discover -s tests -v
```

运行依赖为 PyYAML 与 NumPy（ROS runtime 依赖已在 `package.xml` 声明）。测试覆盖原硬件接口、默认值、外部目录与 `files` 引用、字段和类型拒绝、严格 YAML、串口 symlink realpath 去重、所有话题碰撞组合、共享诊断及硬件校验失败。

## 目录结构

每个硬件配置目录描述一块具体板子：

```text
config/ak09973d/
  board.json
  magnetometer.json
  array.json
  affine.json
  imu.json
```

运行时使用的名字就是目录名，所以：

```bash
hardware_config:=ak09973d
```

会加载：

```text
config/ak09973d/board.json
```

## `board.json`

整板配置入口文件。

| 字段 | 作用 |
| --- | --- |
| `name` | 硬件配置名，应与目录名一致。 |
| `firmware_protocol` | 该板子期望使用的固件串口协议。 |
| `calibration_status` | 人看的标定状态，例如 `placeholder`、`measured`。 |
| `notes` | 自由备注。 |
| `files` | 子配置文件名，loader 会按这里的引用组装完整硬件配置。 |

## `magnetometer.json`

磁传感器芯片和阵列数量相关信息。

| 字段 | 作用 |
| --- | --- |
| `name`, `manufacturer` | 磁传感器型号和厂商。 |
| `bit_width`, `range_gs`, `adu_to_gs`, `gs_to_tesla` | 从固件 raw/ADU 到 Gs/Tesla 的单位换算参数。 |
| `n_sensors` | 这块板上的磁传感器数量。 |
| `n_groups`, `sensors_per_group` | 逻辑分组信息，用于阵列布局和方向修正。 |
| `frame_id` | 磁阵列数据使用的 ROS frame。 |
| `calibration_status`, `notes` | 磁传感器相关的标定状态和备注。 |

## `array.json`

磁传感器阵列的物理布局和每个传感器的安装方向修正。

| 字段 | 作用 |
| --- | --- |
| `d_list` | 每个磁传感器相对阵列原点的位置，单位是米；每一行是一个 `[x, y, z]`。 |
| `R_CORR` | 方向修正配置。每个条目把一组 `sensor_ids` 映射到同一个 3x3 修正矩阵。矩阵按 column-major 顺序展开。 |

`R_CORR` 在 `serial_processor` 中应用，发生在发布 `stm_uplink_raw` 之前。因此：

```text
stm_uplink_raw = 已完成单位换算 + R_CORR 方向统一，但未做 affine 标定
stm_uplink     = stm_uplink_raw 再经过 affine 标定
```

## `affine.json`

每个磁传感器的 affine 标定参数。

| 字段 | 作用 |
| --- | --- |
| `sensors` | 标定条目列表，每个 sensor 一个条目。 |
| `sensor_id` | 1-based 传感器编号。 |
| `D_i` | 该传感器的 3x3 affine 矩阵。 |
| `e_i` | 该传感器的 3 维 affine 偏置。 |

在线修正公式是：

```text
b_cal = D_i @ b_raw + e_i
```

`calibration_affine_model.py` 默认会写回这个文件：

```text
config/<hardware_config>/affine.json
```

## `imu.json`

IMU 的缩放参数、frame、相对板子的位姿和轴变换。

| 字段 | 作用 |
| --- | --- |
| `name`, `imu_type` | IMU 配置名和芯片型号。 |
| `imu_frame_id` | IMU ROS 消息使用的 frame id。 |
| `publish_scaled_imu` | 是否发布缩放后的 `sensor_msgs/Imu`。 |
| `accel_range_g`, `gyro_range_dps` | 加速度计和陀螺仪量程。 |
| `accel_odr_hz`, `gyro_odr_hz` | 加速度计和陀螺仪输出频率。 |
| `accel_lsb_per_g`, `gyro_lsb_per_dps` | raw 数据到物理单位的缩放系数。 |
| `temp_lsb_per_c`, `temp_offset_c` | 温度换算参数。 |
| `position_m` | IMU 相对板子/阵列原点的位置，单位是米。 |
| `rotation_deg` | 人看的安装角度备注；运行时不使用它做轴变换。 |
| `axis_transform_matrix` | 运行时真正使用的 3x3 轴变换矩阵，会应用到 accel/gyro 向量。 |

`rotation_deg` 和 `axis_transform_matrix` 最好描述同一个物理安装关系，但当前运行时代码只使用 `axis_transform_matrix`。如果两者不一致，以 `axis_transform_matrix` 为准。
