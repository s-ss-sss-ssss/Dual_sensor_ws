# sensor_array_bringup

每块串口板一个顶层 YAML；固定 launch 启动 bridge、protocol、在线校正和终端可视化节点。
不生成 launch、不复制配置快照，不再依赖 `sensor_array_config.load_deployment`。

## 使用

```bash
catkin build sensor_array_bringup
source devel/setup.bash
rosrun sensor_array_bringup bringup --config src/sensor_array_bringup/config/single_array.yaml --check
roslaunch sensor_array_bringup array.launch config:=$(rospack find sensor_array_bringup)/config/single_array.yaml
```

也可直接 `rosrun sensor_array_bringup bringup --config /absolute/path/board.yaml`。
`--check` 仅检查顶层配置、标定根目录和 `root/device_id` 目录，不连接串口、不写文件；
各 family 的标定文件及内容在校正节点启动时校验。
旧 `--generate-only`、`--run-dir` 与多设备 `devices:` YAML 已取消。

`array.launch` 和命令行共用 `bringup` 入口。入口读取 YAML 后用 `exec` 替换为固定的
`device.launch`，不产生临时 launch，也不需要单独的信号转发包装进程。校正节点属于本包，
不属于 protocol。

## 单板配置

```yaml
device_id: array_01                  # 单个安全 ROS 名称，也是 ROS namespace 和标定目录名
firmware_device_id: 123456789        # STM32 STATS 上报的 uint32，请替换
check_device_id: true
port: /dev/serial/by-id/your-board
baud_rate: 115200
sensor_correction_dir: ../../sensor_array_config/config  # 根目录，不是 JSON 文件
raw_scale: 1.0                      # 额外缩放；AK 的 1.1 uT/count 在代码中单独应用
field_range: 1000.0                 # 合成场强上限，示例值；单位沿用校正输出，请按实际设置

# 可选：内联协议预期地址，不另需 expected_sensors YAML
expected_ak: []                     # [bus, mask] 对
expected_tmag: []                   # [ch_mask, addr] 对
expect_icm: false
```

必填：`device_id`、`firmware_device_id`、`port`、`sensor_correction_dir`。
根目录路径相对于 YAML 所在目录，也支持绝对路径、`~` 和 `$(find package)`；
根目录及其 `device_id` 子目录必须存在，JSON 文件路径不接受。
`device_id` 必须匹配 `[A-Za-z_][A-Za-z0-9_]*`，禁止 `/`、`..`、路径穿越、私有名及替换表达式。
`device_id` 同时作为逻辑设备名、标定子目录名和 ROS namespace 的末段；例如 `array_01`
对应 `/array_01`。旧配置中的 `namespace` 字段不再接受。
YAML 不接受 `topics`，即使为空也会报错。节点使用相对话题名，由 ROS namespace 解析为：

- `/<device_id>/serial_frame_bridge/raw_frames`
- `/<device_id>/serial_frame_bridge/tx_bytes`
- `/<device_id>/maggrad/sample`
- `/<device_id>/diagnostics`
- 校正输出：`/<device_id>/maggrad/sample_corr`（原始 `sample` 加 `_corr`）

例如 `device_id: array_01` 时所有节点都在 `/array_01` 下；三个节点的诊断汇入
`/array_01/diagnostics`。ROS 节点名为 `sensor_correction`，可执行文件为
`sensor_correction_node`，标定根目录通过私有参数 `~sensor_correction_dir` 传入。
示例硬件 ID `0` 与引用的单位矩阵标定均是占位内容，实物使用前必须替换。
`five_arrays.example.yaml` 保留旧文件名，但内容也已改为单板模板。

## 终端可视化

bringup 自动启动 `sensor_visualization_node`，订阅同 namespace 下的
`maggrad/sample_corr`。每 0.2 秒（5 Hz）读取最新一帧，在 AK 和 TMAG 的 ID 1～12
中选出 `|B| = sqrt(X² + Y² + Z²)` 最大的传感器，输出设备名、轮次、类型、ID、
`|B|` 和 X/Y/Z；相同 ID 的两种类型独立参与比较，等值时取先出现者。
无有效样本时输出提示；NaN/Inf 不参与比较。未收到新帧时重复显示最新轮次，
可通过 round 判断数据是否更新。

`single_array.yaml` 的 `field_range` 必须为有限正数，表示合成场强上限，
与校正输出单位相同，默认 `1000.0` 仅为示例，并非芯片硬件量程。
每次收到消息均检查上述 ID 范围的最大值，严格超过上限时发出 WARN；
持续超量程时告警最多每秒一次，终端数值仍以 5 Hz 更新。

也可以单独启动（已有 bringup 时无需重复启动）：

```bash
rosrun sensor_array_bringup sensor_visualization_node __ns:=/array_01 _field_range:=1000.0
```

## 多板

每块板准备独立 YAML，配置不同的 `device_id` 和串口；可共享同一标定根目录。可以分别启动，或在总 launch 中：

```xml
<launch>
  <include file="$(find sensor_array_bringup)/launch/array.launch">
    <arg name="config" value="/absolute/path/left.yaml"/>
    <arg name="launcher_name" value="left_bringup"/>
  </include>
  <include file="$(find sensor_array_bringup)/launch/array.launch">
    <arg name="config" value="/absolute/path/right.yaml"/>
    <arg name="launcher_name" value="right_bringup"/>
  </include>
</launch>
```

默认入口名称根据配置路径生成，亦可显式设置。不要重复启动相同 `device_id` 或串口。
单板校验不提供跨进程串口独占或跨板话题冲突检测。

## 在线传感器校正

标定按设备和传感器 family 分目录，既有硬件目录保持不变：

```text
<sensor_correction_dir>/
  <device_id>/
    ak/affine.json
    tmag/affine.json
```

例如 `config/array_01/ak/affine.json` 和 `config/array_01/tmag/affine.json`。
模型接口为 `AffineSensorCorrection(root, device_id, raw_scale)`，按 `(family, sensor_id)` 匹配系数，
AK 与 TMAG 可使用相同 `sensor_id`，不会互相覆盖。不改变标定文件或 JSON 格式，
各 family 文件仍兼容历史 `calibration/calibration/affine_model.py` 导出的 JSON：

```json
{"sensors": [{"sensor_id": 1, "D_i": [[1,0,0],[0,1,0],[0,0,1]], "e_i": [0,0,0]}]}
```

AK 每轴先换算为物理场强：`B_uT = RAW × 1.1`，单位为 µT。
协议层已将 RAW 按 `int16_t` 解码，保留负数，不按无符号数解释，也不减 32768。
AK 每轴标称量程为 ±36.04 mT（±36040 µT）；这是单轴量程，
不是合成场强上限，`field_range` 仍按合成场强独立配置。

随后应用仿射校正：

- AK：`b_corr = D_i @ (raw_scale * (b_raw * 1.1)) + e_i`。
- TMAG：暂保留 `b_corr = D_i @ (raw_scale * b_raw) + e_i`。

`raw_scale` 为额外公共缩放，通常保持 `1.0`，不应再填 `1.1`。
AK 标定系数应对应 µT 输入（输出为 µT 时，偏置也应使用 µT）；
如已有系数包含原始计数到物理单位的转换，需要同步调整以免重复缩放。

**TODO（TMAG）：后续沟通确认传感器型号、量程设置、各轴灵敏度及原始读数到物理单位的换算。**
目前不新增 TMAG 换算，其输出单位尚未统一为 µT；AK/TMAG 混合时，
可视化中的跨类型最大值比较和共用量程阈值暂不具备统一物理单位含义。

该历史代码实际是 affine 拟合，不是在线椭球求解；本节点只应用已有系数。
缩放在独立步骤中预留，不做 R_CORR/坐标旋转，不标定 IMU。矩阵和偏置须有限且尺寸正确，
同一 family 内重复 sensor_id、空标定列表、缺文件等在启动时报错。

### 身份检查

默认 `check_device_id: true`。读取 `has_stats` 有效时的 `stats.device_id`：

- 首次匹配成功前不发布校正轮次；验证后，无 STATS 的普通轮次也可发布。
- 任何一次不匹配会锁定阻止状态，直到重启；原始 protocol 话题不受影响。
- `check_device_id: false` 时不阻止发布，仍记录观察到的 ID 和身份状态。
- uint32 硬件 ID 以十进制字符串传入 ROS 参数，避免 XML-RPC 有符号整数限制。

这不是重连身份发现：若替换硬件，需要新的 STATS 才能发现，建议更换设备后重启。

### 浮点接口

`SensorCorrectionRound` 包含原始轮次时间、phase、IMU、错误和统计信息，另含逻辑设备名称、
硬件 ID 是否已观察到及身份验证状态。AK/TMAG 使用 `SensorCorrectionSample` 的 float64 三轴值；
保留 sensor_id、读取偏移、硬件地址和状态位。AK 仿射输入已换算为 µT，
最终输出单位取决于标定系数；TMAG 物理单位换算待确认。

`sensor_id == 0xFF` 直接跳过；缺对应系数也跳过，并分别记录
`skipped_unknown_id` 与 `skipped_missing_sensor_correction`。原始消息不修改。
所有传感器被跳过时仍发布空校正轮次及计数，不填充假的磁场值。
`complete` 与 `missing_sensor_mask` 描述原始协议完整性，并非标定覆盖率。
当前协议逻辑 ID 都是 `0xFF`，因此在映射补充前磁场数组为空。
`/<device_id>/diagnostics` 汇总 bridge、protocol 和校正节点状态。

### 微秒时间接口

原始 `RoundSample` 与校正 `SensorCorrectionRound` 均使用 `uint32 device_tick_us`
和 `uint64 device_tick_unwrapped_us`。校正节点原样转发轮首时间、读取偏移、phase
及主机接收 header，不进行毫秒换算。仅支持上报 TIM2 微秒计数的新固件；launch
和 YAML 无需增加时间单位参数。串口桥接仍只透传原始帧。

正常顺序且时间跟踪有效时，读取完成时间（设备时间域，微秒）为
`device_tick_unwrapped_us + read_done_offset_us`，无 phase 时也适用。
这不是传感器内部转换时刻，也不提供双板同步。原始计数约 71.58 分钟回绕；
可可靠跟踪的真实前向间隔须小于约 35.79 分钟。迟到帧沿用上一展开时间，
设备重启或长时间断流后需要显式重置解码跟踪或重启协议节点。

旧 `device_tick_ms`、`device_tick_unwrapped_ms` 字段已改名，两个轮次消息的 MD5
随之变化，须重新编译并一起重启相关发布者和订阅者；旧 rosbag 和外部消费者需单独适配。

### 迁移

- YAML 和外部 launch 的 `sensor_correction_file` 改为 `sensor_correction_dir`，值改为标定根目录，
  不是 `device_id` 目录或单个 JSON；使用既有 `root/device_id/ak/affine.json` 和
  `root/device_id/tmag/affine.json`，不转换 JSON 内容。
- 删除 YAML `topics` 和 `namespace` 字段；订阅端使用相对话题名解析到 `/<device_id>/...`，
  并确认对应标定目录存在。
- 为每块设备使用安全且唯一的 `device_id`，并继续使用 `SensorCorrectionRound` 浮点校正消息。

## 验证

```bash
PYTHONPATH=src/sensor_array_bringup/src /usr/bin/python3 -m unittest discover -s src/sensor_array_bringup/test -v
catkin build sensor_array_bringup
catkin run_tests sensor_array_bringup
catkin_test_results build/sensor_array_bringup
```

无需硬件的测试不替代实际串口、数据缩放和测量标定效果验证。
`launch/device_00`、`resolved_config.yaml` 和 `solo_dev0.launch` 为旧流程遗留文件，
新入口不使用；保留它们以免覆盖已有实验配置。
