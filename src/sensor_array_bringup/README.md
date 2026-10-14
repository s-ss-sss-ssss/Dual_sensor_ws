# sensor_array_bringup

每块串口板一个顶层 YAML；固定 launch 启动 bridge、protocol 和在线校正节点。
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

Noetic XML launch 不能直接用已加载的参数展开 namespace。`array.launch` 使用轻量
`configured_bringup` 入口读取 YAML，将参数传给固定的 `device.launch`，不产生临时 launch。
入口前台管理子 roslaunch，转发 SIGINT/SIGTERM；校正节点属于本包，不属于 protocol。

## 单板配置

```yaml
device_id: array_01                  # 单个安全 ROS 名称，也是根目录下的设备目录名
firmware_device_id: 123456789        # STM32 STATS 上报的 uint32，请替换
check_device_id: true
namespace: /sensor1
port: /dev/serial/by-id/your-board
baud_rate: 115200
sensor_correction_dir: ../../sensor_array_config/config  # 根目录，不是 JSON 文件
raw_scale: 1.0                      # 暂存占位，待确定原始数据单位

# 可选：内联协议预期地址，不另需 expected_sensors YAML
expected_ak: []                     # [bus, mask] 对
expected_tmag: []                   # [ch_mask, addr] 对
expect_icm: false
```

必填：`device_id`、`firmware_device_id`、`namespace`、`port`、`sensor_correction_dir`。
根目录路径相对于 YAML 所在目录，也支持绝对路径、`~` 和 `$(find package)`；
根目录及其 `device_id` 子目录必须存在，JSON 文件路径不接受。
`device_id` 必须匹配 `[A-Za-z_][A-Za-z0-9_]*`，禁止 `/`、`..`、路径穿越、私有名及替换表达式。
`namespace` 仍为独立必填参数，仅用于放置节点，不影响话题前缀。
YAML 不接受 `topics`，即使为空也会报错。固定绝对话题为：

- `/<device_id>/serial_frame_bridge/raw_frames`
- `/<device_id>/serial_frame_bridge/tx_bytes`
- `/<device_id>/maggrad/sample`
- `/<device_id>/diagnostics`
- 校正输出：`/<device_id>/maggrad/sample_corr`（原始 `sample` 加 `_corr`）

例如 `device_id: array_01`、`namespace: /sensor1` 的节点位于 `/sensor1`，
但原始和校正话题分别为 `/array_01/maggrad/sample` 和 `/array_01/maggrad/sample_corr`。
launcher 将显式解析后的绝对话题传给 `device.launch`；直接启动此 launch 时默认话题同样以
`/$(arg device_id)/` 开头。ROS 节点名为 `sensor_correction`，可执行文件为 `sensor_correction_node`，
根目录通过私有参数 `~sensor_correction_dir` 传入。
示例硬件 ID `0` 与引用的单位矩阵标定均是占位内容，实物使用前必须替换。
`five_arrays.example.yaml` 保留旧文件名，但内容也已改为单板模板。

## 多板

每块板准备独立 YAML，配置不同的 device_id、namespace 和串口；可共享同一标定根目录。可以分别启动，或在总 launch 中：

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

默认入口名称根据配置路径生成，亦可显式设置。不要重复启动相同 device_id/namespace/串口。
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

公式严格使用历史模型：`b_corr = D_i @ (raw_scale * b_raw) + e_i`。
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
保留 sensor_id、读取偏移、硬件地址和状态位。单位取决于 raw_scale 与标定文件，暂不声明 SI。

`sensor_id == 0xFF` 直接跳过；缺对应系数也跳过，并分别记录
`skipped_unknown_id` 与 `skipped_missing_sensor_correction`。原始消息不修改。
所有传感器被跳过时仍发布空校正轮次及计数，不填充假的磁场值。
`complete` 与 `missing_sensor_mask` 描述原始协议完整性，并非标定覆盖率。
当前协议逻辑 ID 都是 `0xFF`，因此在映射补充前磁场数组为空。
`/<device_id>/diagnostics` 报告身份状态与跳过情况。

### 迁移

- YAML 和外部 launch 的 `sensor_correction_file` 改为 `sensor_correction_dir`，值改为标定根目录，
  不是 `device_id` 目录或单个 JSON；使用既有 `root/device_id/ak/affine.json` 和
  `root/device_id/tmag/affine.json`，不转换 JSON 内容。
- 删除 YAML `topics` 覆盖项；旧字段会被拒绝。订阅端改用 `/<device_id>/...` 的绝对话题及
  `SensorCorrectionRound` 浮点校正消息。修改 `namespace` 不再改变话题路径。
- 保留必填 `namespace`；为每块设备使用安全且唯一的 `device_id`，并确认对应标定目录存在。

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
