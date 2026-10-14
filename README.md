# embedded_array_ws_dual

ROS Noetic catkin 工作区，独立于 Mi-Gels 源码，使用与 Mi-Gels 相同的底层依赖工作区 `zlab_robots`。新的 ROS 包放在 `src/`，运行数据放在 `data/`，不要将构建产物或实验数据提交到仓库。

## 初始化与构建

需要安装 ROS Noetic、`python3-catkin-tools`，并先构建同级的 `zlab_robots` 工作区：

```bash
cd ~/zlab_robots
catkin build

cd ~/embedded_array_ws_dual
catkin init                         # 首次使用时运行
catkin config --extend ~/zlab_robots/devel
catkin build
source devel/setup.bash
```

`catkin config --extend` 会将依赖固定为 `zlab_robots/devel`，避免当前 shell 中其他 ROS overlay 意外进入构建链。首次克隆后须在本机重新执行初始化和配置；`.catkin_tools/` 是本地配置，不纳入版本控制。`src/CMakeLists.txt` 指向 ROS Noetic 的 catkin 顶层 CMake 文件。

此工作区未复制 Mi-Gels 的包；如需使用 Mi-Gels 的包，应明确选择上游 overlay 或迁移对应包，并避免同时加载同名包。

## MagGrad v2 串口通信

- `src/serial_frame_bridge/`：ROS 无关的 Boost.Asio 串口与 v2 整帧校验库，以及 ROS 话题桥接。发布 `serial_frame_bridge/RawFrame` 到 `/serial_frame_bridge/raw_frames`；订阅 `/serial_frame_bridge/tx_bytes`（`std_msgs/UInt8MultiArray`，直接写入原始字节）。
- `src/maggrad_protocol/`：只订阅原始帧话题，解析传感器记录并将 `maggrad_protocol/RoundSample` 发布到 `/maggrad/sample`。两包都发布 `/diagnostics`。

```bash
source devel/setup.bash
roslaunch serial_frame_bridge serial_frame_bridge.launch port:=/dev/maggrad
# 另开终端并 source devel/setup.bash
roslaunch maggrad_protocol protocol.launch
```

联合调试（串口桥接 + 协议解码 + 统计输出）：

```bash
source devel/setup.bash
roslaunch maggrad_protocol joint_debug.launch
rostopic echo /maggrad_statistics/statistics
```

默认使用本机 STM32 的稳定 `by-id` 路径；其他板卡用 `port:=...` 覆盖。
统计与本次实物检查结果见 [联合调试说明](src/maggrad_protocol/README.md#联合调试与统计)。
ROUND 频率不等同于有效传感器采样频率，默认空 expected 配置也不证明硬件完整。

完整参数和消息字段分别见 [串口桥接包](src/serial_frame_bridge/README.md) 与 [协议抽象包](src/maggrad_protocol/README.md)。使用 `complete` 前应按实物配置 `src/maggrad_protocol/config/expected_sensors.yaml` 中的传感器 ID；留空仅表示没有预期设备。固件尚未定义下行命令协议，`tx_bytes` 只提供原始字节发送能力，不能视为已经实现设备命令。

## 配置驱动的多设备启动

`sensor_array_bringup` 使用每块串口板一个顶层 YAML，将逻辑 ID、固件硬件 ID、串口、namespace 和标定根目录绑定。`sensor_correction_dir` 指向根目录（示例为 `../../sensor_array_config/config`，相对 YAML），不是 JSON；支持绝对路径和 `$(find pkg)`。根目录及 `root/device_id` 必须存在。固定 launch 启动 bridge、protocol 和在线校正节点，不再生成 launch 或配置快照：

```bash
source devel/setup.bash
rosrun sensor_array_bringup bringup --config src/sensor_array_bringup/config/single_array.yaml --check
roslaunch sensor_array_bringup array.launch config:=$(rospack find sensor_array_bringup)/config/single_array.yaml
```

`device_id` 是安全的单个 ROS 名称及目录名，禁止斜杠、`..` 等路径；`namespace` 仍为独立必填参数，只决定节点位置。YAML 不接受 `topics` 覆盖：话题自动使用 `/<device_id>/serial_frame_bridge/raw_frames`、`/<device_id>/serial_frame_bridge/tx_bytes`、`/<device_id>/maggrad/sample` 和 `/<device_id>/diagnostics`，校正输出为 `/<device_id>/maggrad/sample_corr`，与 namespace 无关。launcher 显式传入解析后的话题，直接启动 `device.launch` 的默认值也使用设备 ID 前缀。

标定位于 `root/device_id/ak/affine.json` 和 `root/device_id/tmag/affine.json`，按 `(family, sensor_id)` 匹配现有 `D_i/e_i` 系数，JSON 格式不变。迁移时将 YAML/launch 的 `sensor_correction_file` 改为 `sensor_correction_dir` 根目录，并删除 YAML `topics`；旧字段会被拒绝，订阅端须更新为设备 ID 前缀话题。

示例硬件 ID 和标定系数是占位内容，须按实物替换。多板使用不同 device_id、namespace、串口的独立 YAML，可共享根目录，并在总 launch 中 include 多次 `array.launch`。校正节点发布浮点 `sample_corr`；硬件 ID 检查可配置关闭。AK/TMAG 芯片 `sensor_id` 暂为 `0xFF`，校正节点直接跳过，映射补充前发布空标定数组与跳过计数。原始消息接口不因本次重构而改变。

配置、标定接口和启动流程见 [bringup 文档](src/sensor_array_bringup/README.md)。

无需硬件的验证：`catkin build`、`catkin run_tests` 和 `catkin_test_results`。伪终端测试不代替 CDC 实物的吞吐与热拔插测试。
