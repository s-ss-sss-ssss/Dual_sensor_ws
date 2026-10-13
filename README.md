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

无需硬件的验证：`catkin build`、`catkin run_tests` 和 `catkin_test_results`。伪终端测试不代替 CDC 实物的吞吐与热拔插测试。
