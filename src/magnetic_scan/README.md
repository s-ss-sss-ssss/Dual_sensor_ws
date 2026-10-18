# magnetic_scan

ROS Noetic 双臂磁场扫描框架。Diana7 携带阵列，`source_arm` 指定的 FR5 保持位置；静止视觉对正后执行 JSON 相对平移点位。关键点通过视觉修正后继续，保存补偿阶段与原始测量。首版不计算梯度，不控制磁源供电。

## 无硬件运行

```bash
cd /home/zhang/embedded_array_ws_dual
catkin build magnetic_scan
source devel/setup.bash
rosrun magnetic_scan scan_mock --config src/magnetic_scan/config/experiment.example.yaml --check
rosrun magnetic_scan scan_mock --config src/magnetic_scan/config/experiment.example.yaml
```

若依赖包已经构建，可使用 `catkin build magnetic_scan --no-deps` 仅构建本包。本次环境原有依赖构建目录存在文件权限问题；本包已通过该方式构建，并在 `/tmp` 独立工作区完成构建及测试。

Mock 使用真实点位解析器、几何计算、状态机和文件记录器，模拟有偏差的运动坐标、视觉反馈及磁场。无需 ROS master。每次输出一个独立实验目录；mock 不生成 rosbag。

## 实验配置

复制 `config/experiment.example.yaml` 到自己的实验配置。相对路径以 YAML 所在目录解析；`points_file` 为点位 JSON，`output_dir` 为结果根目录。配置文件在每次 start 时重新读取并保存快照。独立硬件启动后更改工具、机械臂或外参需重启整套实验节点，不能只重新 start。

示例是可执行的 mock 配置，物理变换、间距范围和磁源工具尚未确认；实机入口会拒绝 `geometry_confirmed: false`、空磁源工具或缺失的未连接机械臂状态。实机前填写：

- `hardware.sensor_tool/source_tool/inactive_tool`：实际安装的 URDF 工具名。
- `hardware.inactive_joint_positions.arm1|arm2`：未连接 FR5 的六个实测关节角，单位 rad；模型中仍保留它及其工具的碰撞几何，不凭空补零。
- `tracking_to_sensor`：视觉刚体 → 阵列几何原点及统一轴系。
- `tracking_to_source`：磁源视觉刚体 → 磁源物理中心，+Z 为磁源向外轴线。
- `planning_link_to_sensor`：MoveIt 所用末端 link → 相同的阵列原点。支架 TCP 不一定是阵列原点。
- `center_distance_range_m`：允许的正轴向中心间距范围。
- 确认上述几何和工具后，设置 `hardware.geometry_confirmed: true`。

所有刚体变换使用 `translation_m` 和 `quaternion_xyzw`。`sensor_tracking_frame` 默认 `diana7_flange_calib`；磁源帧允许 `{source_arm}` 占位符。可通过 `vision_launch_args` 指定已有 TagSLAM launch 的配置参数。视觉可采用其他刚体，但必须发布带真实观测时间戳的动态 TF。

`array.json` 描述芯片相对阵列中心的内部位置和安装轴向，不替代上述工具/视觉外参。采样器保留当前 `SensorCorrectionRound` 的仿射校正向量，不额外应用 `R_CORR`，不隐式转换磁场单位。AK 当前上游输出为 μT，TMAG 单位仍需确认；后续梯度估算层需统一轴系与单位。

## 点位 JSON

```json
{
  "version": 1,
  "frame": "initial_sensor",
  "points": [
    {"id": "origin", "offset_m": [0, 0, 0], "vision_check": true},
    {"id": "x_plus", "offset_m": [0.002, 0, 0], "vision_check": false},
    {"id": "return", "offset_m": [0, 0, 0], "vision_check": true}
  ]
}
```

偏移单位 m，坐标轴为对正后冻结的阵列轴系。每个偏移均相对于同一起点，绝非上一点的增量；工具朝向固定。ID 唯一，位置可以重复。每点可配置正数 `settle_s`、`sample_s`。首末点强制视觉检查；末尾检查就在最后点位，不额外自动回起点。

## 启动与操作

操作者先将 FR5 与 Diana7 放在起始附近，阵列位于磁源物理 +Z 半空间。默认相对朝向为绕磁源 X 转 π，即两端 +Z 相向；绕轴角包含在可配置的 `source_to_sensor_quaternion_xyzw` 中。初始位置保留观测到的轴向距离，修正横向位置和朝向。

先校验配置，不连接硬件：

```bash
rosrun magnetic_scan scan_bringup --config /absolute/experiment.yaml --source-arm arm1 --check
rosrun magnetic_scan scan_bringup --config /absolute/experiment.yaml --source-arm arm2 --dump-launch
```

完整启动，机械臂可选：

```bash
roslaunch magnetic_scan experiment.launch config:=/absolute/experiment.yaml source_arm:=arm1 start_vision:=true start_sensors:=true
```

`source_arm` 支持 arm1/arm2，默认 arm1；`hardware.source_ip` 为空时由上游按所选 FR5 自动选 IP。视觉、传感器默认不重复启动；启用传感器需正确配置 `hardware.sensor_config`。生成的 launch 复用上游描述、硬件和 MoveIt，仅注册两条活动机械臂的执行控制器。关节汇总要求两条活动机械臂都有新鲜状态；未连接臂只提供已填写的静态碰撞状态。

如已有兼容的机器人/MoveIt、视觉和采集进程，可仅启动实验节点：

```bash
roslaunch magnetic_scan scan.launch config:=/absolute/experiment.yaml source_arm:=arm1
```

启动不会自动扫描：

```bash
rosservice call /magnetic_scan/start '{}'
rostopic echo /magnetic_scan/status
rosservice call /magnetic_scan/stop '{}'
```

初始对正与关键点均为“移动 → 关节静止 → 等待 → 新视觉窗口”的迭代过程。视觉修正只重建实验内部的运动指令映射，不发布/覆盖 robot base 或 world TF。关键点修正保留扫描原点及物理目标，后续点使用最新补偿；不重新采集先前点。观察到磁源移动则失败，不以阵列运动补偿磁源移动。

笛卡尔轨迹必须完整规划且执行成功，否则停止，不执行部分轨迹或自动绕行。停止或失败保存已有数据，不自动返回或断开保持中的机械臂。单点采样期间检查活动机械臂静止，使用预期传感器集合验证数据，不以协议 `complete` 单独判断有效性。

## 默认参数与数据

在 `parameters` 中覆盖下列值；未列出的完整默认项见 `configuration.DEFAULTS`。

| 参数 | 默认值 |
|---|---:|
| `position_tolerance_m` / `angle_tolerance_rad` | 0.0005 / 0.008727 |
| `correction_step_m` / `correction_step_rad` | 0.005 / 0.034907 |
| `max_iterations` | 10 |
| `vision_window_s` / `vision_min_observations` | 1 / 10 |
| `vision_timeout_s` / `vision_max_age_s` | 30 / 0.5 |
| `vision_pair_skew_s` | 0.1 |
| `vision_stability_m` / `vision_stability_rad` | 0.0005 / 0.008727 |
| `source_tolerance_m` / `source_tolerance_rad` | 0.0005 / 0.008727 |
| `settle_s` / `sample_s` / `min_rounds` | 1 / 1 / 10 |
| `sample_timeout_s` | 10 |
| `cartesian_step_m` | 0.001 |
| `velocity_scaling` / `acceleration_scaling` | 0.05 / 0.05 |
| `motion_timeout_s` | 60 |
| `stationary_window_s` / `joint_velocity_rad_s` | 0.5 / 0.005 |
| `joint_max_age_s` | 0.5 |

这些是框架初值，不是实测精度保证。视觉窗口要求两刚体同时有独立、递增且采集开始后的时间戳；不能使用给缓存位姿重打时间戳的滤波 TF。共同参考系不应随相机漂移，默认使用固定 `lab_table`。

每次实验目录包含：

- `config.yaml`、`points.json`：实际输入快照。
- `events.jsonl`：状态、视觉窗口、残差、补偿矩阵、实际发送的运动指令和完成状态。
- `results.jsonl`：点位结果、物理目标、运动学位姿、视觉观测（未检查点为 null）、最近检查时间、补偿阶段，以及有效磁场轮次和逐芯片均值/样本标准差。
- `summary.json`：COMPLETE / FAILED / STOPPED 和原因。
- `capture.bag`：实机记录 `/tf`、`/tf_static`、`/joint_states` 及配置的原始帧、协议轮次、校正轮次。异常和采样不足时仍保留此前收到的原始消息。

`sampling.require_identity_verified` 默认 true；上游设备 ID 检查须先正确配置。有效轮要求预期芯片齐全、ID 不重复、DRDY 有效且无 err/dor/overflow、设备错误或非有限磁场值。陈旧/重复轮次拒绝进入统计，拒绝原因和计数随结果保存。当前消息时间戳仍为主机收帧时间，MCU 时间及读完成偏移原样保留，没有宣称视觉与 MCU 已精确同步。

## 验证

```bash
catkin build magnetic_scan
catkin run_tests magnetic_scan
catkin_test_results build/magnetic_scan/test_results
```

测试覆盖几何偏移、带末端外参的全流程、补偿不改变原点、磁源移动、视觉超时、不收敛、运动失败、停止、采样不足、数据质量和双臂启动选择。实机先使用 origin、小偏移、return 三点验收，再加载正式 JSON。
