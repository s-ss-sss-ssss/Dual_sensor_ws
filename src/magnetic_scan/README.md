# magnetic_scan

ROS Noetic 双臂磁场扫描框架。Diana7 携带阵列，`source_arm` 指定的 FR5 保持位置；独立视觉对正并保存双臂起点，正式实验恢复关节角并核验视觉后执行 JSON 相对平移点位。关键点通过视觉修正后继续，保存补偿阶段与原始测量。首版不计算梯度，不控制磁源供电。

## 当前实验直接启动

`align.launch` 和 `experiment.launch` 默认使用 `config/experiment.arm2.yaml`，默认启动视觉，不需要再传工具参数：

- Diana7：挂载 `calibrated_magnetmeter_array_dual`，阵列中心自动取工具配置的 `tcp_link_name`，本次为 `diana7_magnetmeter_array_dual_tcp_link`。MoveIt 规划末端仍为 `diana7_ee_link`。
- ARM2：`calibrated_em4098`，磁源中心采用上游视觉 TCP 参数（轴向 71.5 mm，绕 Z 转 −90°）。该电磁铁参数仍为上游名义几何，并非本次新增实测标定。
- ARM1：连接硬件并使用真实关节反馈；工具模型沿用上游三臂 launch 的 `em40250` 默认值，实际安装不同时应修改 `hardware.inactive_tool`。
- 三条机械臂 IP 分别为 `.200`、`.201`、`.202`；共用上游机器人描述、布局与 MoveIt。

机器人描述与 MoveIt 使用相同工具名，URDF 固定关节链给出法兰到阵列中心 +219.6 mm。视觉独立传感器支路使用 `sensor_array_calib → sensor_array`，Diana7 视觉工具支路关闭，ARM2 支路输出 `arm2_em_tcp`。两支路读取同一份上游 `visual_tool_transforms.yaml`，默认选择 `default` profile。阵列视觉变换遵循新 profile：+217.1 mm、绕 Z −90°；旧实验的负轴向及旋转定义已移除。

扫描节点直接查询 `world → diana7_magnetmeter_array_dual_tcp_link`、`lab_table → sensor_array` 和 `lab_table → arm2_em_tcp`。视觉偏移只由上游 TCP 桥应用一次，使用未滤波 TCP 的观测时间戳。运动前核验规划末端到阵列 TCP 是 URDF 固定链且 TF 一致，并核验两条视觉静态 TF 与选定 profile 一致；缺失、超时或不一致均失败，不退回手写外参。

```bash
source src/magnetic_scan/scripts/experiment_env.bash
# 启动三条机械臂、MoveIt、ZED/TagSLAM 和起点对正节点
roslaunch magnetic_scan align.launch
# 在另一终端触发对正
rosservice call /magnetic_align/start '{}'
```

对正完成并退出该 launch 后，启动正式实验：

```bash
roslaunch magnetic_scan experiment.launch start_sensors:=true
rosservice call /magnetic_scan/start '{}'
```

正式实验 launch 的 `start_sensors` 默认 false；已有传感器进程时无需重复启动。两个完整 launch 的 `start_vision` 默认 true，若复用已有视觉进程，传 `start_vision:=false`。launch 本身不会触发运动或采集；对正和正式实验均需调用各自的 `start` 服务。`scan.launch` 只启动扫描节点，用于复用已有机器人和视觉进程。

环境脚本加载本工作区、同级 `zed_ws` 和 `tagslam_root`，使用 `local_setup` 排除它们构建时缓存的其他实验工作区。可通过 `MAGNETIC_SCAN_ZED_WS`、`MAGNETIC_SCAN_TAGSLAM_WS` 指定其他位置。只加载本工作区的 `devel/setup.bash` 不一定能找到相机和 TagSLAM 包；如重新 source 了该文件，应再次加载实验环境。

默认点位使用三点示例，目标中心距离 0.1 m、允许轴向范围 0.05–0.2 m；正式点位及距离在 `experiment.arm2.yaml` 中修改。起点保存到 `data/magnetic_scan/initial_pose.arm2.json`。工具或外参更新后须重新对正，不复用旧起点。

## 无硬件运行

```bash
cd /home/zhang/embedded_array_ws_dual
catkin build magnetic_scan
source devel/setup.bash
rosrun magnetic_scan scan_mock --config src/magnetic_scan/config/experiment.example.yaml --check
rosrun magnetic_scan scan_mock --config src/magnetic_scan/config/experiment.example.yaml --align
rosrun magnetic_scan scan_mock --config src/magnetic_scan/config/experiment.example.yaml
```

若依赖包已经构建，可使用 `catkin build magnetic_scan --no-deps` 仅构建本包。

Mock 可通过适配器的 `transforms` 参数注入固定 TF 矩阵；默认从已选工具/profile 创建模拟 TF，无需 ROS master。Mock 使用真实点位解析器、几何计算、状态机和文件记录器，模拟有偏差的运动坐标、视觉反馈及磁场。先运行 `--align` 保存起点，再运行正式采集；Mock 关节角使用可逆的合成模型，仅用于流程验证。无需 ROS master。每次输出一个独立实验目录；mock 不生成 rosbag。

## 实验配置

复制 `config/experiment.example.yaml` 到自己的实验配置。相对路径以 YAML 所在目录解析；`points_file` 为点位 JSON，`output_dir` 为结果根目录，`initial_pose_file` 为起点 JSON 的读写路径。不同磁源臂或实验几何应使用各自的起点文件。配置文件在每次 start 时重新读取并保存快照。独立硬件启动后更改工具、机械臂或外参需重启整套实验节点，不能只重新 start。

示例是可执行的 mock 配置，使用与实机相同的工具/profile 来源，实机几何尚需确认；实机入口会拒绝 `geometry_confirmed: false`、空磁源工具或缺失的未连接机械臂状态。实机前填写：

- `hardware.sensor_tool/source_tool/inactive_tool`：实际安装的 URDF 工具名。
- `hardware.connect_inactive_arm`：true 时连接另一条 FR5 并要求真实关节反馈；false（缺省）时填写 `hardware.inactive_joint_positions.arm1|arm2` 的六个实测关节角（rad），保留其静态碰撞几何，不凭空补零。
- `vision.transforms_file`：视觉工具配置文件；省略时使用上游 `zlab_robots_calibration/config/visual_tool_transforms.yaml`。
- `vision.sensor_profile` / `vision.source_profile`：工具 profile，默认均为 `default`。
- `sensor_tracking_frame` / `source_tracking_frame`：视觉刚体帧，当前为 `sensor_array_calib` / `{source_arm}_flange_calib`。
- `sensor_tcp_frame` / `source_tcp_frame`：未滤波视觉中心帧，当前为 `sensor_array` / `{source_arm}_em_tcp`。
- `center_distance_range_m`：允许的正轴向中心间距范围。
- `alignment.target_distance_m`：独立对正的目标中心距离，必须位于上述范围内。
- `initial_pose_file`：对正成功后自动保存、正式实验读取的起点文件。
- 确认上述几何和工具后，设置 `hardware.geometry_confirmed: true`。

实验 YAML 不再保存 `planning_link_to_sensor`、`tracking_to_sensor`、`tracking_to_source`，出现任一字段即提示迁移。删除旧字段，选择工具和视觉 profile，再重新对正；旧 `vision_launch_args` 也需迁移到上述显式配置。工具几何只在上游工具描述定义，视觉几何只在 profile 定义；本工作区不改写它们。磁源帧支持 `{source_arm}` 占位符。

起点格式升级为 version 2，保存实际机器人中心帧、URDF 固定变换、工具配置和所选视觉 profile 完整内容。正式实验在恢复关节角前完成几何校验并与起点对比；即使文件路径不变，几何或 profile 内容改变也要求重新对正。version 1 起点明确拒绝。Mock 起点与实机起点分开校验，不可混用。

`array.json` 描述芯片相对阵列中心的内部位置和安装轴向，不替代工具 TF 和视觉 profile。采样器保留当前 `SensorCorrectionRound` 的仿射校正向量，不额外应用 `R_CORR`，不隐式转换磁场单位。AK 当前上游输出为 μT，TMAG 单位仍需确认；后续梯度估算层需统一轴系与单位。

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

偏移单位 m，坐标轴为正式实验启动核验通过后冻结的阵列轴系。每个偏移均相对于同一起点，绝非上一点的增量；工具朝向固定。ID 唯一，位置可以重复。每点可配置正数 `settle_s`、`sample_s`。`vision_check` 为布尔值，缺省 `false`；所有点（包括首末点）均按此开关决定是否视觉检查并自动修正。启动视觉核验始终执行，不受点位开关影响。末尾不额外自动回起点。

## 启动与操作

操作者先将 FR5 与 Diana7 放在起始附近，阵列位于磁源物理 +Z 半空间。默认相对朝向为绕磁源 X 转 π，即两端 +Z 相向；绕轴角包含在可配置的 `source_to_sensor_quaternion_xyzw` 中。独立对正保持磁源臂不动，通过视觉闭环调整 Diana7 的位置和朝向，使中心距离达到 `alignment.target_distance_m`。操作者应将双臂置于可规划位置附近，初始观测轴向距离也须在配置范围内。

先校验配置，不连接硬件：

```bash
rosrun magnetic_scan scan_bringup --config /absolute/experiment.yaml --source-arm arm1 --check
rosrun magnetic_scan scan_bringup --config /absolute/experiment.yaml --source-arm arm2 --dump-launch
```

先独立确定起点（无需点位文件、采样配置或传感器节点）：

```bash
roslaunch magnetic_scan align.launch config:=/absolute/experiment.yaml source_arm:=arm1 start_vision:=true
rosservice call /magnetic_align/start '{}'
rostopic echo /magnetic_align/status
# 如需中止：rosservice call /magnetic_align/stop '{}'
```

对正成功后，`initial_pose_file` 保存所选磁源臂、Diana7 七个和 FR5 六个实测关节角（rad）、视觉实测相对位姿、目标距离及工具/外参配置。只有成功才原子替换原起点文件；失败保留已有起点。对正同样会在 `output_dir` 下保存事件与结果摘要，但不启动磁场采样器或生成 rosbag。

结束对正 launch 后，启动正式实验：

```bash
roslaunch magnetic_scan experiment.launch config:=/absolute/experiment.yaml source_arm:=arm1 start_vision:=true start_sensors:=true
```

两个完整 launch 都会启动机械臂和 MoveIt，应先退出对正 launch，再启动正式实验，避免重复硬件节点。若使用已有兼容硬件和视觉进程，可单独启动对正节点：

```bash
rosrun magnetic_scan scan_node __name:=magnetic_align _alignment:=true _config:=/absolute/experiment.yaml _source_arm:=arm1
```

`source_arm` 支持 arm1/arm2，launch 缺省遵循配置文件（当前实机配置为 arm2）；`hardware.source_ip` 为空时由上游按所选 FR5 自动选 IP。完整 launch 默认启动视觉，传感器通过 `start_sensors` 启用。生成的 launch 复用上游描述、硬件和 MoveIt，注册已连接机械臂的执行控制器。关节汇总要求所有已连接臂都有新鲜状态，任何一条失联均停止提供完整状态。ARM1 连接时也参与静止检查，但扫描只向 Diana7 和磁源臂发送运动命令。

如已有兼容的机器人/MoveIt、视觉和采集进程，可仅启动实验节点：

```bash
roslaunch magnetic_scan scan.launch config:=/absolute/experiment.yaml source_arm:=arm1
```

启动 launch 不会自动移动或扫描。正式实验每次调用 `start` 后，先校验机器人固定链、视觉静态 TF 和 version 2 起点几何，再使用 MoveIt 依次恢复磁源臂、Diana7 的保存关节角，再等待静止和新鲜稳定的视觉窗口。视觉实测相对位姿与保存起点在位置和角度阈值内才允许扫描；超差直接失败，不在启动阶段自动修正。起点文件缺失、损坏、所选机械臂或工具/外参/目标距离不匹配时均拒绝执行。

```bash
# 可先校验对正启动配置，不连接硬件
rosrun magnetic_scan scan_bringup --align --config /absolute/experiment.yaml --check
```

触发正式扫描：

```bash
rosservice call /magnetic_scan/start '{}'
rostopic echo /magnetic_scan/status
rosservice call /magnetic_scan/stop '{}'
```

启动核验通过后冻结本次实测阵列位姿作为扫描原点，后续相对点位均使用该原点。初始对正与指定视觉检查点均为“移动 → 关节静止 → 等待 → 新视觉窗口”的迭代过程。视觉修正只重建实验内部的运动指令映射，不发布/覆盖 robot base 或 world TF。关键点修正保留扫描原点及物理目标，后续点使用最新补偿；不重新采集先前点。观察到磁源移动则失败，不以阵列运动补偿磁源移动。

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
| `initial_joint_tolerance_rad` | 0.005 |

这些是框架初值，不是实测精度保证。视觉窗口要求两刚体同时有独立、递增且采集开始后的时间戳；不能使用给缓存位姿重打时间戳的滤波 TF。共同参考系不应随相机漂移，默认使用固定 `lab_table`。

每次实验目录包含：

- `config.yaml`、`points.json`：实际输入快照。
- `initial_pose.json`：本次对正生成或正式实验实际使用的起点快照。
- `geometry.json`：本次实际使用的机器人中心帧、规划末端到中心的固定变换、视觉 profile 内容和运行模式。
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

测试覆盖明确距离对正、起点文件原子保存与校验、双臂关节回位、启动超差禁止采样、点位视觉开关、几何偏移、带末端外参的全流程、补偿不改变原点、磁源移动、视觉超时、不收敛、运动失败、停止、采样不足、数据质量和双臂启动选择。实机先使用 origin、小偏移、return 三点验收，并人为制造启动位姿超差确认采样被阻止，再加载正式 JSON。迁移后必须重新对正生成 version 2 起点；没有有效起点时正式采集不会自动重新对正。测试还覆盖完整 TF 查询、旋转工具的中心目标转换、视觉偏移只应用一次、缺失/陈旧/不一致 TF 的拒绝及旧配置/起点拒绝。
