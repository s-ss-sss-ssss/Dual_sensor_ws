# AK09973D 一致性标定工具

实验操作全流程、文件保存位置及算法公式见 [一致性标定完整说明](AK09973D_consistency_calibration_guide.md)。

本工具采集 AK 原始话题并输出相对标定结果，不修改在线仿射校正、原始消息或既有标定文件。人工在启动前完成放大器增益调整和设备预热，实验中保持增益不变。软件只做一次默认 3 秒启动数据检查。

## 配置与运行

先复制 `config/consistency_calibration.yaml` 到实验配置目录，填写实际 `firmware_device_id`、各轴 `levels_v` 五档直流电压绝对值、`max_abs_offset_v` 和路径。复制后相对路径以新 YAML 所在目录为基准。核对实际安装与 `array_geometry` 的 R_CORR 及 d_list，设置 `axes_confirmed: true`。d_list 按 ID1～12 顺序、单位 m；Q 是列向量左乘矩阵。sensor_mode 是人工记录的适用配置，当前固件没有提供寄存器回读接口，工具不能自动证明硬件工作模式。

用户已调整放大器增益，确保五档电平在在每个通道的MAX值时，磁力计测的磁场在1500uT左右，磁力计阵列单向最大磁场为3604uT。放大器工作模式为电流反馈，可以近似认为磁场和电流近似线性。

通道映射可配置，X/Y/Z 必须分别对应不同的 CH1/CH2/CH3。五档必须为互异递增正值，单位 V；负极性通过负 offset 产生，不是负 amplitude。`validation_v` 可按轴配置，默认取该轴第 2、3 档中点。电压及限值必须由现场确定；模板的 null 和未确认轴向会阻止启动。

```bash
catkin build sensor_array_bringup
source devel/setup.bash
rosrun sensor_array_bringup consistency_calibration --config /absolute/experiment.yaml --check
rosrun sensor_array_bringup consistency_calibration --config /absolute/experiment.yaml --dry-run
```

这两种模式不导入 ROS、不连接设备、不创建运行文件。`--dry-run` 打印完整状态计划；模拟设备执行状态机由单元测试覆盖。

正常启动阵列上位机，确认 `/<device_id>/maggrad/sample` 正在发布。正式采集直接订阅该原始话题，不使用 `sample_corr` 或现有 affine 参数。设备 ID 必须由 STATS 核对成功；未知地址、重复传感器或地址与 ID 不一致会中止。

如果 FY8300 驱动已在运行，禁止重复启动驱动，显式传入 `start_generator:=false`：

```bash
roslaunch sensor_array_bringup consistency_calibration.launch config:=/absolute/experiment.yaml start_generator:=false
# 或
rosrun sensor_array_bringup consistency_calibration --config /absolute/experiment.yaml
```

标定 launch 的 `start_generator` 默认是 `true`，会同时启动 FY8300 驱动。先核对专用 `config/fy8300_calibration.yaml` 的串口，然后：

```bash
roslaunch sensor_array_bringup consistency_calibration.launch config:=/absolute/experiment.yaml
```

不要重复启动同一串口的驱动。专用配置初始为三个输出关闭、直流波形、零 offset、零 amplitude、输出同步关闭。它不会修改外部 `signal_generator` 包；依赖工作区须已构建并作为当前 catkin underlay。

首次实物联调使用下列入口，仅采集 X 轴最低档正负两个 trial，不拟合矩阵：

```bash
rosrun sensor_array_bringup consistency_calibration --config /absolute/experiment.yaml --smoke-test
```

核对实际正负直流输出和传感器响应后再运行完整实验。工具每步等待两个新鲜状态回读确认，原驱动每条串口命令约有 0.5 秒延迟，因此实际总时间会明显高于纯采集的 18 分钟。设备保存的其他同步选项也应在现场关闭。FY8300 状态回读中 CH2/3 的直流波形编号为 5，工具对此作转换，不将其误认为设置失败。

## 时序、质量与停止

默认 90 组训练、12 组独立验证。每组都是独立的前背景、励磁、后背景，各采集 3 秒并各等待稳定 1 秒。第 2/3 档中点仅用于验证，验证数据不参与参考选择或拟合。

每窗口每颗至少 500 个有效新样本，仅样本数不足会延长，最大 15 秒。DRDY=0、ERR、裁剪、非法时间等不会参与统计；DOR 当前有效新样本保留并报告。按 0.5 秒子窗口检查共同时间覆盖和数据间隙。参考选择首先比较有效样本数，再比较子窗口均值离散程度和噪声，最后按 ID 打破平局；配置 reference_id 可以覆盖选择，指定传感器仍须通过启动检查。

`max_std_ut`、`max_slope_ut_s`、`max_background_change_ut` 是现场质量门限；null 表示报告统计、不据此作门限拒绝。`max_condition` 默认 1000 是工程起点。性能门限 `max_validation_rmse_ut`、`max_validation_pairwise_ut` 为 null 时不宣称性能验收通过。有效性检查与性能评价分别输出；若性能门限不通过，仍保留数值有效的矩阵并明确标记 `performance_pass: false`。

串口样本使用 `device_tick_unwrapped_us + read_done_offset_us`，这是设备读取完成时间，非芯片内部转换时刻。工具用短时主机接收时间的最小到达偏移估计与单调时钟的映射，锁定后保持单位斜率，不将主机回调到达时刻当采样时刻。每窗口默认排除前后各 0.25 秒；传输残差超过阈值、设备时间回退、重复／乱序轮次或设备重启会中止。此方法不能证明绝对同步或推断固定串口延迟；现场需确认延迟小于边界保护量，时钟漂移导致中止时需重新开始采集。硬件模式禁止 `/use_sim_time=true`。

Ctrl+C/SIGTERM、超时、断流和异常退出都会尝试关闭全部通道并保留诊断；关闭时忽略采集取消标志，仍等待关闭状态确认。关闭失败明确报警且不保留成功标定文件。进程被强制杀死、主机断电等无法运行清理逻辑的情形需要现场关闭输出。

## 输出与离线重新拟合

每次创建 `output_root/run_<UTC timestamp>/`：

| 文件 | 内容 |
| --- | --- |
| config.yaml | 解析后的配置、Q、位置、地址、档位和随机种子快照 |
| events.csv | 命令与状态回读确认、阶段切换、失败和重采 |
| raw_stream.csv | ROS 原始全流；保留过渡、无效和迟到记录，不按回调当前状态标记 phase |
| windows.json | 采样时间域的窗口边界、trial、split 和 attempt |
| samples.csv | 按映射后采样时间划分的窗口原始记录，含有效性与 Q 转换值 |
| startup.json | 启动统计与冻结参考 ID |
| trial_summary.csv | 每 trial/attempt/传感器的前后背景、励磁统计、delta 和拒绝原因 |
| calibration.json | Q、T、参考尺度、单位、配置哈希、求解诊断和独立验证结果 |
| validation.csv | 独立验证逐状态前后指标和逐传感器参考残差 |
| short_validation.csv | 应用短窗指标，与长平台平均分开报告 |
| report.md | 结果摘要、条件数、逐状态指标和限制 |

CSV 中向量、矩阵和对象使用 JSON 文本；布尔值为 true/false。原始 `sample_seq` 留空，因为固件只有 round_seq，不能假造逐传感器序号。通信失败只能通过解码器未发布、设备 errors 或断流诊断观察，不能为没有收到的读数伪造 raw；已解码样本的 comm_ok 为 true。温度缺测为 null，不填零。原始全流的 phase 应按 windows.json 中窗口边界离线求得；窗口外是过渡／非采集阶段。边界保护期间到达的正常延迟样本会纳入窗口；晚于保护量才到达的记录保留在全流并触发时钟／传输异常，不应默默归入下一窗口。

矩阵作用为 `T_i @ (Q_i @ (1.1 * raw_i) - background_i)`。本轮不提供在线背景管理，不输出真实零偏，也不覆盖 affine.json。背景是本次实验现场测量值。梯度报告采用无约束空间最小二乘，秩不足时为 null，且仅为名义坐标下的诊断量。

```bash
rosrun sensor_array_bringup consistency_calibration --refit /absolute/run_2026.../
```

离线模式检查源配置哈希和冻结参考，从已完成实验的 samples.csv 原始计数和保存的 Q 重新计算窗口、插值、矩阵和长／短窗验证；通过临时 SQLite 索引逐窗口处理，避免将整场原始数据载入内存；按 trial/attempt/ID 显式对齐，不依赖 CSV 行顺序。配置、启动参考和完整 trial 元数据必须存在。结果写入新的运行目录并记录来源，原实验文件不变。不支持为未完成的硬件实验自动补齐或宣布成功。

## 开发验证

```bash
PYTHONPATH=src/sensor_array_bringup/src /usr/bin/python3 -m unittest discover -s src/sensor_array_bringup/test -v
catkin build sensor_array_bringup
catkin run_tests sensor_array_bringup
catkin_test_results build/sensor_array_bringup
```

合成数据和模拟话题测试不代替真实电平、放大器响应、轴向和线圈均匀性验证。

## note

在这个算法中的梯度仅用于测试评估，没有施加梯度矩阵对称或无迹约束，与实际应用时的梯度求解算法不完全相同
