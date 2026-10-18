# 固定一致性矩阵的独立验证

本工具用于在不同温度、静磁场环境下验证**同一份**一致性标定。每次运行24组，不重新拟合、不重选参考、不修改在线参数；输出逐轴残差、阵列一致性和MaPS局部梯度张量X。

## 配置和启动

1. 人工将阵列和线圈固定，设置环境，等待温度与静磁场稳定。记录测温位置、稳定等待时间及静磁场来源；不要在一场验证中主动改变环境。外部温控和测温设备不由本工具控制。
2. 复制 `config/consistency_validation.yaml` 到实验配置目录，填写 `environment_label`、人工实测的 `temperature_c` 和可选 `notes`。默认空标签、null温度会阻止启动，避免误记实验条件。温度可以为负值，但必须是有限数值；它不是芯片内部温度。
3. 设置 `calibration_file`、`array_geometry`、`output_root` 的路径。**相对路径相对复制后的YAML目录解析**；建议复制后填写绝对路径。默认标定来自 `run_20261017T083604.813944Z/calibration.json`。
4. 核对设备ID、硬件工作模式、传感器安装和放大器固定增益。程序使用标定中冻结的Q、T、位置与参考ID，当前几何不一致会报错。`sensor_mode`是人工记录，软件没有硬件寄存器回读接口。

```bash
catkin build sensor_array_bringup
source devel/setup.bash
rosrun sensor_array_bringup consistency_validation --config /absolute/validation.yaml --check
rosrun sensor_array_bringup consistency_validation --config /absolute/validation.yaml --dry-run
```

检查和打印计划不导入ROS、不连接设备、不创建运行目录。默认仍需 `python3-numpy`、`python3-yaml`；生成图表需要 `python3-matplotlib`。

正常启动阵列，确认 `/<device_id>/maggrad/sample` 正在发布。采集使用原始话题，不使用 `sample_corr`，避免重复校正。

已有FY8300驱动时：

```bash
roslaunch sensor_array_bringup consistency_validation.launch config:=/absolute/validation.yaml start_generator:=false
# 或直接执行；该命令不会启动信号发生器驱动
rosrun sensor_array_bringup consistency_validation --config /absolute/validation.yaml
```

需要同时启动驱动时，先核对 `config/fy8300_calibration.yaml` 的串口，再执行：

```bash
roslaunch sensor_array_bringup consistency_validation.launch config:=/absolute/validation.yaml
```

launch的 `start_generator` 默认为true。不要重复启动占用同一串口的驱动。程序确认设备身份，启用直流、零amplitude，并按offset产生正负输出。

## 24组计划与等待时间

| 状态 | X/Z输入 | Y输入 | 数量 |
| --- | --- | --- | --- |
| 单轴低档正负 | ±1 V | ±0.4 V | 6 |
| 单轴中档正负 | ±2.5 V | ±1 V | 6 |
| 单轴高档正负 | ±4.5 V | ±1.8 V | 6 |
| XY、XZ、YZ组合正负 | 参与轴用±2.5 V | 参与轴用±1 V | 6 |

同一组合的两个轴取相同极性。单轴按低/中/高档，各档X/Y/Z依次正负执行，之后XY/XZ/YZ依次正负，所有环境保持相同顺序。

依据原实验读数，这三档单轴约0.55/1.4/2.5 mT，组合约2 mT；这些是选档估计，不是精密场强真值。放大器增益不变，实际参考场矢量和模长写入结果。电压、组合电压与 `max_abs_offset_v` 均可按轴配置。新环境的背景也会占用传感器量程；沿用原采集器的int16边界检查，没有额外的3604μT硬件量程判定。

启动时关闭输出，等待后采集3秒检查数据，保留标定参考ID10。每组为：

```text
全关确认 → 等待3秒 → 前背景采集3秒
设置励磁并确认 → 等待3秒 → 励磁采集3秒
全关确认 → 等待3秒 → 后背景采集3秒
```

`settle_s` 同时控制建立与衰减等待，默认3秒；它是固定等待，不是电流反馈判稳。采集窗前后各排除0.25秒，默认实际用于长窗均值的是2.5秒。每颗至少500个有效样本，仅样本数量不足会延长到最多15秒，其他质量失败触发重采，默认额外重试两次。噪声、斜率与背景变化门限为null时只报告。原驱动每条串口指令和两次新鲜状态回读也占时间，预计15～20分钟，以实际通信为准。

Ctrl+C、SIGTERM、断流、超时和异常均尝试关闭全部通道，关闭确认后才发布 `result.json`。关闭失败写 `failure.json`，不生成成功结果。强制杀进程或主机断电无法执行清理，需要现场关闭输出。

## 指标解释

输入统一转换为 `B_i = Q_i @ (1.1 * raw_i)`，单位μT。固定矩阵应用于四种量：

- `increment`：前后背景按平均采样时间插值，`T_i @ (B_on,i − background_i)`。
- `total_pre`、`total_on`、`total_post`：各平台的 `T_i @ B_i`，不扣背景。

每种量均保存校正前、后指标。每颗传感器的X/Y/Z有符号残差为其矢量减去固定参考矢量，参考自身为零；另存残差模长。阵列分量RMSE、向量RMSE使用各传感器相对阵列均值的偏差，最大差为所有传感器对的最大矢量距离。这些都是相对一致性指标，不是绝对场强误差。

总场的固定矩阵应用不等同于绝对校准：原标定没有估计真实零偏，参考传感器的公共畸变也保留。增量表现良好而总场较差，可能提示偏置/背景依赖，需要结合现场环境排查。

### MaPS梯度X

局部估计复现指定 `maps_estimator.py` 的方法，运行时不依赖另一个工作区，不执行位移/姿态求解：

```text
D：3×12位置矩阵，单位m
w：由null(D)构造，满足 D w=0、1ᵀw=1
Pw = I − w1ᵀ
X = [[x1,x2,x3], [x2,x4,x5], [x3,x5,−x1−x4]]
C = (Dᵀ ⊗ I₃) S
x = pinv(C) vec_F(B Pw)
```

输出 `maps_X_ut_m` 完整3×3张量、`maps_x5_ut_m` 五个独立分量和 `maps_gradient_frobenius_ut_m` 范数，单位μT/m，同时保存局部场估计与设计矩阵条件数。秩不足会在启动前拒绝。

它与原标定报告的无约束梯度不同。**只有近似均匀场下才能将X解释为伪梯度；环境真实空间梯度也会进入X。** 这是比较新环境时必须保留的物理限制。

每个前背景、励磁、后背景平台也分成100ms短窗，分别统计总场，励磁窗另统计背景扣除增量。短窗保存逐窗指标及中位数/95分位/最大值；短窗共用长平台背景，不是独立重复实验。缺少任何传感器的短窗会标记拒绝并报告数量。

## 输出、离线分析与跨环境比较

每次创建 `output_root/run_<UTC>/`，默认在 `data/consistency_validation/`：

| 文件 | 内容 |
| --- | --- |
| config.yaml、calibration_snapshot.json、manifest.json | 冻结配置、原标定内容及完整性哈希 |
| compact_manifest.json、short_measurements.json | 精简分析输入的哈希，以及全部100ms窗中每颗传感器的均值、平均时间、样本数 |
| startup.json、events.csv、windows.json | 启动检查、命令回读、窗口边界及失败诊断 |
| raw_stream.csv、samples.csv | 仅在运行期间保存原始全流与逐样本数据；成功完成后自动清理，失败或中断时保留 |
| trial_summary.csv | 每组/尝试/传感器的均值、噪声、斜率、有效率、DOR等 |
| validation.csv、result.json | 四种量的校正前后指标、实际参考场和前后背景变化；result为成功完成标记 |
| sensor_residuals.csv | 每组每传感器的X/Y/Z残差与模长 |
| maps_gradients.csv | 各状态校正前后完整X、五参数及范数 |
| short_validation.csv | 各平台逐100ms窗口指标与拒绝原因 |
| report.md、metrics.png、axis_residuals.png | 中文摘要、场强相关的一致性/梯度曲线，以及X/Y/Z分量RMSE曲线 |
| storage.json | 原始文件清理状态、释放字节数和清理错误 |

CSV向量、矩阵和对象采用JSON文本。运行期间仍需预留全量记录空间。成功完成采集与输出关闭后，程序先保存平台统计和**全部短窗**的测量摘要，校验输入哈希并实际重算，逐项比较长短窗残差、梯度、汇总和验收结果，再关闭原始文件写入并删除两个大文件。传感器回调不会重新创建已清理的文件。清理失败会记录并提示，不会删除必要的精简记录。历史实验目录不会被自动清理。

```bash
rosrun sensor_array_bringup consistency_validation --reanalyze /absolute/validation_run
rosrun sensor_array_bringup consistency_validation --compare /absolute/run_A /absolute/run_B > comparison.csv
```

离线重算要求完整成功的验证记录，检查配置、标定及精简输入哈希。从 `trial_summary.csv` 的平台均值和时间重新计算背景插值、长窗残差及梯度；从 `short_measurements.json` 中每一个短窗的测量均值和时间重算全部短窗指标，不依赖已导出的残差/梯度结果表，也不做代表窗抽样。平台噪声、协方差、斜率、覆盖率与DOR等完整统计继续保留，固定窗口下的现有分析结果不降低精度。

清理原始数据后不能重新设置窗口边界、短窗长度或逐样本筛选规则，也不能恢复开关过渡波形。对旧版成功验证目录，仍支持读取其全量 `samples.csv`，按原始计数重算；通过SQLite逐窗口处理。两种离线分析均写入新的精简运行目录，原实验不变；不连接硬件、不重新拟合矩阵，也不要求原标定或当前几何文件仍在原路径。

比较结果按标定哈希、几何哈希、励磁方向/电压、数据种类、校正阶段排序，再按温度与环境标签排列，含实际参考场矢量、三轴RMSE和完整X。不同标定或几何明确分组，不混合求平均。比较总场时还应查看实测背景：环境标签本身不证明静场相同。

`performance_limits` 的四项分别对应校正后分量RMSE（每分量均需满足）、向量RMSE、最大差和MaPS梯度范数。所设上限共同应用于增量及三个总场平台；长窗与短窗分别验收，短窗有拒绝则短窗验收失败。默认全部null，`performance_pass.long/short` 为null，表示仅报告。`data_valid` 表示完整采集与分析，不等同于性能验收通过。实际门限需按定位/梯度误差预算填写。

建议先在原标定环境运行一场基准，再分别改变温度和静场，保留同样的励磁电压、增益、位置与等待时间。改变温度和静场两个因素时，记录完整组合，避免把来源不同的变化混为温度影响。
