# Tracking

双机跟踪控制模块，当前优先实现 T3 真值双机跟踪。

职责：

- 接收带时间戳的目标机与 tracker 共享 ENU 状态；
- 计算期望站位、速度和固定偏航等制导指令；
- 将制导结果交给 `px4ctrl` 的 `CMD_CTRL` 状态，不直接实现 MAVLink；
- 记录站位误差、相对距离、LOS 角速度和控制饱和指标。

边界：

- 本模块不得依赖 Isaac Sim、Pegasus 或相机驱动；
- 飞控命令只能通过 `px4ctrl/` 下发；
- 第一版使用真值或加噪真值，不接视觉；估计器通过稳定接口替换。

## V0：GT 位置跟踪

### 当前默认：软距离带、有限机动与只读 FOV 监控

本阶段以安全与稳定为前提保持可见，不以固定站位误差最小化作为验收。
`run_tracker` 保留 5--10 m 水平软距离带，带内不主动牵引位置；允许瞬时滞后与横向偏差。
`PositionTrackerV0` 仍保留固定 offset 的底层接口，运行时由距离带更新 offset。

| 项目 | 修改前默认 | 当前默认 |
| --- | --- | --- |
| 水平站位 | 5--10 m 软距离带 | 保留，不强制固定相对位置 |
| 完整加速度 | PD + 全量前馈，末端倾角限制 | 水平 1.5、垂向 [-0.8, 0.8] m/s²，jerk 3 m/s³ |
| 目标加速度直接前馈 | 系数 1 | 系数 0，可配置 0.2/0.5/1 |
| 组合倾角预算 | 30° | 10°，不代表 FOV 保证 |
| 指令俯仰驱动高度参考 | 上限 1.5 m | 关闭，避免额外高度耦合 |
| FOV 旁路 | 默认关闭 | 默认开启，只记录，不修改姿态 |
| 验收输出 | PASS 易被误读为跟踪通过 | 明确安全落地与 tracking_assessment 分离 |

`tracking.tracker_control.TrackerLinearControl` 只在 CMD_CTRL 生效。使用 FSM 刷新后的同一份
odom 合成完整加速度，先限制水平/垂向和倾角可行范围，再在前一控制量与可行控制量之间
做向量 slew。此可行集合是凸集，因此插值不破坏加速度或倾角约束。进入跟踪时从零加速度
开始；长间断的 slew 时间最多为 `tracking_max_dt`。退出跟踪后恢复原 LinearControl，
不限制原起飞、悬停、降落控制。限幅只约束指令，不保证实际姿态/加速度绝不超调。

完整控制量通过当次 `p=odom.p, v=odom.v, a=limited` 交给原姿态/推力换算，避免重复 PD。
公共 LinearControl、PX4 内环、感知网络和 EKF 没有为此改动。
参数均在 `config/tracker.yaml`；`--no-tracking-safety` 只关闭新的加速度/jerk 适配。
复现旧参数还需显式设置 `--max-tilt-deg 30 --pitch-fov-compensation 1.5
--acceleration-feedforward-gain 1`；关闭观测用 `--no-visibility-prediction`。
控制预测时域本次不改变；直接前馈权重不等于预测加速度权重。

`run.json` 的 `control_samples` 保存逐周期原始/受限加速度、相对位置/速度、距离带越界量、
期望/实际姿态以及限幅标志。`tracking_assessment` 使用全程、按时间加权的当前几何投影，
报告出画占比、中央区域占比、最长已观测连续出画时间和缺测覆盖率。
默认中央区域为归一化误差不超过 0.7，有效覆盖至少 95%，中央区域占比至少 90%，
有效区间内允许出画占比为 0；这些门槛只用于报告，绝不提高控制增益或驱动 roll/pitch。
超过 0.2 s 的采样间断及无效样本不计为可见；未来预测出画不计入当前出画率。
姿态峰值、变化率 RMS、限幅比例与距离范围单独记录，不把小位置误差当作成功。
`fov_passed` 只表示几何门槛；即使满足，也标为 `requires_stability_review`，需要复核振荡、
hunting 和距离是否持续发散。`passed` 为兼容旧编排保留，仅代表安全落地/上锁。

限制：当前 truth 仍是 PX4 shared ENU 遥测代理，加速度参考来自目标轨迹，不是 Isaac
物理状态真值；像素仍是点投影，不是检测结果，也不建模遮挡和目标尺寸。
本次只完成离线回归，尚未验证 4--6 m/s、2--4 m/s² 实际飞行效果。

### 固定偏移接口（历史 V0）

`tracking.guidance.PositionTrackerV0` 只做固定世界系偏移：

```text
p_des_shared = p_target_shared + relative_offset
p_des_local  = p_observer_local + (p_des_shared - p_observer_shared)
v_des        = v_target
a_des        = a_target
```

第二行只把共享系中的相对位移叠加到 tracker 的 PX4 local ENU 位置，避免把两个不同原点的
绝对坐标混用。输出是与本体控制器同构的 `DesiredState(p, v, a, j, yaw, yaw_rate)`，不产生
姿态、推力或电机命令。

第一轮在线验收只做：静止 target → tracker 保持固定世界系相对位置。通过后依次增加匀速
直线、圆周、加减速和 S 型目标轨迹；V1 才引入目标姿态和 body-frame offset。

当前检查点：target/tracker 双进程 dry-run 与 target 单机 2 m 往返已通过。动态双机跟踪
尚未重新验收；不要把 target 单机的通过结果等同于 tracker 的飞行验收。

## 量测与估计器接口

`tracking.estimation` 提供以下边界：

- `TargetMeasurement`：带单调时间戳的共享 ENU 位置和速度量测；
- `NoisyTargetSensor`：给 target 真值加入可复现的独立高斯噪声；
- `TargetStateEstimator`：估计器协议，后续 EKF/UKF 实现此 `update()` 接口；
- `PassthroughEstimator`：当前占位实现，直接把加噪量测作为状态输出。

在线运行默认选择 `estimator`，即：

```text
target GT → Gaussian noise → PassthroughEstimator → PositionTrackerV0
```

可用 `--state-source truth` 绕过噪声与估计器，作为基准对照。替换真实估计器时不需要修改
`PositionTrackerV0` 或 `px4ctrl`。

计划入口：

```text
tracking/
├── README.md
├── tracking/
│   ├── __init__.py
│   ├── estimation.py      # 噪声量测、估计器协议与状态源切换
│   ├── guidance.py        # TargetState → DesiredState 的纯制导
│   ├── run_tracker.py     # tracker-only 状态订阅与跟踪控制
│   └── target_waypoints.py # target-only 航点与共享状态发布
└── tests/
    └── test_*.py
```

测试命令：

```bash
PYTHONPATH=px4ctrl:tracking PX4-Autopilot/.venv/bin/python -m unittest discover -s tracking/tests -p 'test_*.py'
```

## 运行

场景启动后，先执行 dry-run，只检查两机 heartbeat、落地状态和共享 ENU：

```bash
./scripts/run_target_waypoints.sh   # 终端 1：target 发布状态（dry-run）
./scripts/run_tracker.sh            # 终端 2：tracker 订阅状态（dry-run）
```

tracker 的 dry-run 需要 target 端同时在发布状态。正式飞行见
[scripts/README.md](../scripts/README.md)，其中 target 与 tracker 是两个独立进程：

target 的固定航点默认是保守的 `v<=0.25 m/s, a<=0.25 m/s²`，该参数组只完成过单机
2 m 往返验收；动态双机阶段仍是待办。

- `scripts/run_target_waypoints.sh --interactive --execute`：手动输入航点；
- `scripts/run_tracker.sh --duration 0 --execute`：持续跟踪直到中断或目标状态超时。

早期单进程双机静态测试 `run_static.py` 已删除。它会在同一进程内同时拥有 `14540` 与 `14541`，
与当前“每个端口一个控制进程”的主路径冲突；双机回归统一使用 target/tracker 独立进程。
