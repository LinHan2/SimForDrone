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
