# 项目进度

| 阶段 | 状态 | 证据/下一步 |
|---|---|---|
| P0：Isaac/PX4/WebRTC | 已运行 | Isaac Sim 5.1.0、Pegasus v5.1.0、两套 PX4 SITL 已启动；WebRTC 可查看。 |
| P1：双机观测 | 已通过最低验收 | 两机 `PoseStamped`、tracker RGB、CameraInfo、Depth 均已发布；RGB-D 内部同步已验证。 |
| P1.1：Warehouse 取景质量 | 已降级（非阻塞） | 官方 `full_warehouse.usd` 已加载；画面构图不作为验收项，仅为可选人工检查。 |
| P2.0：tracker 起飞验证 | 已通过 | `TAKEOFF/ARM/LAND` 均接受；ROS 位姿最高 `2.515 m`、最终 `0.055 m`。 |
| P2.1-T0：目标机单机验证 | **已通过** | Offboard 路线：最高 `2.012 m`、站位误差均值 `0.041 m`、落地并确认上锁。证据 `logs/offboard_hold/target-20260918-142828/`（旧验收器，已被 px4ctrl 取代）。 |
| **px4ctrl 控制环** | **已落地并验证** | 唯一命令执行模块 `px4ctrl/`；姿态+推力闭环：最高 `1.974 m`、稳态误差均值 `0.023 m`、落地并上锁。证据 `logs/px4ctrl/hold-target-20260918-214430/`。 |
| P2.1-T2：tracker 站位保持 | **已通过** | 稳态误差均值 `0.014 m`、最大 `0.020 m`（门槛 0.5 m）。证据 `logs/px4ctrl/hold-tracker-20260918-214228/`。 |
| P2.2：多机共享坐标系 | **已实现并实测验证** | `GLOBAL_POSITION_INT` + 共用地理原点换算；与 ROS 真值同时刻对比偏差约 2 cm。 |
| P2.3-T3：真值双机跟踪 | 已飞行，未通过验收 | target 八字轨迹正常完成并安全落地；tracker 可进入跟踪，但后段触发持续倾角饱和并在 target 结束后因状态超时退出。必须先降低动态难度并取得无饱和的双机基准。 |
| P3：视觉位姿/EKF | 未开始 | `vision/` 仅有模块骨架；前置是完成可重复的真值双机动态基准，并录制带时间戳和真值的仿真图像数据。 |

`vision/` 已建立为独立模块，但按阶段约束暂不接入控制；T3/T4 通过后再以视觉相对位姿替换
真值输入，所有飞控命令仍统一经过 `px4ctrl/`。

## 当前检查点

### 已具备

- 场景、两套 PX4 SITL、WebRTC 与 ROS 2 观测接口已可运行；双机 dry-run、target 单机航点、
  自动降落与上锁均已验证。
- `px4ctrl` 是唯一 MAVLink 控制出口；target 与 tracker 各自独占端口，目标状态经本机 UDP 转发。
- 圆形、八字、螺旋轨迹、真值跟踪指标和响应图已实现；完整 `tracking/tests` 离线回归为 **80 项通过**。

### 最近双机动态结果

- 2026-09-26 的八字试验中，target 完成轨迹并安全落地：
  `logs/tracking/target-trajectory-20260926-230758/`。
- tracker 成功起飞并进入跟踪，但后段发生持续倾角饱和，状态机转入 `AUTO_HOVER`；target 落地停止
  状态发布后，tracker 以目标状态超时安全退出：
  `logs/tracking/tracker-v0-20260926-230813/`。
- 因此当前双机动态真值跟踪**没有通过验收**；不能据此开始视觉/EKF 或提高轨迹速度。

### 当前限制

- 固定航点默认使用 `v<=0.25 m/s, a<=0.25 m/s²`；显式命令行参数可覆盖，但不应在双机验收前提高。
- SO(3) body-rate 模式仍关闭。此前观察到约 $150\,\mathrm{ms}$ 延迟下的 yaw 震荡，必须先完成单机
  低带宽阶跃验收，不能用于双机或真机。
- `vision/` 尚未实现数据接入或姿态估计链路，当前只允许将仿真真值用于跟踪基准。

### 下一步

1. 使用比当前八字更保守的速度、加速度和半径，复飞无噪真值双机轨迹。
2. 验收两端 `on_ground=true`、`disarmed=true`，tracker 的 `tilt_saturated_samples=0`，并检查
   `cmd_ctrl_fraction`、动态误差和响应图。
3. 取得无饱和基准后，才用同一轨迹测试 `--state-source estimator`；视觉数据集与网络接入仍排在其后。

## 最近记录

### SO(3) body-rate 外环（2026-09-21）

- 新增 `use_bodyrate_ctrl` 可选模式：由精确群对数 $Log(R^T R_d)$ 和角速度误差生成 FLU
  body-rate，再经 `SET_ATTITUDE_TARGET` 的 body-rate 掩码交给 PX4；默认仍发送四元数姿态。
- 该模式避免把大角度误差以 $sin(theta)$ 近似，输出以 `so3.max_bodyrate=3 rad/s` 限制。
  PX4 继续承担角速度到力矩/电机的内部闭环，因此它不是直接力矩型 SO(3) 控制器。
- 初始实现阶段新增大角度群对数、发送路径和速率限幅回归测试；首次启动场景前，完整离线
  套件为 66 项通过。随后单机仿真结果见下一节。

### SO(3) 单机悬停故障修复（2026-09-21）

- 首次仿真单机 `takeoff-hover-land` 使用 `KAngY=20`、`rate_damping=0.3` 和
  `max_bodyrate=3 rad/s`；出现持续 yaw 震荡，未作为验收通过项。控制日志为
  `logs/px4ctrl/takeoff-hover-land-target-20260921-225420/run.json`。
- 根因：`link.py` 的 `HIGHRES_IMU` 映射只更新加速度、未更新 `imu.w`，故
  $e_\Omega=\Omega-\Omega_d$ 错误地近似为零，SO(3) 的速率阻尼没有收到实测角速度。
  现已按 FRD $\rightarrow$ FLU 映射写入 $(p,-q,-r)$。
- tracker 后续 ULog 显示 PX4 yaw-rate setpoint 与实际 yaw-rate 的最大相关发生在约 150 ms
  延迟处（相关系数约 0.964），所以 FRD/FLU 符号正确；延迟下使用先前的外环带宽会产生
  往复响应。临时 `/tmp/sim-so3.yaml` 已改为 `KAngR/P=2`、`KAngY=0.3`、
  `rate_damping=0.15`、`max_bodyrate=0.5 rad/s`，不改动仓库基线。
- 新增 `px4ctrl step-response` 单机单轴阶跃任务，记录上升时间、超调、整定时间、末段误差、
  倾角和 body-rate 峰值以及安全回退。新增 3 项指标回归，完整离线套件现为 74 项通过；仍须
  先完成 tracker 单机阶跃验收，不能恢复双机跟踪。

### 飞后响应图（2026-09-22）

- 所有 `px4ctrl` 飞行任务在 LAND 和上锁完成后，自动在同次运行目录生成 `response.png`。
  阶跃图含位置、速度、姿态/角速度请求；单机保持和标定图含高度、误差、推力。
- target 航点图含 local ENU 实际/参考三轴轨迹与航点误差；tracker 跟踪图含 shared ENU
  target、tracker、期望站位三轴轨迹，以及跟踪/量测误差。中断任务只要已采样也会在安全收尾后
  出图。
- 绘图使用无界面后端，失败仅跳过 PNG，不改变 LAND、上锁、任务结果或退出码。新增四项图像
  生成回归，完整离线套件为 74 项通过。

### 场景重启与 dry-run 时序修复（2026-09-20）

- `scripts/start_dual_px4_scene.sh` 迁移后首次真实启动通过：Isaac Sim 5.1、WebRTC、内置
  Jazzy rclpy、Pegasus 和两套 PX4 SITL 均启动；两机均打印 `Ready for takeoff!`。启动中的
  GLFW/X display、GPU P2P/IOMMU 与相机 aperture 警告均为已知非致命初始化提示。
- 首次按“先 target、再 tracker”执行 dry-run 暴露一个时序缺陷：target 只发布 5 秒状态，而
  tracker 进程完成 MAVLink 就绪时该窗口已结束。target 自身链路已通过，故问题不在 MAVLink、
  共享 ENU 或 UDP 编解码。
- 修复：`target_waypoints.py` 新增 `--probe-seconds`（默认 15 秒）控制 dry-run 的发布窗口；
  非正值在起飞前拒绝。新增参数解析回归测试，后续按同一推荐顺序重做双进程 dry-run。

### target 单机航点复测（2026-09-20）

- 修复后的 target/tracker 双进程 dry-run 通过：tracker 在 target 的 15 秒发布窗口内收到新鲜
  共享 ENU 状态，不再受启动时序影响。
- 首次单机 2 m 航段使用 `v<=0.8 m/s, a<=1.0 m/s²` 时，控制器在起步后持续倾角饱和
  `0.523 s`，看门狗立即切换当前点悬停；人工中断后 `LAND` 被接受，最终
  `on_ground=True, disarmed=True`，没有姿态翻转。
- 根因是轨迹限值只约束加速度前馈，未给 `Kp·位置误差 + Kv·速度误差` 留出 25 度倾角预算。
  默认限值已收紧为 `v<=0.25 m/s, a<=0.25 m/s²`，保留显式命令行覆盖能力。
- 保守限值复飞通过：两段 2 m 往返均收敛，最终误差约 `0.013 m` 与 `0.016 m`，无倾角饱和；
  正常 `LAND` 并确认 `on_ground=True, disarmed=True`。证据
  `logs/tracking/target-waypoints-20260920-154346/`。
- 任务末尾原本会停止刷新 `CMD_CTRL` 指令并触发一次无害的命令超时告警；现已在最终保持前
  显式切换 `AUTO_HOVER`。完整离线回归为 63 项通过。

### 删除已被取代的 shell 包装（2026-09-19）

按“能力已被主路径完全覆盖 + 近期无使用痕迹”筛掉 3 个脚本，项目 shell 由 14 个减到 11 个：

| 已删除 | 取代它的路径 | 删除依据 |
|---|---|---|
| `scripts/start_ros2_px4.sh` | Isaac + Pegasus + PX4 SITL | 独立的 Gazebo/ROS2 栈，`logs/ros2_px4/` 最后活动停在 9月7 |
| `scripts/run_tracking.sh` | `scripts/run_target_waypoints.sh` + `scripts/run_tracker.sh` | 单进程旧形态（同时占用 14540/14541），文档自述已被取代 |
| `scripts/record_tracker_takeoff_pose.sh` | `scripts/run_px4ctrl.sh` 的 `hold`/`measure-hover` | P2.0 起降验收已完成，核验统一由 px4ctrl 负责 |

当时遵循“只删 shell 包装，不删底层能力”的原则；该结论已被后续主流程重构取代。

### 清理旧双机测试入口（2026-09-26）

- 删除 `tracking/tracking/run_static.py`：它在单个进程同时拥有 `14540`/`14541`，与当前双进程
  端口所有权模型冲突，且已没有调用点。
- 合并 `check_dual_uav_observation.sh`、`capture_rgbd_sample.sh` 与
  `check_observer_rgb_scene.sh` 为 `scripts/check_observation.sh`；默认检查 ROS 2 话题，
  `--rgbd` 才额外采样并检查前视图像。
- 完整 `tracking` 离线回归为 80 项通过。

恢复任一被删脚本（内容都在 git 中，已验证可读出）：

```bash
git show HEAD:scripts/run_tracking.sh > scripts/run_tracking.sh
git show HEAD:start_ros2_px4.sh > scripts/start_ros2_px4.sh
git show :scripts/record_tracker_takeoff_pose.sh > scripts/record_tracker_takeoff_pose.sh
```

同步更新的文档：`tracking/README.md`、`READMELIST/invocation_map.md`、
`READMELIST/runbook.md`；`READMELIST/ros2_px4_startup_test.md` 加了顶部归档说明
（保留 uXRCE-DDS / `/fmu` 配置知识，并注明恢复命令）。

### shell 启动脚本集中到 `scripts/`（2026-09-19）

此前入口散在仓库根、`scripts/`、`test/`、`utils/` 四处，找一条命令要在四个目录之间翻。
现已全部集中：

| 位置 | 内容 |
|---|---|
| `scripts/*.sh` | 全部启动入口（场景、px4ctrl、target/tracker、只读验收、结果汇总、ROS 2/PX4 启动） |
| `scripts/env/*.sh` | 三个**只能 `source`** 的环境 profile（保留“不可直接执行”的语义） |
| `scripts/01_dual_px4_scene.py` | 场景入口（未动） |
| `utils/*.py` | 只读验收的 Python 实现（脚本已移走，模块路径不变） |

- 移除了空目录 `test/`、`env/`；`test/README.md` → `scripts/README.md`，与脚本同目录。
- 所有脚本仍用 `BASH_SOURCE/..` 推导项目根：`scripts/` 与原 `test/`、`utils/`、`env/` 同为
  一级目录，因此该推导搬迁后依然成立；只有原在仓库根的 `start_ros2_px4.sh` 需要改成
  `.../..`，已修正。
- 同步更新了 9 处 `source` 路径，以及 `README.md`、`scripts/README.md`、`tracking/README.md`、
  `READMELIST/{runbook,invocation_map,progress,pegasus_aerostack2_architecture,
  ros2_px4_startup_test}.md` 和 `run_tracker.py` 的提示语。
- 验证：14 个脚本 `bash -n` 全通过、执行位保持；无参 `start_ros2_px4.sh` 打印 usage，四个
  Python 入口 `--help` 正常（说明 ROOT 推导、profile source 与 PYTHONPATH 均正确）；
  `show_latest_tracking_result.sh` 实测可读最新 `run.json`；61 项离线单测通过。
- **尚未复测**：`start_dual_px4_scene.sh` 需要真正重启 Isaac/PX4 才能验证，下次启动场景时确认。

### 代码评审与视角修复（2026-09-19）

评审按“控制律 → 通信层 → 跟踪器 → 估计器”的顺序展开，已完成的修复：

- **视角不再被程序锁死**：新增 `free`（`0`）手动模式并改为**默认**。此前默认是
  `follow_tracker`，它每帧写相机位姿，会立即覆盖鼠标拖动，表现为“无法手动调整视角”。
  现在只有显式选择 2/3 跟随时才驱动相机；从机载相机返回时会把 viewport 挂回
  `/OmniverseKit_Persp`；机载相机缺失时退回 `free` 而不是跟随。
- **控制律与接口修复**：见下一条。评审列出的 5 项已全部修完，并额外修掉一处同源的
  近似误差；单测从 25 项增至 44 项，全部通过。

评审发现（详见下条“控制律评审结论”）：

1. **倾角公式除以 `g` 而非垂向指令 `des_a_z`** —— 与推力计算不自洽，高度环活跃时
   倾角误差可达 ±66%，是候选的振荡机制。
2. **`msg_timeout.cmd` 从未被使用** —— `CMD_CTRL` 不检查指令新鲜度，制导源中断时
   会永久使用陈旧设定点（与 FSM 自己的注释相矛盾）。
3. `estimate_thrust_model`（在线 RLS）**从未被调用**，但 `sim.yaml` 写着
   `accurate_thrust_model: true`：配置与行为不一致。
4. `MIN_NORMALIZED_COLLECTIVE_THRUST`、`DesiredState.j`/`yaw_rate` 为未使用的死代码。
5. 收尾上锁只用 `force=False`，被 PX4 拒绝（`Disarming denied: not landed`）时
   只记录不重试，实测出现“靠 PX4 failsafe 落地”的情况。

### 控制律与接口修复（2026-09-19，已完成）

上一条评审的 5 项已全部修完，并额外修掉一处同源的近似误差。所有改动都有中文注释说明
“为什么”，其中控制律部分由单测钉住（共 44 项，全部通过）。

| # | 问题 | 修法 | 验证 |
|---|---|---|---|
| C1 | 倾角反解除以常数 `g` | 改为精确反解（见下） | 几何一致性单测：以 2 个偏航角 × 3 组指令验证机体 z 轴与总期望加速度方向**逐分量相等**（9 位小数） |
| C1b | 线性小角度形式 `θ ≈ a_h/a_z` | 改为精确形式 `sinφ = a_y^b/\|a\|`、`θ = atan2(a_x^b, a_z)` | 同上；线性形式在 25° 时会多给约 6% 倾角 |
| C3 | 缺 `1/cosθ` 垂向推力补偿 | 推力除以 `cosφ·cosθ`（该式对任意倾角精确） | 25° 时补掉 9.4% 垂向缺口 |
| C2 | `msg_timeout.cmd` 未使用 | `set_command()` 内统一盖到达时刻；`CMD_CTRL` 下指令超时 → `AUTO_HOVER` 悬停 | 单测覆盖新鲜/超时/从未收到指令三种情形 |
| C4 | 配置与行为不一致、死代码 | 核实上游 `accurate_thrust_model` **本来就只读不用**（保留并注明）；删除未使用的 `MIN_NORMALIZED_COLLECTIVE_THRUST`；新增 `thrust_model.online_estimate` 作为 RLS 的显式开关（默认关）；`disarm_timeout` 此前被解析但从未传给 `finish`，现已接通 | 配置文件两侧同步；`load_params` 实测加载正确 |
| L3 | 收尾上锁被拒只记录不重试 | 两级上锁：常规 → 有界等待 → 强制（21196），结果写 `disarm_attempts`/`disarm_forced` | 代码路径审阅；下次实飞验证 |
| S1 | UDP 报文畸形会让 tracker 崩溃 | `poll()` 捕获解析异常、丢弃并计数（`dropped`/`last_drop_reason`），并校验 p/v 长度与非有限值 | 单测：畸形包 + 长度错误 + NaN 均被丢弃，随后正常报文仍可接收 |
| E1 | 估计器噪声与调用顺序相关、无法表达无效量测 | 每通道派生独立随机流；`measure()` 可返回 `None`（`dropout_probability`），由 `select_target_state` 保持上一帧估计 | 单测：默认不丢包、全丢包返回 `None`、丢包可复现、首帧无效即报错 |

### C1 的量化证据（为什么它值得改）

用最小被控对象（加速度 = 推力·thr2acc·机体 z 轴 − g·ẑ，不含内环延迟）直接度量
**“指令加速度 → 实际加速度”** 的映射误差：

| 指令 a_x | 指令 a_z 增量 | 旧写法实际（水平/垂向/倾角） | 新写法实际（水平/垂向/倾角） |
|---|---|---|---|
| 2.0 | 0.0 | 1.986 / **−0.203** / 11.68° | 2.000 / 0.000 / 11.52° |
| 4.0 | +6.5（爬升权） | **6.468** / 5.163 / **23.36°** | 4.000 / 6.500 / 13.78° |
| 4.0 | −6.5（下降权） | **1.313** / −6.771 / 23.36° | 1.543 / −6.500 / 25.00°（限幅） |
| 18.0 | 0.0 | 4.146 / **−0.919** / 25.00° | 4.574 / 0.000 / 25.00° |

三个可直接读出的结论：

1. **第 2 行是“x–z 耦合”的直接证据**：有爬升权（`a_z = g+6.5`）时旧写法把倾角给到
   23.36°（应为 13.78°），水平加速度超出指令 62%；一旦转为下降权（`a_z = g−6.5`），
   水平响应又塌到指令的 33%。同一条水平指令，实际横向增益由 `a_z` 决定 ⇒ 高度环一
   动，水平环增益就跟着变，两侧互相激励。
2. **第 1 行是垂向缺口的直接证据**：11.68° 倾角下悬停推力少 `cos` 分量 0.203 m/s²，
   即“一倾斜就掉高”；25° 时该缺口为 0.919 m/s²（配合 `Kp2=5` 约 0.18 m 稳态误差）。
3. 限幅后新写法的横向权限为 `a_z·tan25°`（正确的饱和值），旧写法只有
   `a_z·sin25°`——限幅区的横向权限也偏小，且随 `a_z` 变化。

因此把 C1/C1b/C3 一起改是必要的：只改除数会留下垂向缺口，只补补偿会在下降权时
过冲，三者是同一个几何自洽性问题的三个面。

**悬停点未被改动（因此既有标定与验证结果继续有效）**：`a_h = 0` 时两种写法都给出
零倾角，补偿系数为 1，推力仍为 `g/thr2acc = hover_percentage`。实测
`hover_percentage = 0.2893` 与标定值逐位相等、期望四元数为单位四元数。也就是说这次
只修正了**偏离悬停时**的映射，`logs/px4ctrl/measure-hover-*` 的标定值不需要重测。

### 下一步（按优先级）

1. **重飞 A/B 实验**：先做“target 单机航点”，再做“两机同飞”，对比修复前后的
   水平/高度耦合振荡。预期：单机无变化；两机的 x–z 同步摆动应显著减弱或消失。
2. 若两机仍有振荡，再按原计划测仿真实时率与控制环实际频率（`world.current_time`
   对比墙钟），并复测 `--no-stream-ui`，区分“CPU 竞争导致的消息延迟”与“控制律”。
3. C1 改动后横向权限在限幅区由 `a_z·sin25°` 变为 `a_z·tan25°`（+10%），
   重飞时留意横向是否过于激进；必要时下调 `Kp` 或收紧 `max_angle`。

### 油门/推力模型：RLS 恢复为可用状态（2026-09-19）

**先说结论：那段 RLS 代码从未被删除过，问题是它从来没有调用点。** 本移植忠实搬了
`estimate_thrust_model`，但没有任何地方调用它；而上游 `PX4CtrlFSM.cpp:344` 在
`AUTO_HOVER` 与 `CMD_CTRL` **每个周期**都会调用。所以它一直是死代码，而不是“被扔掉”。

现在它是一条真实可用的通路，并且能用于真机建油门模型：

| 改动 | 内容 |
|---|---|
| 调用点 | `PX4CtrlFSM.process()` 在 `AUTO_HOVER`/`CMD_CTRL` 下调用（与上游一致），由 `thrust_model.online_estimate` 控制；`--online-estimate` 可对单次运行打开 |
| 配对窗口 | `estimate_delay_min_s`/`estimate_delay_max_s` 可配置。**这是它以前“看起来没用”的真正原因**：原窗口固定 35~45 ms，而本项目 `rate_hz = 20`（周期 50 ms）大于窗口上界，窗口内永远没有样本 |
| 降级配对 | 周期大于窗口时可用最近样本配对（`estimate_allow_degraded`），并计数；`real.yaml` 已把 `rate_hz` 提到 50 使严格窗口可达 |
| 安全限制 | `thr2acc` 只能在标定值的 `estimate_min_ratio`~`estimate_max_ratio` 倍内变动，越界更新**被拒绝**并计数（IMU 符号接错/标定差一个量级时不会被带跑） |
| 可观测性 | `ThrustEstimateStats`（更新/拒绝/空窗/降级/负加速度）；`print_value` 打开后每 5 s 打印推力模型；`run.json` 新增 `thrust_model` 段与 `samples[].thr2acc` 序列 |
| 反标定通道 | `suggested_hover_percentage()` 把收敛后的 `thr2acc` 反算回 `hover_percentage`，可直接写回 YAML 固化 |

**顺带纠正一个我先前的错误结论**：我曾写“倾角补偿与在线估计不应同时开启，否则估计会被
系统性压低”。这是错的。补偿（除以 `cosφ·cosθ`）是**指令的一部分**，被控对象会忠实执行，
因此 `est_a[2] = 已发送指令 · thr2acc` 在任意倾角下仍然成立，两者可以同时开启。

**验证**（合成被控对象，真值已知，见 `tracking/tests/test_thrust_model.py`，新增 13 项测试）：

| 场景 | 结果 |
|---|---|
| 50 Hz，真值 1.3× 标定值 | 严格窗口配对（`degraded=0`），收敛到 0.36% 内 |
| 20 Hz（当前仿真配置） | 降级配对（`degraded=39`），收敛到 0.000% |
| 50 Hz + 2% 量测噪声 | 8 s 后误差 0.077% |
| IMU 符号接反（真值为负） | `rejected=98`、`negative_accel=98`，**模型纹丝不动**（仍为标定值） |
| 真值为标定值 5 倍 | `rejected=39`，模型不变，并提示标定量级有问题 |

注意 `est_a` 的坐标系要求：必须是**机体 FLU 系的比力 z 分量**（`link.imu.acc[2]`，悬停
约 `+g`），不能用世界系垂向加速度。代码里保留了计数器，状态机在持续为负时打印
`WARN: 机体 z 轴比力持续为负`。完整标定步骤见
[测试与诊断脚本](test_scripts.md)。

### 飞行日志取证：振限循环 → 翻转 → EKF 崩坏（2026-09-19）

17:20–17:27 那轮实飞用的是**修改前**的代码。事后从 PX4 的 `.ulg` 日志（落盘在
`/tmp/tmp*/log/`，不是仓库内，容易被忽略）把机理挖出来了：

| 指标 | target（出问题） | tracker |
|---|---|---|
| 飞行时长 | 54 s | 36 s |
| \|roll\| 最大 | **178.3°（翻转）** | 27.6° |
| \|pitch\| 最大 | 89.0° | 25.1° |
| 最大倾角 | **107.8°** | 34.5° |
| 倾角 > 24°（接近 25° 限幅）的时间占比 | **32.4%** | **24.7%** |
| 前 32 s 内 pitch 变号次数 ⇒ 振荡周期 | 19 次 ⇒ **≈3.4 s** | 31 次 ⇒ ≈2.1 s |
| 最长连续饱和 | 10.2 s（t=32.2~42.4，即翻转段） | 1.0 s |

**因果顺序（由时间线钉死，不是推测）**：

1. t<32 s：两台都在 ±25° 限幅附近持续振荡（这是已记录的 x–z 耦合振限循环，本次首次
   量化出周期与占比）。target 因为制导给了加速度前馈，激励更大。
2. t=32.2 s：target 进入 10.2 s 的连续饱和，姿态发散到 roll=71.7°→178°（翻转），
   日志里有 10 个 \|az\|>60 m/s² 的冲击样本；随后落到地面并以 4.5 m/s 水平速度滑出。
3. t=36 s 之后：EKF 的 `accel_bias[0]` 从 ±0.15 涨到 **0.400 = `accel_bias_limit`**，
   `accel_bias_stable` 转 False。
4. t=47.7 s：`local_position_invalid` / `local_velocity_invalid` / `global_position_invalid`
   同时置位 → `Failsafe: blind land`，nav_state 变成 DESCEND(12)，failsafe=1。
5. 最终水平位移 **28.09 m**（末值 27.95 m），即场景里 `v` 读到的 `target=(-25.11,…)`
   **是真实位移**，不是坐标系假象。

**这条时间线推翻了我先前的一个判断**：不是“EKF 退化导致失控”，而是**先振限循环、
再翻转坠地、EKF 才被冲击带崩**（偏置在翻滚前一直只有 ±0.15，翻身后才饱和到 0.4）。
tracker 那台同样的控制器、同样的限幅，但没有前馈激励，只小幅振荡、未翻转、无 failsafe。

**因此下一轮实飞有一个可证伪的判据**（修改前基线 → 修改后目标）：

- 倾角 > 24° 的时间占比：target 32.4% / tracker 24.7% → 应显著下降；
- 最大倾角：107.8°（target） → 必须 < 30°；
- 姿态翻转（\|roll\|>90°）与 \|az\|>60 m/s² 冲击样本数：必须为 0；
- `accel_bias[0]`：不再逼近 0.400 上限。

复现命令（就绪后）：

```bash
# 1) 取最新 ulg（PX4 写在 /tmp，不在仓库里）
ls -lt /tmp/tmp*/log/*/*.ulg | head
# 2) 用 PX4 venv 的 pyulog 复查姿态与偏置时间线（脚本见本文档取证段落）
PYTHONPATH= PX4-Autopilot/.venv/bin/python -c "from pyulog import ULog; print('pyulog ok')"
```

另外确认了收尾问题：控制台里 `WARN [commander] Disarming denied: not landed` 之后是
`Landing detected` / `Disarmed by landing` —— 正是 L3 修复覆盖的场景，旧代码只能记 WARN。

### 后续安全加固：持续饱和回退与无阻塞诊断（2026-09-19）

- **新增 `CMD_CTRL` 倾角饱和看门狗**：控制器在限幅**前**记录
  `debug.tilt_saturated`，避免把“限幅后刚好等于上限”的正常数值误判为饱和。仅当该标记
  连续达到 `0.5 s`，FSM 才记录告警、以当前实测 ENU 位置重置悬停点并转入 `AUTO_HOVER`；
  单次短暂触限不会触发。切换的同一控制周期会重新计算悬停输出，不会多发送一帧旧的饱和
  指令。该防线直接针对 ULog 中 target 连续饱和 10.2 s 后翻转的失控链条。
- **target 航点的 1 Hz 诊断不再阻塞 20 Hz 控制**：`[target-state]` 输出改入容量为 1 的
  后台队列。终端/管道拥塞时只保留最新状态并丢弃旧诊断；`fsm.process()`、UDP 状态发布和
  设定点下发均不等待 stdout。交互航点的即时提示保持原样。
- **离线验证**：控制律/FSM 窄测试 15 项通过，航点诊断窄测试 2 项通过；完整
  `tracking/tests` 套件共 **61 项**通过，三个修改模块的 `py_compile` 通过。该验证只说明
  状态机与脚本语义成立，尚不替代新场景下的 target 单机和双机复飞验收。

### A/B 实验：平滑轨迹本身正确，振荡只在“两机同飞”时出现（2026-09-19）

换上五次多项式轨迹后复飞仍失败，于是做受控 A/B（同一轨迹、同一场景、EKF 健康检查
仅剩启动瞬态的 `no heading reference`）。

**A 组：只飞 target → `PASS: target-waypoints`**

```
local=(-3.38,+0.32,+2.01) shared=(-0.39,+0.29,+2.07)   # 两坐标系偏移恒定 ≈ +2.99
本地航点误差 2.947 → 2.594 → 2.020 → 1.247 → 0.508 → 0.102 → 0.050 m  # 单调收敛
```

- 结论 1：**local ENU 与 shared ENU 始终保持固定偏移** → 此前的振荡是**真实运动**，
  不是估计发散或坐标系假象。
- 结论 2：**五次多项式轨迹工作正常**，3 m 机动单调收敛到约 5 cm，无超调。

**B 组：相同轨迹，但 tracker 同时飞行 → 复现并加重振荡**

```
ref_local 全程恒为 +2.70（参考稳定），而 target 本地 x 在 0.16 ~ 3.88 m 间往复
→ 幅值由 ±1.5 m 增长到 ±2.3 m、周期约 10 s 的极限环；z 同步摆动 0.35 ~ 2.99 m
最终本地误差 1.176 m → FAIL，且振荡把 target 带到世界 x = -25 m（漂了约 28 m）
```

- 结论 3：**触发条件是“第二架同时飞行”**，与参考轨迹无关（参考稳定）、
  与估计无关（两坐标系一致）。相同轨迹下单机 5 cm、双机 ±2 m 的对比可复现。
- 收尾：两机均落地并上锁；target 走的是 PX4 失效保护路径
  （`Failsafe: blind land` → `Disarmed by landing`），不是我们的正常 disarm——
  说明退出阶段位置估计也不可靠，收尾逻辑需要加固。

**尚未定位的机制（下一步诊断顺序）**：

1. **优先**：测量仿真实时因子与控制环实际达成频率。控制外环跑**墙钟 20 Hz**，
   而 PX4 姿态/角速率内环跑在**仿真时间**；若双机 + 渲染负载使仿真慢于实时，
   外环在仿真时间里的有效频率下降、相位裕度丢失 → 低频极限环。
   做法：场景里打印 `world.current_time` 与墙钟对比；控制进程统计实际循环频率。
2. 用 `--no-stream-ui` 复测 B 组，隔离渲染/推流负载（本会话出现过
   `nvstPushStreamData timeout`）。
3. tracker **只解锁悬停、不做机动**再复测，区分“第二架在空中”与“第二架也在机动”。
4. 若确认为环路频率问题：提高墙钟控制频率以补偿实时因子，或适当降低 `Kp/Kv`。

**已排除**：长时运行导致 EKF 退化（新场景健康检查干净仍复现）；阶跃饱和
（已换平滑轨迹且单机 PASS）；坐标系/估计发散（两坐标系偏移恒定）。

### 首次双机实飞：两次尝试均安全结束，但暴露 EKF 姿态失效（2026-09-19）

- **飞行 1（阶跃航点）**：两机起飞正常；target 悬停期间 tracker 站位误差 **≈1 cm**
  （`logs/tracking/tracker-v0-20260919-132331`）。但 target 执行 3 m 阶跃航点时出现
  持续振荡（世界 x 在 3.3–7.2 m、z 在 1.65–3.47 m），60 s 内未进入 0.25 m 容差 →
  FAIL；tracker 在 target 停止发布状态后按 0.5 s 超时**自动降落**。两机均落地并上锁。
- **根因分析（阶跃 + 倾角饱和）**：控制器 `Kp≈6`，3 m 误差要求 `18 m/s²`，而
  `max_angle: 25°` 把水平加速度限制在 `g·tan(25°) ≈ 4.6 m/s²`。环路饱和 → 持续超调
  振荡；水平饱和又削弱垂直推力分量，把高度一起带成耦合振荡。
- **修复：新增 `tracking/tracking/trajectory.py`（五次多项式航段）**，用它替代阶跃，
  并把位置/速度/加速度**全部前馈**给控制器。`target_waypoints.py` 的脚本航点与交互航点
  都已改为每周期下发平滑参考（此前交互模式只在输入时下发一次，等于阶跃）。
  新增 8 项单测；其中一个单测抓到我的一个真实 bug：按**分轴**距离求段时长只能保证
  每轴不超限，合成速度模长会到 `1.014·v_max`，改为按**向量模长**约束后才成立。
- **飞行 2（平滑航段，v≤0.8 m/s、a≤1.0 m/s²）**：两机起飞、落地均正常，但仍 FAIL——
  这次原因不同：target 的**共享 ENU 高度从 +2.09 m 缓慢降到 −3.29 m 又回升**（约 5 m
  漂移），tracker 如实跟随了这个错误估计。**两机同时缓慢下沉又不撞地，说明是估计问题而非真实机动。**
  证据：`logs/tracking/tracker-v0-20260919-132758`、`logs/tracking/target-waypoints-20260919-132750`。
- **PX4 侧证据：`WARN [health_and_arming_checks] Preflight Fail: Attitude failure (roll/pitch)`**
  在本次会话中多次出现 → **EKF 姿态估计失效**。我们的控制器直接消费 EKF 的
  `odom.q`（ATTITUDE_QUATERNION）与 `odom.v`（LOCAL_POSITION_NED），因此在 EKF 不健康时
  不可能收敛。
- **待查（在解决前不要继续实飞）**：
  1. **【已定位主因】场景长时运行导致 PX4 IMU/EKF 健康度下降**。本次场景连续运行
     **4.4 小时**（最后一条 carb 时间戳 `15,843,560 ms`），同一会话内累计：
     `Attitude failure`×7、`High Accelerometer Bias`×3、`High Gyro Bias`×1、
     `velocity unstable`×2；同时仿真侧出现 `poll timeout`×4 与
     `nvstPushStreamData timeout`×2（物理步进/推流卡顿）。IMU 偏差估计恶化会让 EKF 的
     姿态与位置缓慢漂移，正是飞行 2 中“共享高度缓慢漂移 5 m”的直接原因。
     **结论：每次飞行实验前重启场景，不要复用长时运行的场景；**
     以后排查顺序应为“先看健康检查与仿真停帧，再看自定义模块”。
  2. 该 PX4 构建内有一个自定义模块 `vision_target_estimator`（启动时打印
     `VTE for static target init`）。目前**没有证据**表明它与本次异常相关（它只打印了
     初始化信息），保留观察，但优先级低于第 1 条。
  3. 连续飞行之间没有重置 EKF 原点：target 上一次飞行后落点已偏移 2.1 m，下次起飞从
     新位置开始，而 EKF 原点仍是旧的。
- 结论：**平滑轨迹修复方向正确且有单测保证，但尚未在健康 EKF 下完成实飞验证**；
  当前阻塞点是姿态/位置估计，不是制导或控制器。

### 视频流内嵌 HUD：根因、修复与实测结论（2026-09-19）

- **⚠️ 两次段错误的真实根因是我的代码，不是 Isaac 环境**：HUD 里写了
  `self._models[role] = label.model`，但 `omni.ui.Label` **只有静态文本、没有 `model`
  属性**。该 `AttributeError` 逃逸出场景 `__init__`，随后在 Kit 的
  `Py_FinalizeEx` / atexit 清理阶段段错误，表现为“Isaac 启动即崩溃”。
  证据：`dual_px4_20260919-125021.log` 中的 Python traceback 指向
  `vehicle_monitor.py:51`；崩溃栈顶部是 `atexit_callfuncs → Py_FinalizeEx`，
  说明段错误是**次生**结果。
- **正确写法（已核实 API）**：`ui.Label(arg0: str)` 不支持模型；
  实时刷新必须用 `ui.SimpleStringModel` + 只读多行 `ui.StringField(model=..., read_only=True)`。
- **已加降级保护**：HUD/视角控制创建包在 `try/except` 内，失败时关闭 UI 开关并回到
  终端 1 Hz 状态输出。UI bug 今后最多只丢失 HUD，不会再拖垮整个场景。
- **`--stream-ui`（原应用 + `hideUi=false`）已验证可用，并改为默认启动方式**。
  实测证据（`logs/isaac_px4/dual_px4_20260919-125328.log`）：
  - kit 参数 `--/app/window/hideUi=False` + `--no-window`，流媒体服务器已启动；
  - 日志出现 `[ui] HUD 已叠加到视频流`，无 AttributeError、无段错误；
  - 两台 PX4 均 `Ready for takeoff`；
  - 在场景终端实测四条视角命令：`v` → `view=follow_tracker target=(+2.97,-0.01,+0.06)
    tracker=(-3.03,-0.01,+0.06)`；`2` → `follow_target`；`4` →
    `onboard_tracker @ /World/tracker_uav/body/front_camera`（相机路径由遍历 stage
    自动取得）；`3` → 切回 `follow_tracker` 成功（默认相机恢复路径有效）。
- **仍不可用的两项**：
  - `--stream-ui-full`（`isaacsim.exp.full.streaming.kit`）在本机扩展启动阶段
    段错误（约第 43 s，早于场景代码执行），原因未定，**不要使用**；
  - `--gui`：`DISPLAY=:0` 存在但鉴权失败（`Invalid MIT-MAGIC-COOKIE-1 key`）。
- 终端状态行补充了速度模长与**两机间距**（`gap=`），比六个坐标分量更直接地反映跟踪几何。

### 视频流 UI 的首次尝试（已被上一条修正）

### 视频流内嵌 HUD 与视角控制（2026-09-19）

- **根因：为什么画面里只有图像、没有属性**。默认启动用的是 `base.python.kit` 且
  `hide_ui=True`，WebRTC 只合成渲染画面。官方 `isaacsim.exp.full.streaming.kit`
  显式设置 `hideUi = false`（见该 app 文件的 `[settings.app.window]` 段），
  才是"无窗口 + UI 随视频流发送"的官方方案。因此启动脚本改为默认
  `--stream-ui`：headless 下加载该 streaming 应用，HUD 与 Stage/Property 面板
  一同进入视频流；`--no-stream-ui` 可退回纯画面流。
- **状态显示位置从终端移到画面**：新增 `src/simfordrone/vehicle_monitor.py`
  的 `VehicleHudWindow`，在视频流上叠加两机世界 ENU 位置/速度/速度模长、
  `stage_path` / `vehicle_id` / PX4 `system_id` / MAVLink 端口与当前视角。
  仅有 UI 合成时才绘制；纯画面流仍保留终端 1 Hz 的 `[vehicle-state]` 作为降级观测。
- **视角控制**：新增 `src/simfordrone/view_control.py`。指令
  `1 overview / 2 target / 3 tracker / 4 onboard / v status` 在场景终端输入：
  - 跟随视角用 `set_camera_view(eye, target)`，每帧按载具位置重算，
    偏移固定为 `(-5, -5, +3) m`（与场景默认取景同向）；
  - 机载视角用 `set_active_viewport_camera(<载具>/body/front_camera)`，把 viewport
    挂到载具子相机上，随载具运动；**该模式下绝不能再调用 `set_camera_view`**，
    否则会把载具自己的相机挪走、破坏视觉观测。返回时先恢复
    `/OmniverseKit_Persp` 再设置观察视角。
  - 命令解析是纯函数 `parse_view_command`，用 `select` 零超时轮询 stdin，
    不阻塞仿真主循环；`src/` 中没有 Isaac 顶层导入，因此可单测。
- **相机路径不能写死**：Pegasus 用 `get_stage_next_free_path` 生成
  `<载具>/body/<相机名>`，同名时可能追加后缀，且相机 prim 在传感器 `start()`
  阶段才真正建立。因此改为遍历 stage 查找 `UsdGeom.Camera`，并在主循环里
  低频重扫直到拿到路径；缺失时自动退回跟随视角而不是卡在错误相机上。
- 单元测试从 10 项增至 **16 项**（新增视角命令解析与模式回退测试）。
- 截至本次记录，场景已验证可启动并加载 streaming 应用；视频流内 HUD 的实际
  可见性需以浏览器画面确认为准。

### V0 双机 GT 跟踪与观测改进（2026-09-19）

- **静止 target 加噪闭环基线（已通过）**：平均站位误差 `0.0161 m`、最大 `0.0301 m`，
  双机均落地并上锁。证据 `logs/tracking/static-v0-20260918-232517/`。
- **架构拆分为两个独立进程**（此前单进程同时占用 14540/14541，无法分终端操作）：
  - `scripts/run_target_waypoints.sh`：独占 target `14540`，执行航点，并通过本机 UDP
    `127.0.0.1:14600` 发布共享 ENU 真值；
  - `scripts/run_tracker.sh`：独占 tracker `14541`，订阅上述状态后执行 V0 跟踪。
  拆分原因：同一 UDP 端口不能同时被两个客户端有效读取，共用会互相抢包。
- **⚠️ “两机相距很远”的根因（已定位，非控制律问题）**：
  - `logs/tracking/tracker-v0-20260918-235711` 显示 tracker 直接以
    “未收到新鲜 target 状态”失败，**未解锁起飞**；因此命令行传入的
    `--offset-east -3` 从未生效，看到的落点是上一轮遗留位置。
  - `logs/tracking/tracker-v0-20260918-235540` 记录实际锁存 offset 为
    `(-4.47, -0.04, -0.09) m`，说明**未显式传参时按实测相对位置锁存**，与命令行
    数值无关。加上固定时长脚本两进程容易错位，才会出现“跑完相距很远”的观感。
  - 同一记录可见 20 s 后误差由 `0.15 m` 增长到 `0.69 m`：target 仍在继续航点，
    而 tracker 已接近其 30 s 窗口末尾 → 固定时长是主要误差来源。
- **因此改为手动航点模式**：`--interactive` 用 `select` 零超时轮询 stdin，等待键盘
  输入期间仍以 20 Hz 持续下发设定点（`input()` 会阻塞并触发 Offboard failsafe）；
  tracker 支持 `--duration 0` 持续跟踪到中断或目标状态超时。非法输入不改变航点。
- **状态超时保护**：target 状态超过 `0.5 s` 未更新时 tracker 退出跟踪并安全降落，
  避免跟踪陈旧目标。
- **Isaac 可观测性改进**：`--gui` 保留原生 Stage/Property 面板（可查看
  `/World/target_uav`、`/World/tracker_uav` 全部 USD 属性），并新增
  `SimForDrone Vehicles` 监视窗（世界 ENU 位置/速度 + stage 路径 + system_id +
  MAVLink 端口，10 Hz 刷新）；headless/WebRTC 无 UI，改为终端 1 Hz 输出
  `[vehicle-state]` 同内容。位姿统一取自 Pegasus `vehicle.state`，不引入第二套来源。
- 单元测试从 7 项增至 **10 项**（新增状态通道往返/超时、手动命令解析），全部通过。

### T2 / 共享坐标系 / 整定（2026-09-18 晚）

- **T2 通过**：tracker 站位保持稳态误差均值 `0.014 m`、最大 `0.020 m`（门槛 0.5 m）
  （`logs/px4ctrl/hold-tracker-20260918-214228/`）；目标机同任务稳态均值 `0.023 m`
  （`logs/px4ctrl/hold-target-20260918-214430/`）。
- 运行记录保留数由 5 提高到 **20**：单次仅约 30 KB，而轮转过早会让文档引用的证据消失
  （上一轮 `hold-target` 记录即因此被清掉，需要重跑生成）。
- **双机共享坐标系已实现并实测验证**。方案：不依赖 ROS，用 MAVLink `GLOBAL_POSITION_INT`
  加一个**两机共用**的地理原点换算到共享 ENU（`frames.geodetic_to_enu`）。
  仿真原点取 Pegasus 世界原点 `38.736832 / -9.137977 / 90.0`（`PegasusSimulator/.../config/
  configs.yaml`）；真机填入实测 home 或 RTK 基准即可，路径完全一致。
- **⚠️ 地球模型必须与 GPS 产生方一致（此前的归因有误，已更正）**：Pegasus 的重投影
  （`geo_mag_utils.reprojection`）是**球面**且半径取 `EARTH_RADIUS = 6353000.0`。我最初反解
  用了 WGS-84 椭球，在纬度 38.74° 处 `N/R ≈ 1.0052`，即东向有约 0.5% 系统性尺度差。
  改为同一球面模型后实测：东向偏差 `1.95 cm → 1.0 cm`，而**北向几乎不变**（该方向模型尺度差
  仅 0.08%），构成自洽检验。结论：残差约**一半是模型不一致（已消除），一半是真实 GPS 噪声
  与 EKF 估计误差**（Pegasus 的 GPS 会注入噪声与 60 s 相关时间的随机游走偏置，
  `GLOBAL_POSITION_INT` 又是 EKF 融合值）。配置项 `shared_frame.earth_radius` 用于选择模型：
  仿真填 `6353000.0`，真机留空即用 WGS-84。
- 同时刻对比（模型一致后）水平偏差约 `0.7~1.3 cm`、高度 `2~3 mm`。
- **关键设计约束（勿混淆两套坐标系）**：位置设定点由 PX4 在**本机 local 系**解释，而共享系
  是另一套原点。因此 `link.odom.p` **始终保持本机 local ENU**（控制用），共享系位置单独放在
  `link.shared_position`（仅供多机相对几何）。若把 `odom.p` 直接改成共享系，位置设定点会整体
  偏移。实测佐证：target 的 local 原点已偏离其出生点 `0.61 m`，两机 local 系无法互比。
- **起飞→悬停交接改为依据实测高度**（`fsm.py` 的 `HANDOVER_TOLERANCE_M = 0.10`）。原实现只看
  时间，斜坡比实际爬升快，实测交接瞬间机体落后 `0.66 m`，该落差直接成为保持误差峰值。
  修前/修后（同一任务、同一机体）：

  | 指标 | 时间交接 | 实测交接 |
  |---|---|---|
  | 保持误差均值 | `0.107 m` | `0.048 m` |
  | 保持误差最大 | `0.666 m` | `0.100 m` |
  | 稳态误差均值 | `0.051 m` | `0.045 m` |
  | 末端高度 | `1.979 m` | `1.974 m`（指令 2.0） |

  另：交接时若实测已进入容差，悬停点取**指令点**（闭合残差、保住高度精度）；仅超时才退化为
  实测点（避免指令突变）。
- **整定结论：增益不是瓶颈**。加大阻尼 `Kv=(4,4,5)→(6,6,6)` 仅带来约 5~11% 的边际改善
  （稳态均值 `0.0455→0.0430`、末端高度 `1.974→1.984`），属单次样本、可能在重复性噪声内。
  稳态残差已接近估计器噪声下限（GPS/EKF 相对真值本身就有 2~3 cm 偏差），故**保留与上游一致的
  基线增益**以维持可比性。临时增益配置通过 `--config` 覆盖，未写入基线。
- **推力标定获得独立印证**：稳态推力指令实测 `0.290`，与标定值 `hover_percentage = 0.2893`
  几乎一致。
- 新增整定所需的**时间序列记录**：`hold` 任务输出 `samples`（t/误差/高度/推力）与稳态统计
  （去掉前 8 s 瞬态），使"该调哪个增益"可被数据回答而非猜测。

## 最近记录（早前）

- **命令执行模块已统一到 `px4ctrl/`**，不再分散。分层：`params`（YAML+校验）/
  `vehicle`（角色→端口/system id）/ `frames`（ENU·FLU ↔ NED·FRD，带可执行自检）/
  `inputs` / `controller`（线性几何控制器）/ `link`（唯一接触 MAVLink 的模块）/
  `fsm`（手动·悬停·指令跟踪·自动起降）/ `cli`（唯一入口）。
  入口：`./scripts/run_px4ctrl.sh <task> --role <role> [--execute]`。
- 已删除被其取代的冗余文件：`src/simfordrone/{offboard_control,offboard_hold_validation,
  vehicle_roles,tracker_takeoff_validation}.py`、
  `scripts/run_{offboard_hold_validation,tracker_takeoff_validation}.sh`。
  删除前已确认这些模块仅被彼此与各自启动脚本引用。
- 标定与运行参数已集中到 YAML（`px4ctrl/config/{sim,real}.yaml`）：
  推力标定值 `thrust_model.hover_percentage`（含方法/日期/证据路径）与 `tasks` 段默认
  （高度、保持时长、频率、各类超时）。日常运行只需 `--role`/`--execute`，命令行同名参数
  仅作覆盖。**标定值直接复用，不必每次重跑。**
- 推力标定已实测复用而非重跑：`hover_percentage = 0.2893`，方法为 position 模式悬停取
  稳定段 `SERVO_OUTPUT_RAW` 均值（124 样本），与上游默认 `0.30` 吻合。
  它与场景侧 `rotor_input_scaling=2000`（Pegasus 的 PX4 输出→转子转速增益，被控对象标定）
  是两层不同的量，不可互相替代。
- px4ctrl 首次闭环中出现并修复的缺陷（均已复测）：`await_ack` 传入 `tick=None` 时从不
  收包导致 ACK 永远超时；`LOCAL_POSITION_NED` 的速度分量被丢弃导致 D 项恒为零；
  位置设定点的 ENU 偏航角被直接当 NED 下发（相差 `π/2 − yaw`）；`ImuData` 缺姿态四元数
  导致姿态补偿无法计算；`measure-hover` 缺收尾降落。
- **`.etli` 跟踪日志更正**（此前记录有误）：`--/app/livestream/webrtcEtli=0` **实测无效**，
  该开关在编译库 `libNvStreamServer.so` 的 `EtliTracing` 内，命令行无法关闭。实测一次
  5 小时运行会写出单个 368 MB 文件。
  现采取两层约束：① 启动脚本把工作目录切到 `logs/nvstreamer/`，`.etli` 不再落到仓库根
  （此前累计 232 MB / 14 个已在根目录）；② 新增后台守护，单文件超过 `ISAAC_ETLI_MAX_MB`
  （默认 64）即 `truncate` 清零，使磁盘占用有上界，并在场景退出时回收守护进程。
  启动时另按 `ISAAC_ETLI_KEEP`（默认 2）清理历史文件。
- 双机 ROS 接口通过：`/target_uav_0/state/pose`、`/tracker_uav_1/state/pose` 及 tracker RGB-D 话题均可采样。
- RGB-D 样本可保存至 `logs/rgbd_samples/`；相机时间与 PX4 位姿时间未对齐，当前不得进入视觉闭环。
- 场景已从 `Curved Gridroom` 和程序化机库改为官方 Isaac `full_warehouse.usd`。它提供真实
  货架、叉车、工业材质和灯光；首次启动需缓存依赖。尚未运行验收，不能声称对 6D pose 有效。
- 场景超参数已集中到 `configs/dual_uav_hangar.yaml`；`src/simfordrone/config.py` 在启动时
  校验顶层结构。YAML 离线解析与场景代码静态编译已通过，尚未重启加载。
- P2.0 通过证据：控制 `logs/takeoff_validation/20260917-210831/` 的三项命令均接受；
  位姿 `logs/takeoff_validation/20260917-210917/` 满足 `z_max_after_ground_baseline=2.514618 m`
  与 `z_final=0.054701 m`。同轮 PX4 ULog 的峰值为 `2.459 m`，并已自动降落/解锁。
- `20260917-185323` 曾被旧验收器误判为通过：它记录的是场景启动时从 `2 m` 自然落地，
  不含与控制命令重叠的起飞过程。验收器现要求地面基线后才统计离地高度。
- `20260917-185253` 的控制日志中 ARM 被 PX4 拒绝；它发生在场景尚未稳定时，不能评价
  新推力标定。下一轮须先确认 tracker 已贴地且 PX4 显示 `Ready for takeoff!`。
- 1700 档有效复验中仅到 `z_max=0.073031 m`；2000 档确认能离地。启动时可通过
  `SIMFORDRONE_PX4_INPUT_SCALING` 单变量覆盖，下一步先延长保持时间而不改增益。
- 启动脚本新增 `--gui`（`SIMFORDRONE_ISAAC_GUI=1`），用于加载 Isaac Sim 标准界面
  （状态栏、Stage、Property、Timeline）；默认仍为 headless + WebRTC，避免无图形显示的
  服务器在 UI 初始化阶段失败。
- 取景相关的临时改动已撤回：双机出生点恢复为 `target [3,0,2]`、`tracker [-3,0,2]`，
  相机姿态恢复 `[0,0,180]`，采样器的 `5 s` 稳定帧门限已移除。`pegasus_compat.py`
  中修复 RGB-D writer 创建的补丁保留，因为它与画面构图无关。
- 双机跟踪规划的关键事实：Pegasus `ROS2Backend` 的 `sub_control` 只订阅转子转速
  `/<ns><id>/control/rotor<i>/ref`，**不是**高层控制通道；因此跟踪闭环必须走 PX4
  MAVLink。目标机的 `port=14540 / system_id=1` 尚未单独验证，是 T0 的内容。
- 控制层已按选定的 Offboard 方案实现：`src/simfordrone/vehicle_roles.py`（角色→端口/
  system id 的唯一映射）、`src/simfordrone/offboard_control.py`（单线程阶段机，20 Hz
  持续设定点，等待 ACK 期间也保持流，所有退出路径兜底 `LAND`）、
  `src/simfordrone/offboard_hold_validation.py`（CLI，默认 dry-run）与
  `scripts/run_offboard_hold_validation.sh`。
- 已离线核实而非猜测的数值：位置+偏航 `type_mask=2552`；Offboard 主模式 `6`、
  AUTO_LAND 子模式 `6`（取自 `PX4-Autopilot/src/modules/commander/px4_custom_mode.h`）；
  `SET_POSITION_TARGET_LOCAL_NED` 编码为 msgid 84、MAVLink1 下 61 字节。
  已验证失败路径：无场景时连接超时并干净退出（退出码 1），日志仍落盘。
- 待办：需在场景运行时执行 `--role target`（先 dry-run 再 `--execute`）才能通过 T0，
  目前尚无运行证据。

### T0 运行结论（2026-09-18）

- **PX4 MAVLink 端口模型已实测确认**（源码依据 `PX4-Autopilot/ROMFS/px4fmu_common/
  init.d-posix/px4-rc.mavlink`）：
  | 用途 | 端口 | 方向 |
  |---|---|---|
  | offboard 本地（PX4 绑定） | `14580 + instance` | 客户端实际发往此处 |
  | offboard 远端（PX4 发往） | `14540 + instance` | 客户端 bind 此端口收 heartbeat |
  | GCS 链路本地 | `18570 + instance` | 仅此链路有 `STATUSTEXT` |
  | HIL（Pegasus TCP） | `4560 + vehicle_id` | PX4 作为客户端连 Isaac |

  实测 target(instance 0) → `14540` → system id `1`；tracker(instance 1) → `14541` →
  system id `2`。两者均通过 dry-run。
- 早前"`14540` 收不到 heartbeat"的根因是**残留的第二个 Isaac 实例占用了 HIL 端口
  `4560/4561`**，不是端口映射错误。并发启动两个场景会同时抢 `49100` 与 `4560/4561`；
  启动前必须确认没有其它实例在运行。
- **onboard 链路收不到 `STATUSTEXT`**：`-m onboard` 实例不下发该消息，实测 12 s 内
  一条都没有，因此不能用 "Ready for takeoff" 文本作为就绪门限（会导致永远无法起飞）。
  就绪判定改为遥测：`LOCAL_POSITION_NED` 已收到 且 `EXTENDED_SYS_STATE.landed_state
  == ON_GROUND`。该文本仍可从 GCS 链路（`18570+i`）或场景日志读到。
- **`MAV_CMD_DO_SET_MODE` 的命令编码与旧版 PX4 不同**：本分支
  （`v1.18.0-beta1-577`，`3ebb2923a1`）在 `Commander.cpp` 中按**逐字节**解析——
  `param1`=base_mode 标志、`param2`=主模式、`param3`=子模式；而旧约定是把
  `(main<<16)|(sub<<8)` 打包进 `param2`。沿用旧约定时打包值被 `(uint8_t)` 截断为 0，
  PX4 报 `Unsupported main mode` 并以 `TEMPORARILY_REJECTED` 拒绝。
  **但 HEARTBEAT 的 `custom_mode` 仍是打包结构**，其主模式位于 bit 16-23，故遥测解析
  仍用 `(custom_mode >> 16) & 0xFF`；命令与遥测的语义必须区别对待。
- 落地后不能立即读 `armed`：实测按地瞬间仍为 `armed=True`，与 PX4 自动上锁存在竞态。
  收尾改为"先等自动上锁，超时后显式上锁并确认"，通过条件用 `disarmed_after_land`。
- 站位误差从进入 `altitude - 0.25 m` 窗口后才开始统计；否则记录的是爬升瞬态，其最大
  误差必然等于目标高度，无法反映保持性能（修正前 `hold_error_max_m` 恒为约 2.0 m）。
- 安全设计已被真实触发并验证：`DO_SET_MODE` 被拒时，兜底 `LAND` 仍被执行
  （`on_ground=True`，`armed=False`），未出现载具滞留解锁状态。

操作命令见 [运行手册](runbook.md)，整体技术路线见
[双机跟踪实施规划](uav_visual_tracking_project_plan.md)，两套仿真/自主框架的架构见
[Pegasus 与 Aerostack2 架构解读](pegasus_aerostack2_architecture.md)。
