# px4ctrl（SimForDrone 统一控制环）

本包是 SimForDrone 的**唯一命令执行模块**：所有对飞控的操控都经过这里，控制代码不再分散
到多个脚本。结构对照上游 `Fast-Gamma/src/realflight_modules/px4ctrl`（ROS 1 + MAVROS），
去掉 ROS 版本依赖后以 MAVLink 直连，其余分层与命名保持一致。

## 为什么要集中

之前控制逻辑分散在 `offboard_control` / `offboard_hold_validation` / `vehicle_roles` /
`tracker_takeoff_validation` 等多个文件中，容易出现"端口表被复制两份而失去一致性"、
"某个脚本忘了收尾降落"这类问题。现在端口、命令、坐标转换、状态机各有唯一归属。

## 模块划分

| 模块 | 对应上游 | 职责 |
|---|---|---|
| `params.py` | `PX4CtrlParam` | YAML → 不可变参数，加载期校验 |
| `vehicle.py` | — | 角色 → 端口 / system id / ROS 命名空间 |
| `frames.py` | MAVROS 代劳 | ENU/FLU ↔ NED/FRD，**含可执行自检** |
| `inputs.py` | `input.h` | 输入状态与新鲜度判定 |
| `controller.py` | `controller.h` | 线性几何控制器（推力 + 姿态） |
| `link.py` | MAVROS | MAVLink 连接层：**唯一**接触飞控协议的模块 |
| `fsm.py` | `PX4CtrlFSM` | 手动/悬停/指令跟踪/自动起降 |
| `cli.py` | `px4ctrl_node` | 唯一命令入口 |

## 仿真与真机

控制器、状态机、坐标转换对仿真与真机**完全一致**；差异只在参数文件的 `link` 段：

```yaml
# sim.yaml —— 留空即使用 vehicle.py 的角色端点
link:
  connection: ""              # → udpin:0.0.0.0:14540（target）/ 14541（tracker）
# real.yaml
link:
  connection: "serial:/dev/ttyACM0:921600"   # 或 udpout:<飞控IP>:14550
```

## SO(3) body-rate 模式

默认 `use_bodyrate_ctrl: false` 发送四元数姿态和推力。设置为 `true` 后，控制器以精确
$Log(R^T R_d)$ 姿态误差和角速度误差构造 FLU body-rate 设定点，再以
`SET_ATTITUDE_TARGET` 的 body-rate 模式发送给 PX4。PX4 继续负责角速度到力矩/电机的
内部闭环。

该模式不是直接力矩控制，且目前只经离线回归验证。先完成仿真与单机低高度验收，再考虑
启用；`so3.max_bodyrate` 默认限制为 `3 rad/s`，不要在获得该验收日志前提高。

## 用法

```bash
# 只探测端点，不发任何命令（安全）
./scripts/run_px4ctrl.sh probe --role target

# 位置模式悬停，测量执行器指令以标定 hover_percentage
./scripts/run_px4ctrl.sh measure-hover --role target --execute

# 姿态+推力控制：自动起飞 → 悬停 → 降落 → 上锁
./scripts/run_px4ctrl.sh takeoff-hover-land --role target --execute

# 真机：先改 link 段，再用同一入口
./scripts/run_px4ctrl.sh takeoff-hover-land --role tracker --profile real --execute
```

运行参数（高度、保持时长、频率、各类超时）默认取自 YAML 的 `tasks` 段，命令行同名参数
仅作覆盖。日志写入 `logs/px4ctrl/<task>-<role>-<时间戳>/run.json`，只保留最近 5 次。

## 标定

**`hover_percentage` 是控制器唯一的推力标定量**：本控制律中 `thrust = des_a_z / thr2acc`
且 `thr2acc = gra / hover_percentage`，因此悬停时的推力指令恰好等于该值。取值与来源记录在
`config/sim.yaml` / `config/real.yaml` 的 `thrust_model` 注释中，**直接复用，不必每次重跑**。

它与场景侧 `configs/dual_uav_hangar.yaml` 的 `rotor_input_scaling`（Pegasus 的 PX4 输出→
转子转速增益，被控对象标定）是两层不同的量，不可互相替代。

复测方法：

```bash
./scripts/run_px4ctrl.sh measure-hover --role target --execute
```

## 必须知道的协议细节

1. **`MAV_CMD_DO_SET_MODE` 逐字节解析**：本工程所用 PX4 分支的 `param2` = 主模式、
   `param3` = 子模式；旧版把 `(main<<16)|(sub<<8)` 打包进 `param2` 会被 `(uint8_t)` 截断为
   0 并报 `Unsupported main mode`。而 HEARTBEAT 的 `custom_mode` 仍是打包值。
2. **onboard 链路不下发 `STATUSTEXT`**：不能用 "Ready for takeoff" 文本作就绪门限；
   就绪判定用 `LOCAL_POSITION_NED` + `EXTENDED_SYS_STATE.landed_state`。
3. **onboard 链路没有 `HIL_ACTUATOR_CONTROLS`**（那只发往仿真链路），执行器输出取自
   `SERVO_OUTPUT_RAW`。
4. **Offboard 要求设定点流 > 2 Hz**，中断即 failsafe；因此收包与发流在同一循环内，
   等待 ACK 期间也不中断。
5. **各机 PX4 EKF 的局部原点互相独立**，双机相对位置不能用两条 `LOCAL_POSITION_NED`
   相减，必须经共享坐标系（T3 前必须解决）。

## 共享坐标系（多机必需）

各机 PX4 EKF 的 `LOCAL_POSITION_NED` 原点由**各自的**起飞点建立，因此两机的 local 位置
不在同一系里（实测 target 的 local 原点已偏离其出生点 0.61 m），**直接相减得不到真实相对
位置**。启用 `shared_frame` 后，位置改由 `GLOBAL_POSITION_INT` 相对一个**两机共用**的地理
原点换算到共享 ENU：

```yaml
shared_frame:
  enabled: true
  origin_latitude: 38.736832     # 仿真取 Pegasus 世界原点；真机填实测 home 或 RTK 基准
  origin_longitude: -9.137977
  origin_altitude: 90.0
```

⚠️ **两套坐标系不要混用**：

| 属性 | 坐标系 | 用途 |
|---|---|---|
| `link.odom.p` | 本机 local ENU | **控制与位置设定点**（PX4 用本机系解释 setpoint） |
| `link.shared_position` | 共享 ENU | **多机相对几何**（T3 制导） |

把 `odom.p` 当成共享系会让位置设定点整体偏移。姿态与速度无需换算：local NED 与共享 ENU
同为"上=上"，只差原点，不差朝向。

实测验证（同时刻对比 ROS 真值）：水平偏差约 2 cm、高度 4~6 mm，残差来自各机 EKF 估计误差。

## 起飞→悬停交接

交接依据**实测高度**（`fsm.HANDOVER_TOLERANCE_M = 0.10 m`）而非仅凭时间。时间参数化的斜坡
比实际爬升快，只看时间会导致交接瞬间机体落后 0.66 m，该落差会直接变成保持误差峰值；
改为实测交接后，同一任务的保持误差最大从 `0.666 m` 降到 `0.100 m`，稳态均值 `0.045 m`。

## 整定

**增益不是瓶颈**：加大阻尼仅带来约 5–11% 的边际改善，稳态残差已接近估计器噪声下限
（GPS/EKF 相对真值本身有 2–3 cm 偏差）。故保留与上游一致的基线增益以维持可比性。

临时试验用 `--config <临时yaml>` 覆盖，不要改动基线。整定所需的时间序列见 `hold` 任务
输出中的 `samples` 与 `steady_*` 指标。

## 自检

```bash
python -m px4ctrl.frames      # 坐标转换的解析断言（姿态符号、共享系曲率半径的红线测试）
```
