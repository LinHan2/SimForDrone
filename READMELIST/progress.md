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
| P2.3-T3：真值双机跟踪 | 进行中 | 静止 target 加噪闭环已通过；已拆为“target 航点”与“tracker 跟踪”两个独立进程，含手动航点模式与 Isaac 车辆监视窗；10 项单测、双进程 dry-run 通过。 |
| P3：视觉位姿/EKF | 未开始 | 前置为量化图像与位姿的时间偏差。 |

`vision/` 已建立为独立模块，但按阶段约束暂不接入控制；T3/T4 通过后再以视觉相对位姿替换
真值输入，所有飞控命令仍统一经过 `px4ctrl/`。

## 最近记录

### V0 双机 GT 跟踪与观测改进（2026-09-19）

- **静止 target 加噪闭环基线（已通过）**：平均站位误差 `0.0161 m`、最大 `0.0301 m`，
  双机均落地并上锁。证据 `logs/tracking/static-v0-20260918-232517/`。
- **架构拆分为两个独立进程**（此前单进程同时占用 14540/14541，无法分终端操作）：
  - `test/run_target_waypoints.sh`：独占 target `14540`，执行航点，并通过本机 UDP
    `127.0.0.1:14600` 发布共享 ENU 真值；
  - `test/run_tracker.sh`：独占 tracker `14541`，订阅上述状态后执行 V0 跟踪。
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
