# 双机跟踪测试入口

本目录集中存放项目的全部 shell 入口（原先散在仓库根、`test/`、`utils/`）；
环境 profile 在 `scripts/env/`，只能被 `source`，不要直接执行。
算法实现位于 `tracking/tracking/`，飞控执行仍统一经过 `px4ctrl/`。

## 职责拆分

```text
target_waypoints (MAVLink 14540)
  ├─ 控制 target 起飞和航点
  └─ UDP 127.0.0.1:14600 发布共享 ENU 真值

run_tracker (MAVLink 14541)
  ├─ 接收 target 状态
  ├─ 可选 truth / estimator
  └─ 只控制 tracker 起飞、跟踪和降落
```

两个进程不会争抢同一个 MAVLink 端口。

## 推荐启动顺序

先启动 Isaac/Pegasus/PX4 场景。然后打开终端 1，启动 target。默认起飞后等待 15 秒，再执行
“原点悬停 → 东向 2 m → 返回原点”：

```bash
./scripts/run_target_waypoints.sh --execute
```

紧接着打开终端 2，启动 tracker：

```bash
./scripts/run_tracker.sh --execute
```

tracker 默认运行 30 秒、使用加噪 target 状态和占位估计器，并在起飞稳定后锁存当时相对
位置。target 状态超过 0.5 秒未更新时，tracker 会退出跟踪并安全降落。

制导与执行之间还有一层保护：进入 `CMD_CTRL` 后，若**指令流本身**超过 `msg_timeout.cmd`
（0.5 s）未更新，状态机会退出指令控制并在**当前点悬停**（不是降落），终端打印
`WARN: 制导指令超时 …`。因此制导循环被拖慢时飞行器会停住，而不是继续追一条陈旧设定点。

tracker 终端每秒输出一行实时状态：

```text
TRACK t= 8.0s source=estimator target=(...) tracker=(...) desired=(...) error=0.123m
```

其中 `error` 是 tracker 共享 ENU 真值与期望相对站位之间的瞬时距离。运行结束后还会输出
均值和最大误差，完整时间序列写入 `logs/tracking/tracker-v0-*/run.json`。

查看最新一次 tracker 汇总：

```bash
./scripts/show_latest_tracking_result.sh
```

## 手动输入航点（推荐）

固定时长的两个脚本容易错位：若 tracker 先于 target 结束，后段误差会被当成跟踪失败。
因此推荐由操作员在 target 终端随时输入航点，并让 tracker 一直跟踪：

终端 1：

```bash
./scripts/run_target_waypoints.sh --interactive --execute
```

起飞稳定后会出现 `waypoint>` 提示符。输入相对起飞点的 ENU 偏移（单位 m）：

```text
waypoint> 3 0 1      # 向东 3 m，同时上升 1 m
waypoint> 3 2 0      # 再向北 2 m（高度回到起点高度）
waypoint> 0 0 -1     # 回到起点上方 1 m 高度
waypoint> status     # 查看 target 当前位置、速度和当前航点误差
waypoint> land       # 安全降落
```

终端 2（`--duration 0` 表示一直跟踪到中断或目标状态超时）：

```bash
./scripts/run_tracker.sh --duration 0 --status-period 1 --execute
```

输入解析采用非阻塞轮询，等待键盘输入期间仍以 20 Hz 持续下发 Offboard 设定点，
不会因为 `input()` 阻塞而触发 failsafe。非法输入只提示，不改变当前航点。

## 自定义航点

每个 `--point EAST NORTH UP` 都是相对 target 起飞点的 local ENU 坐标，单位为米。可重复传入：

```bash
./scripts/run_target_waypoints.sh \
  --point 0 0 2 \
  --point 3 0 2 \
  --point 3 2 2 \
  --point 0 0 2 \
  --start-delay 15 \
  --dwell 4 \
  --final-hold 15 \
  --execute
```

建议 tracker 的 `--duration` 小于 target 的“启动等待 + 航点运行 + 末端保持”总时间。例如：

```bash
./scripts/run_tracker.sh --duration 30 --status-period 1 --execute
```

无噪真值跟踪对照：

```bash
./scripts/run_tracker.sh --state-source truth --duration 30 --execute
```

默认不带 `--execute` 时只做安全探测。注意 tracker 的 dry-run 需要 target 端先运行并发布状态。
按一次 `Ctrl+C` 只会让当前脚本负责的无人机降落，两个进程互不越权。

## ROS 2 迁移

当前 V0 使用 UDP `127.0.0.1:14600` 隔离两个 MAVLink 客户端。下一阶段将只替换消息适配层：

```text
/tracking/target/ground_truth  nav_msgs/msg/Odometry  (map/ENU)
/tracking/target/measurement   nav_msgs/msg/Odometry  (含 pose/twist covariance)
/tracking/target/estimate      nav_msgs/msg/Odometry
/tracking/desired_state        trajectory_msgs/msg/MultiDOFJointTrajectoryPoint
```

推荐节点边界：

```text
target_state_adapter → ground_truth
measurement_model    → measurement
target_estimator     → estimate
position_tracker_v0  → desired_state
px4ctrl_bridge       → MAVLink SET_ATTITUDE_TARGET
```

ROS 只标准化状态和期望指令，最终飞控出口仍由 `px4ctrl` 独占，不能让多个 ROS 节点直接连接
同一 MAVLink 端口。系统 ROS 2 Python 与当前 PX4 venv 分离，因此迁移时应建立一个可同时访问
`rclpy` 和 `pymavlink` 的专用 ROS 控制环境，不能在 Isaac Sim 内部 Python 中运行控制节点。

## 场景物体位置与属性

### 默认模式：HUD 就在视频流里（已验证）

```bash
./scripts/start_dual_px4_scene.sh
```

默认即把 UI 合成进 WebRTC 流：headless（`--no-window`）+ `hideUi=false`，
让 `omni.ui` 窗口参与画面合成。已实测可用（启动后能触发视角命令、无崩溃）：

- `SimForDrone HUD` 窗口：两机世界 ENU 位置/速度/速度模长，以及
  `stage_path` / `vehicle_id` / PX4 `system_id` / MAVLink 端口与当前视角；
- Stage 树与 Property 面板：选中 `/World/target_uav` 或 `/World/tracker_uav`
  可查看全部 USD 属性（位姿、质量、关节、传感器等）。

需要只看画面（例如观看端无法播放该流）时用 `--no-stream-ui`，此时场景改为在终端每秒
输出一行状态：

```text
[vehicle-state] target: p=(+2.97,-0.01,+0.06) |v|=0.00 | tracker: p=(-3.03,-0.01,+0.06) |v|=0.00 | gap=6.00m
```

也可用 `SIMFORDRONE_ISAAC_CONSOLE_STATE=1` 在保留 HUD 的同时强制终端输出。

### 两个已排除/失效的选项

- `--stream-ui-full`（官方 `isaacsim.exp.full.streaming.kit`）在本机**已实测段错误**：
  在该应用内部 rclpy 加载完成后约 0.2 s（启动第 43 s）崩溃，早于我们场景代码执行。
  证据：`logs/isaac_px4/dual_px4_20260919-124503.log`。**不要使用**。
- `--gui`：本机 `DISPLAY=:0` 存在但**无权访问**（`Invalid MIT-MAGIC-COOKIE-1 key`），
  因此本地窗口模式不可用。

### 本地桌面界面（当前不可用）

```bash
./scripts/start_dual_px4_scene.sh --gui
```

需要可用的 X 会话；本机 `DISPLAY=:0` 存在但鉴权失败，因此当前只能用默认的流式 UI 模式。

### 视角切换

在**场景终端**输入下列命令，画面立刻切换（已在运行中的场景实测）：

| 命令 | 效果 |
|---|---|
| `0` 或 `free` | **手动模式（默认）**：程序不写相机，鼠标/键盘自由拖动 |
| `1` 或 `overview` | 固定全局取景，看两机整体几何关系 |
| `2` 或 `target` | 相机跟随目标机（从后上方观察） |
| `3` 或 `tracker` | 相机跟随跟随机 |
| `4` 或 `onboard` | 切到跟随机机载前视相机，真实第一视角 |
| `v` 或 `status` | 打印当前视角与两机世界坐标 |

注意：**跟随类模式（2/3）每帧都会写入相机位姿**，这是跟随的必要代价，也因此会
立即覆盖鼠标拖动；想手动调整就先按 `0`（也有这层含义：之前默认就是跟随，
所以看起来像相机被锁死）。默认已改为 `0`，不会主动干预相机。

跟随视角每帧按载具位置重算（偏移 `(-5, -5, +3) m`）；机载视角把 viewport 挂到载具子相机
`/World/tracker_uav/body/front_camera`（路径由遍历 stage 自动取得，不写死），随载具运动并
显示视觉任务真正使用的图像。相机 prim 晚于场景构建才创建时，场景会低频重扫直到拿到路径。
注：`--no-stream-ui` 不创建 UI，也不会响应这些命令，这是有意的。