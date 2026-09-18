# 双机跟踪测试入口

这里仅保存可执行实验入口，算法实现位于 `tracking/tracking/`，飞控执行仍统一经过
`px4ctrl/`。

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
./test/run_target_waypoints.sh --execute
```

紧接着打开终端 2，启动 tracker：

```bash
./test/run_tracker.sh --execute
```

tracker 默认运行 30 秒、使用加噪 target 状态和占位估计器，并在起飞稳定后锁存当时相对
位置。target 状态超过 0.5 秒未更新时，tracker 会退出跟踪并安全降落。

tracker 终端每秒输出一行实时状态：

```text
TRACK t= 8.0s source=estimator target=(...) tracker=(...) desired=(...) error=0.123m
```

其中 `error` 是 tracker 共享 ENU 真值与期望相对站位之间的瞬时距离。运行结束后还会输出
均值和最大误差，完整时间序列写入 `logs/tracking/tracker-v0-*/run.json`。

查看最新一次 tracker 汇总：

```bash
./test/show_latest_tracking_result.sh
```

## 手动输入航点（推荐）

固定时长的两个脚本容易错位：若 tracker 先于 target 结束，后段误差会被当成跟踪失败。
因此推荐由操作员在 target 终端随时输入航点，并让 tracker 一直跟踪：

终端 1：

```bash
./test/run_target_waypoints.sh --interactive --execute
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
./test/run_tracker.sh --duration 0 --status-period 1 --execute
```

输入解析采用非阻塞轮询，等待键盘输入期间仍以 20 Hz 持续下发 Offboard 设定点，
不会因为 `input()` 阻塞而触发 failsafe。非法输入只提示，不改变当前航点。

## 自定义航点

每个 `--point EAST NORTH UP` 都是相对 target 起飞点的 local ENU 坐标，单位为米。可重复传入：

```bash
./test/run_target_waypoints.sh \
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
./test/run_tracker.sh --duration 30 --status-period 1 --execute
```

无噪真值跟踪对照：

```bash
./test/run_tracker.sh --state-source truth --duration 30 --execute
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

## 场景物体位置与属性查看

Isaac 默认以 headless + WebRTC 运行，WebRTC 只传渲染画面，因此看不到物体属性面板。
两种查看方式：

1. 本地桌面图形界面：

```bash
./scripts/start_dual_px4_scene.sh --gui
```

启动后会出现原生 Stage 树、Property 面板。选中 `/World/target_uav` 或
`/World/tracker_uav` 即可查看该物体的全部 USD 属性（位姿、质量、关节、传感器等）。
场景还会弹出 `SimForDrone Vehicles` 监视窗，持续刷新两架无人机的世界 ENU 位置、速度、
速度模长，以及 `stage_path` / `vehicle_id` / PX4 `system_id` / MAVLink 端口。

2. headless + WebRTC：终端每秒输出一行 `[vehicle-state]`，内容与监视窗相同：

```text
[vehicle-state] target: p=(+3.01,+0.02,+2.00) v=(+0.01,-0.00,+0.00) | tracker: p=(-3.00,+0.01,+2.00) v=(...)
```

两种方式读取的都是 Pegasus 维护的 Isaac 世界位姿（ENU），不引入第二套位姿来源，
可与 `TEST/README` 中的 MAVLink 共享 ENU 相互核对。