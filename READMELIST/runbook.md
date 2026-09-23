# 核心实验流程

本页只描述当前双机真值跟踪实验的主路径。标定、观测检查、诊断、历史验证和全部辅助脚本
见[测试与诊断脚本](test_scripts.md)。调用关系与端口归属见[调用关系整理](invocation_map.md)。

## 当前边界

- 飞控命令只能经 `px4ctrl/` 下发；target 使用 MAVLink `14540`，tracker 使用 `14541`。
- target 与 tracker 必须是两个独立进程；target 通过 UDP `127.0.0.1:14600` 发布共享 ENU 状态。
- 已验证 target/tracker dry-run 与 target 单机 2 m 往返；动态双机跟踪尚未重新验收。
- 固定航点默认限制为 `0.25 m/s`、`0.25 m/s²`。不要在没有新的单机验收时提高它们。

## 1. 启动场景

在终端 1 启动 Isaac、Pegasus 和两套 PX4 SITL。不要激活 Conda，也不要加载系统 ROS 2：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_dual_px4_scene.sh
```

场景脚本启动后即可打开控制终端运行 target；target 程序会自行等待目标机的 MAVLink、遥测与
共享 ENU 就绪。场景终端必须持续运行；停止场景时在此终端按 `Ctrl-C`。

默认 WebRTC 地址为 `10.134.88.113:49100`。本机当前不要使用 `--stream-ui-full`；它已知会在
扩展启动时崩溃。

## 2. 单机 target 航点

场景启动后，直接启动 target。程序会自行等待遥测、共享 ENU 与 PX4 内部健康状态；无需先跑
dry-run，也不需要 `--execute`：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/run_target_waypoints.sh
```

默认路线为起飞、悬停、东向 2 m、返回、降落。出现 `WARN: 倾角持续饱和` 时，控制器会转入
当前位置悬停；不要继续运行，按一次 `Ctrl-C` 让该脚本降落并上锁。

仅在需要排查端点或 UDP 状态流时，才使用不解锁的显式探测模式：

```bash
./scripts/run_target_waypoints.sh --probe
./scripts/run_tracker.sh --probe
```

## 3. 双机真值跟踪

终端 2 直接启动交互 target：

```bash
./scripts/run_target_waypoints.sh --interactive
```

target 起飞稳定后，在终端 3 启动 tracker 与估计器链路：

```bash
./scripts/run_tracker.sh --duration 0 --status-period 1
```

tracker 默认链路为 `target GT -> Gaussian noise -> PassthroughEstimator -> PositionTrackerV0`。
需要无噪真值对照时，改用 `--state-source truth`。target 终端输入 `land`，或在任一控制终端
按一次 `Ctrl-C`，只会让该终端负责的无人机安全降落；不要让两个进程共享同一个 MAVLink 端口。

## 4. 结束与记录

确认两个控制终端都打印 `on_ground=True, disarmed=True` 后，再停止场景终端。每次飞行的
控制记录保存在 `logs/tracking/*/run.json`；恢复动态双机验收时还要检查两机 PX4 ULog 的倾角、
姿态翻转、冲击和 EKF 偏置。

真机油门模型、RLS 在线估计和 `px4ctrl` 单机任务不属于本主流程，见
[测试与诊断脚本](test_scripts.md)。