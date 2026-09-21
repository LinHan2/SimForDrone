# 测试与诊断脚本

本页记录辅助验证、标定、观测、结果检查与离线测试。日常启动 target、tracker 和估计器链路
请使用[核心实验流程](runbook.md)，不要把本页的单项测试拼成新的飞行主路径。

## 控制器单机测试

所有 `px4ctrl` 任务都经过唯一 MAVLink 控制出口：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/run_px4ctrl.sh probe --role target
./scripts/run_px4ctrl.sh probe --role tracker
./scripts/run_px4ctrl.sh takeoff-hover-land --role target --execute
./scripts/run_px4ctrl.sh hold --role tracker --execute
```

运行前确保没有 QGroundControl 或其它进程占用相同 MAVLink 端口。每次任务记录在
`logs/px4ctrl/<任务>-<角色>-<时间>/run.json`。

## 推力与油门模型标定

仿真默认使用已固化的 `hover_percentage`，不需要每次重标。真机时按以下顺序进行：

```bash
# 静态悬停标定，得到 suggested_hover_percentage
./scripts/run_px4ctrl.sh measure-hover --role target --execute --rate-hz 50

# 可选：带遗忘因子的 RLS 在线估计，观察 thr2acc 收敛
./scripts/run_px4ctrl.sh hold --role target --execute \
  --rate-hz 50 --online-estimate --hold-seconds 30
```

将静态结果写入 `px4ctrl/config/real.yaml` 的 `hover_percentage`。RLS 用于交叉校验及观察
载荷/电压变化；收敛后关闭 `online_estimate`，下次起飞使用固化值。`rate-hz 50` 用于满足
35--45 ms 采样配对窗口；若持续出现负比力或大量拒绝更新，先检查 IMU FLU 坐标和符号。

## 场景与传感器观测

这些脚本只读取 ROS 2 或采样文件，不下发飞行控制：

```bash
./scripts/check_dual_uav_observation.sh
./scripts/capture_rgbd_sample.sh
./scripts/check_observer_rgb_scene.sh
```

RGB-D 样本写入 `logs/rgbd_samples/`，场景图像检查写入 `logs/scene_rgb_checks/`。当前相机和
位姿时间基准尚未对齐，不能把这些观测直接接入视觉控制或 EKF。

## 跟踪结果与 ULog

```bash
./scripts/show_latest_tracking_result.sh
ls -lt /tmp/tmp*/log/*/*.ulg | head
```

动态双机复飞后，除查看 `logs/tracking/*/run.json` 外，还要在两机 ULog 中检查：倾角超过
24 度的持续时间、是否有 `|roll| > 90` 度、是否有 `|az| > 60 m/s²` 冲击，以及
`accel_bias[0]` 是否接近 0.400 上限。

## 离线回归

使用 PX4 虚拟环境，避免 Conda 环境缺少 YAML 依赖：

```bash
cd /data/disk2/home/hl/research/SimForDrone
PYTHONPATH=px4ctrl:tracking PX4-Autopilot/.venv/bin/python \
  -m unittest discover -s tracking/tests -p 'test_*.py'
```

当前完整套件为 67 项。修改 Python 后可额外执行：

```bash
PYTHONPATH=px4ctrl:tracking PX4-Autopilot/.venv/bin/python \
  -m py_compile tracking/tracking/target_waypoints.py
```

## 环境 profile

| profile | 使用位置 |
|---|---|
| `scripts/env/activate_px4_mavlink_control.sh` | `run_px4ctrl.sh`、target 与 tracker 控制脚本内部加载 |
| `scripts/env/activate_isaacsim_internal_ros.sh` | 仅 Isaac 场景进程内部 ROS 2 |
| `scripts/env/activate_system_ros2_jazzy.sh` | 只读 ROS 2 观测工具 |

不要混用系统 ROS Python、Conda 和 Isaac 内部 Python。