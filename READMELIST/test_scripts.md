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
`logs/px4ctrl/<任务>-<角色>-<时间>/run.json`。飞行任务安全收尾后会自动在同目录生成
`response.png`；绘图失败不会影响 LAND、上锁或任务返回码。

### 阶跃响应与参数整定

不要用双机跟踪或肉眼观察悬停来整定 tracker。先对 tracker 单机运行可重复的单轴位置
阶跃，再依据记录的响应指标调整参数。控制器实现在 `px4ctrl/px4ctrl/controller.py`：位置
外环由 `Kp0/1/2`、`Kv0/1/2` 决定，若启用 SO(3) body-rate 模式，姿态外环再由
`KAngR/P/Y`、`so3.rate_damping` 和 `so3.max_bodyrate` 决定。

SO(3) 首轮只使用临时 profile，不修改 `sim.yaml`：

```bash
./scripts/run_px4ctrl.sh step-response --role tracker \
  --config /tmp/sim-so3.yaml \
  --step-axis east --step-amplitude 0.5 --step-delay 5 --hold-seconds 12 \
  --execute
```

该任务先起飞并悬停，等待 5 秒后沿一个 ENU 轴施加 $0.5\,\mathrm{m}$ 阶跃，持续刷新
`CMD_CTRL`，并沿用倾角持续饱和回退与自动降落上锁。`run.json` 会记录本轮完整控制参数、
`rise_time_s`、`overshoot_m`、`settling_time_s`、`final_error_m`、最大倾角、最大 body-rate
与安全回退状态。

整定顺序固定为：先在四元数模式下整定 `Kp/Kv` 的位置阶跃；SO(3) 模式只在同一位置参数
稳定后，以低 `KAng`/低 body-rate 上限逐步提高。任一试验出现 `safety_fallback=true`、持续
倾角饱和或 yaw 往复时，停止加大增益，先降低 `KAngY` 或 `max_bodyrate` 并重新做单机阶跃。

target 与 tracker 的飞行记录也会自动出图：target 图显示本机 local ENU 实际航迹、轨迹参考
和航点误差；tracker 图显示共享 ENU 下的 target、tracker、期望站位三轴响应，以及跟踪和量测
误差。图像与同目录 `run.json` 一一对应，便于在每轮调参后直接比较。

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
./scripts/check_observation.sh
./scripts/check_observation.sh --rgbd
```

`--rgbd` 样本与场景图像检查均写入 `logs/scene_rgb_checks/`。当前相机和位姿时间基准尚未对齐，
不能把这些观测直接接入视觉控制或 EKF。

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

当前完整套件为 80 项。修改 Python 后可额外执行：

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