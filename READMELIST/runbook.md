# 运行手册

## 当前状态

- 双机 PX4、ROS 2 位姿、跟随机 RGB-D 已运行验收；
- P2.0 跟随机起飞—降落已通过：PX4 命令均接受，ROS 位姿最高 `2.515 m`、最终 `0.055 m`；
- 尚未开始真值跟踪、视觉位姿或 EKF。

## 启动双机 Isaac/PX4

在场景终端中执行。不要激活 Conda，也不要 `source /opt/ros/jazzy/setup.bash`：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_dual_px4_scene.sh
```

当前场景终端必须先按 `Ctrl-C` 停止，新的推力标定才会生效。Windows WebRTC 使用
`10.134.88.113:49100`；该地址可连接仅代表 TCP 信令，视频仍依赖 UDP `47998`。

若需复测其它档位，单次启动可覆盖而不改代码，例如：

```bash
SIMFORDRONE_PX4_INPUT_SCALING=2000 ./scripts/start_dual_px4_scene.sh
```

## 验收 ROS 2 观测

另开终端：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./utils/check_dual_uav_observation.sh
./utils/capture_rgbd_sample.sh
```

第一条应显示两机位姿、跟随机 RGB、CameraInfo、Depth 均为 `OK`；第二条保存一组
RGB-D 样本至 `logs/rgbd_samples/`。当前相机和位姿的时间基准尚未对齐，不能用于视觉
控制或 EKF。

## 工业机库的 observer RGB 检查

场景改为官方 Isaac Sim `full_warehouse.usd`：真实货架、叉车、工业材质与预制灯光。
它不再使用本项目拼出的圆柱/立方体机库；首次启动会下载并缓存资产依赖。重启场景后执行：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./utils/check_observer_rgb_scene.sh
```

它保存样本至 `logs/scene_rgb_checks/`，并检查 RGB 的曝光、对比度、边缘和色彩多样性。
`basic_observer_rgb_readable=true` 只表示图像可读，不代表 target 可见或 6D pose 已验证。

## YAML 超参数

默认文件是 `configs/dual_uav_hangar.yaml`，集中保存 PX4 推力、两机初始位姿、observer
相机、WebRTC 观察相机与官方环境 USD URL。先做离线解析检查：

```bash
cd /data/disk2/home/hl/research/SimForDrone
PYTHONPATH=src /usr/bin/python3 -c 'from simfordrone.config import load_dual_uav_config; print(load_dual_uav_config().keys())'
```

可通过 `SIMFORDRONE_CONFIG=/绝对路径/实验.yaml` 切换同结构参数组；临时推力复测仍使用
`SIMFORDRONE_PX4_INPUT_SCALING=...`，它只覆盖 YAML 的 `rotor_input_scaling`。

## P2.0 跟随机起飞验证

场景稳定、ROS 检查通过后，开两个额外终端。QGC 非必需，也不要运行其他绑定 UDP
`14541` 的 MAVLink 客户端。

先确认 `check_dual_uav_observation.sh` 中 tracker 高度约为 `0.055 m`；场景刚重启时
从约 `2 m` 自然落地的过程不是一次起飞。

终端 A：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./utils/record_tracker_takeoff_pose.sh --duration 50
```

终端 B：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/run_tracker_takeoff_validation.sh
./scripts/run_tracker_takeoff_validation.sh --execute --hold-seconds 25
```

唯一通过条件：控制日志三个命令均 `ACCEPTED`，且位姿 `summary.json` 满足
`ground_baseline_found=true`、`z_max_after_ground_baseline >= 0.8`、`z_final <= 0.30`。
日志位于 `logs/takeoff_validation/`。

## 失败时

- 话题缺失：先检查场景终端是否仍在运行；
- 命令未收到 heartbeat：停止其他 MAVLink 客户端，确认仅控制 `system ID 2`；
- 命令接受但未离地：保留本轮 ULog 和位姿日志，不要连续重发控制命令；当前仿真低于实时，
  不能用 8 秒墙钟时间判断推力不足。

下一步实现仅读取两机真值的跟踪基线；随后才以视觉相对位姿和协方差替换真值并接入 EKF。
