# 运行手册

## 当前状态

- 双机 PX4、ROS 2 位姿、跟随机 RGB-D 已运行验收；
- P2.0 跟随机起飞—降落已通过：PX4 命令均接受，ROS 位姿最高 `2.515 m`、最终 `0.055 m`；
- T0 目标机单机 Offboard 验证已通过：最高 `2.012 m`、站位误差均值 `0.041 m`、落地并上锁；
- 尚未开始真值双机跟踪、视觉位姿或 EKF。

## 启动双机 Isaac/PX4

在场景终端中执行。不要激活 Conda，也不要 `source /opt/ros/jazzy/setup.bash`：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_dual_px4_scene.sh
```

当前场景终端必须先按 `Ctrl-C` 停止，新的推力标定才会生效。Windows WebRTC 使用
`10.134.88.113:49100`；该地址可连接仅代表 TCP 信令，视频仍依赖 UDP `47998`。

本机桌面有可用图形显示时，使用以下命令加载 Isaac Sim 的完整界面：

```bash
./scripts/start_dual_px4_scene.sh --gui
```

界面的 Stage 面板中选择 `/World/target_uav` 或 `/World/tracker_uav`，再在 Property
面板查看该对象的 Transform；Timeline 和底部状态栏也会一同加载。此模式依赖本机
图形显示，SSH-only 服务器继续使用默认 WebRTC 模式。要核验飞行过程中的数值状态，
仍使用 ROS 2 位姿主题，因为它反映 Pegasus/PX4 发布的运行时状态。

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

## P2.0 跟随机起飞验证（历史记录）

本节记录最早用 `MAV_CMD_NAV_TAKEOFF` + `ARM` + `NAV_LAND` 完成的一次验证（已通过：
ROS 位姿最高 `2.515 m`、最终 `0.055 m`），**其脚本已被 px4ctrl 取代并删除**。
现在要验证 tracker 起飞应使用下一节的 px4ctrl 入口：

```bash
./scripts/run_px4ctrl.sh takeoff-hover-land --role tracker --execute
```

如需独立于控制器核验位姿，仍可使用只订阅 ROS 的 `./utils/record_tracker_takeoff_pose.sh`。
历史证据保留在 `logs/takeoff_validation/`。

## px4ctrl 控制环（唯一命令入口）

所有对飞控的操作都经过 `px4ctrl/`，不要再写新的控制脚本。参数（含推力标定与任务默认值）
在 `px4ctrl/config/{sim,real}.yaml`，见 [px4ctrl 说明](../px4ctrl/README.md)。

控制通道是 PX4 Offboard + 姿态/推力设定点（`SET_ATTITUDE_TARGET`），20 Hz 持续流。
不用 Pegasus 的 ROS 控制通道：它在 `sub_control` 下只订阅转子转速
`/<ns><id>/control/rotor<i>/ref`，从那里下发会绕过 PX4 控制器。

场景稳定后执行。默认 dry-run：

```bash
cd /data/disk2/home/hl/research/SimForDrone

# 只探测端点，不飞行
./scripts/run_px4ctrl.sh probe --role target      # system id 1 / 端口 14540
./scripts/run_px4ctrl.sh probe --role tracker     # system id 2 / 端口 14541

# 姿态+推力闭环：自动起飞 → 悬停 → 降落 → 上锁
./scripts/run_px4ctrl.sh takeoff-hover-land --role target --execute

# 站位保持（T2）
./scripts/run_px4ctrl.sh hold --role tracker --execute

# 复测推力标定
./scripts/run_px4ctrl.sh measure-hover --role target --execute
```

高度、保持时长、频率与各类超时取自 YAML 的 `tasks` 段，命令行同名参数仅作覆盖。
同一端口只能有一个 MAVLink 客户端：运行前关闭 QGC。

通过条件（脚本自动判定并以退出码报告）：`ARM` 与 `SET_MODE(OFFBOARD)` 均 `ACCEPTED`；
`HEARTBEAT` 中确实观测到主模式 `6`；`max_altitude_m >= 0.8 × altitude`；
结束后已落地且已上锁。`measure-hover` 还必须有可用标定样本。

日志位于 `logs/px4ctrl/<任务>-<角色>-<时间戳>/run.json`，只保留最近 5 次。

失败与安全：

- 设定点流低于 2 Hz 或中断会触发 PX4 failsafe，因此 `rate_hz` 不允许 ≤ 2；
- 所有退出路径（正常、异常、`Ctrl-C`）都会兜底 `LAND` 并上锁；`Ctrl-C` 只按一次；
- 就绪判定不使用 `STATUSTEXT`（onboard 链路不下发该消息）。

## 三个必须知道的坑

1. **并发启动两个场景会互抢端口**（`49100` 与 HIL 的 `4560/4561`），会表现为某个角色
   收不到 heartbeat。启动前先确认没有其它 Isaac 实例在运行。
2. **onboard 链路不下发 `STATUSTEXT`**，所以 `Ready for takeoff` 只能从 GCS 端口
   `18570 + instance` 或场景日志看到，不能作为就绪门限。同理，onboard 链路上没有
   `HIL_ACTUATOR_CONTROLS`，执行器输出取自 `SERVO_OUTPUT_RAW`。
3. **`MAV_CMD_DO_SET_MODE` 的 `param2` 是主模式字节**（本 PX4 分支），不是旧版的
   打包 `custom_mode`；写错会被 `(uint8_t)` 截断为 0 并报 `Unsupported main mode`。
   而 HEARTBEAT 里的 `custom_mode` 仍是打包值。细节见
   [项目进度](progress.md) 的 T0 运行结论。

## 失败时

- 话题缺失：先检查场景终端是否仍在运行；
- 命令未收到 heartbeat：停止其他 MAVLink 客户端，确认仅控制 `system ID 2`；
- 命令接受但未离地：保留本轮 ULog 和位姿日志，不要连续重发控制命令；当前仿真低于实时，
  不能用 8 秒墙钟时间判断推力不足。

下一步实现仅读取两机真值的跟踪基线；随后才以视觉相对位姿和协方差替换真值并接入 EKF。
