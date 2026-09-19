# 运行手册

> 各命令背后的调用链、环境画像取舍、进程间通信与端口归属，见
> [调用关系整理](invocation_map.md)。本手册只给“怎么操作”。

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
./scripts/check_dual_uav_observation.sh
./scripts/capture_rgbd_sample.sh
```

第一条应显示两机位姿、跟随机 RGB、CameraInfo、Depth 均为 `OK`；第二条保存一组
RGB-D 样本至 `logs/rgbd_samples/`。当前相机和位姿的时间基准尚未对齐，不能用于视觉
控制或 EKF。

## 工业机库的 observer RGB 检查

场景改为官方 Isaac Sim `full_warehouse.usd`：真实货架、叉车、工业材质与预制灯光。
它不再使用本项目拼出的圆柱/立方体机库；首次启动会下载并缓存资产依赖。重启场景后执行：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/check_observer_rgb_scene.sh
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

如需独立于控制器核验位姿，仍可使用只订阅 ROS 的 `./scripts/record_tracker_takeoff_pose.sh`。
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

### 推力模型与指令新鲜度（2026-09-19 修复后）

`thrust_model` 段有两个真正影响行为的开关（`accurate_thrust_model` 是上游遗留项，
只读不用，保留以保持与上游逐字段可比）：

| 开关 | 默认 | 作用 |
|---|---|---|
| `tilt_compensation` | `true` | 推力除以 `cosφ·cosθ`，补掉倾斜造成的垂向推力缺口（25° 时约 9.4%） |
| `online_estimate` | `false` | 启用带遗忘因子的 RLS **在线估计 `thr2acc`**（油门/推力模型）。仿真保持关闭（已有可追溯的静态标定）；真机建油门模型时打开 |

两个开关**可以同时开启**：倾角补偿是指令的一部分，被控对象会忠实执行，因此
`est_a[2] = 已发送指令 · thr2acc` 在任意倾角下依然成立，补偿不会给估计带来偏差。

倾角反解已改为精确形式（`sinφ = a_y^b/\|a\|`、`θ = atan2(a_x^b, a_z)`，除数不再是常数
`g`）。运行中如需对比旧行为，只能临时改代码，**不要**通过改 `gra` 来凑——那会同时破坏
重力补偿。

`msg_timeout.cmd` 现在**真正生效**：`CMD_CTRL` 下指令超过该时限未更新，状态机会退出
指令控制并**在当前点悬停**（不是降落）。因此制导进程卡死或减速时，飞行器会停住而不是
继续追一条陈旧设定点；终端会打印一条 `WARN: 制导指令超时 …`。收尾上锁也变为两级：
常规上锁被拒时会在有限等待后**强制上锁**，`run.json` 里新增 `disarm_forced` 与
`disarm_attempts` 便于区分“PX4 拒绝”与“我们没重试”。

## 真机油门/推力模型标定

“油门模型”在本控制器里就是一个标量：`thrust = des_a_z / thr2acc`，其中
`thr2acc = gra / hover_percentage`。因此标定目标就是得到可靠的 `hover_percentage`。
两种互补方法，建议都做：

### 方法 A：静态标定（`measure-hover`，不估模型）

用位置模式悬停，直接读执行器输出均值：

```bash
./scripts/run_px4ctrl.sh measure-hover --role target --execute --rate-hz 50
```

结果里的 `suggested_hover_percentage` 写回 `px4ctrl/config/real.yaml`。这是最直接的
“油门↔悬停”标定，不依赖 IMU 与推力动力学。

### 方法 B：在线 RLS（`--online-estimate`，需要 IMU）

用于在真实飞行条件下把“推力指令 ↔ 机体 z 轴加速度”的系数也标出来，可以交叉校正
方法 A，也能看出电池电压下降/载荷变化带来的漂移：

```bash
# 1) 打开日志（real.yaml 已设 print_value: true），每 5 s 打印一行推力模型
# 2) 用 50 Hz 运行，使控制周期（20 ms）小于配对窗口下限（35 ms）
./scripts/run_px4ctrl.sh hold --role target --execute --rate-hz 50 --online-estimate --hold-seconds 30
```

看什么：

- 终端 `[thrust-model]` 行的 `thr2acc` 是否收敛，以及反算出的 `hover_percentage`；
- `更新/拒绝/空窗/降级` 四个计数：`拒绝>0` 说明 IMU 符号/坐标系可疑或标定差一个量级
  （模型不会被带跑，但你需要查原因）；`空窗` 持续增长说明控制频率低于配对窗口；
- `logs/px4ctrl/hold-*/run.json` 的 `thrust_model` 段与 `samples[].thr2acc` 序列，
  离线可以看完整收敛曲线。

收敛后把 `suggested_hover_percentage` 写回 `real.yaml`，并把 `online_estimate` 重新关掉：
下次起飞直接用固化好的标定值，避免初期的在线自适应暂态。

### 频率与编码约束

- **控制周期必须显著小于 `estimate_delay_min_s`（0.035 s）**，否则窗口内无样本，只能靠
  `estimate_allow_degraded` 降级配对（悬停/慢速几乎无偏，激烈机动会有滞后偏差）。
  `real.yaml` 已把 `rate_hz` 提到 50。
- `est_a` 必须是**机体 FLU 系的比力 z 分量**（即 `link.imu.acc[2]`，悬停时约为 `+g`），
  **不能**用世界系垂向加速度（那会多出 `cos` 因子）。若看到持续的
  `WARN: 机体 z 轴比力持续为负`，先查 IMU 坐标系与符号，不要继续飞。
- 估计范围为标定值的 `estimate_min_ratio`~`estimate_max_ratio` 倍（默认 0.5~2.0），
  越界更新会被拒绝：如果从一开始就一直被拒，说明 `hover_percentage` 量级就不对，
  先用方法 A 重新标定再回来。
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
