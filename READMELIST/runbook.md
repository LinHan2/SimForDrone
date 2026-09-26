
## 三阶段运行结构

目标运行结构固定为三阶段：

1. **仿真器阶段**：只启动 Isaac、Pegasus 和两套 PX4 SITL，不接收航点控制逻辑。
2. **双机控制阶段**：同一常驻服务同时让 target 与 tracker 解锁、进入 Offboard、起飞悬停；该服务是
	两个 MAVLink 端口的唯一拥有者，并保持接收外部航点命令。
3. **航点发布阶段**：独立发布器只向本地命令 topic 发送 target/tracker 的航点或轨迹请求，绝不直接
	打开 MAVLink 端口。动态跟踪时，服务内部以 target 的实测共享 ENU 状态生成 tracker 制导。

> 当前代码仍处于过渡实现：`run_target_waypoints.sh` 与 `run_tracker.sh` 分别拥有 `14540` 和
> `14541`，并各自承担起飞、跟踪和降落。它可用于下面的动态真值验收，但尚未实现上述第二阶段的
> 常驻双机控制服务及第三阶段的命令 topic 发布器。

## 1. 仿真器阶段

在终端 1 启动 Isaac、Pegasus 和两套 PX4 SITL。不要激活 Conda，也不要加载系统 ROS 2：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_dual_px4_scene.sh
```


## 2. 当前过渡控制阶段

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

## 3. 当前过渡航点与跟踪

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

### 八字动态真值验收

静止目标验证通过后，以八字轨迹验证 tracker 在已知、准确目标运动状态下的动态跟踪性能。
这不是视觉或估计器验收：tracker 必须使用 `--state-source truth`，目标发布的仍是**实测**共享
ENU 位置和速度，而不是规划参考。先在终端 2 启动 target；`--start-delay 45` 留出 tracker
起飞、锁存安全相对站位并切入 `CMD_CTRL` 的时间：

```bash
./scripts/run_target_waypoints.sh \
	--trajectory figure8 --trajectory-radius 1.0 --trajectory-cycles 2 \
	--max-speed 0.40 --max-accel 0.35 --start-delay 45
```

在 target 起飞后立即于终端 3 启动 tracker。此处 `--duration 0` 保证 tracker 不会先于
target 退出；target 的轨迹与末端保持结束后会自行降落，状态流超时将促使 tracker 安全降落：

```bash
./scripts/run_tracker.sh --state-source truth --duration 0 --settling-seconds 5 --status-period 1
```

验收时读取同次 `logs/tracking/target-trajectory-*/run.json` 与 `tracker-v0-*/run.json`：两端必须
均为 `on_ground=true`、`disarmed=true`；`tilt_saturated_samples` 必须为 0，tracker 的
`cmd_ctrl_fraction` 应接近 1，`error_mean_m` 与 `error_max_m` 是剔除起飞后前 5 s 的动态站位
误差。每次运行的 `response.png` 仍保存在对应日志目录，副本统一写入 `picture/<任务-时间>.png`。
其中 target 的 `response.png` 俯视图单独显示实际/参考八字，tracker 的 `response.png` 俯视图
单独显示 target 真值、期望站位与 tracker 实际轨迹。场景默认使用开放网格；修改场景配置后须
完全重启场景进程才会生效。
先记录无噪真值基准，再允许用同一轨迹改为 `--state-source estimator` 进行噪声/EKF 对照。