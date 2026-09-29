
# 运行手册

## 启动

每轮实验先在一个终端启动双 PX4 场景，直接把 WebRTC 主画面切到 tracker 真实机载相机：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_dual_px4_scene.sh --onboard
```

该入口仅对自动启动的两台 PX4 SITL 将模拟电池放电时间设为 3600 秒
（PX4 默认 60 秒）；可在启动前设置 `SIMFORDRONE_PX4_SIM_BAT_DRAIN` 覆盖秒数。
修改后须重新启动场景才会作用于 PX4；不会禁用低电量、Offboard 丢失等安全检查，
也不会影响真机。若再次发生 RTL，应检查当轮 PX4 ULog 的 failsafe 原因，
不能仅凭 `battery warning (fast)` 提示音判断为低电量。

用 Isaac Sim WebRTC 客户端连接场景终端打印的地址（默认 `10.134.88.113:49100`），
即可实时观察 tracker 的前视画面，目视检查 target 是否一直在画面内。
场景终端输入 `4` 可重新切回机载视角，`0` 切回自由视角，`1` 查看全局；
这是实际渲染相机视口，不是 `Camera View (IBVS)` 的模拟投影窗口。
画面可用于人工监看，但不会自动记录失视时长，也不能证明控制器保证持续可见。

场景就绪后，在第二个终端运行唯一的飞行编排入口（会解锁两架飞机）：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/run_target_shadow_gated.sh --execute -- \
	--trajectory circle --trajectory-radius 8 --trajectory-cycles 2 \
	--max-speed 3.0 --max-accel 2.5
```

这是显式高速配置；默认仍为 0.25 m/s，不会自动提速。3 m 半径使圆周
参考峰值超过 2 m/s 时所需加速度保持在 2.5 m/s² 以内；实际速度取决于
控制器、跟踪误差和 PX4 状态，不保证一定超过 2 m/s。高速轨迹可能使目标
离开机载视野，FOV 门只检查起点。任务结束后查看终端打印的 target
`PLOT` 路径下的 `response.png`：右上角对比参考与实测速度和 2 m/s 线，
其余子图检查轨迹、误差；tracker 也会输出自己的 `response.png`。

脚本启动 target（MAVLink `14540`）悬停，再启动 truth tracker（`14541`）；
tracker 进入跟踪且 Oracle FOV + RGB-D 深度连续 15 帧有效后，启动只读 shadow EKF，
再放行 target 轨迹。无需手动切换多个控制终端。日志路径在终端打印，位于
`logs/tracking/gated-*/`；结束时检查两机日志中的 `on_ground` 与 `disarmed`。
按 `Ctrl-C` 会请求本次启动的两机安全降落；脚本不终止已有场景或其他实验进程。
tracker 默认采用 20° 的组合倾角上限和 0.35 rad/s 的偏航参考限速（可通过
`run_tracker.sh` 的 `--max-tilt-deg`、`--max-yaw-rate` 单独调整）；target 的控制参数不变。
两项限制可减轻姿态突变，但不保证 target 在整个轨迹中保持可见。
双机入口还会让 tracker 沿起飞时的水平视线以 0.5 m/s 渐进接近 5 m，
高度差保持不变；`tracker.log` 的 `distance`/`desired` 分别为实际/期望水平间距。
实际距离进入 5 ± 0.5 m 后才检查 FOV 并放行 target；超时不会放行。
tracker 的 `response.png` 右上角展示实际/期望水平间距，配合误差图检查往复摆动。
这不是机载图像闭环，也不能保证高速运动时图像不抖动或目标始终入镜。

## 边界与故障

- 当前 tracker **仅用 truth 状态跟踪**；shadow 使用 target PX4 姿态与 tracker RGB-D，
	只记录相对 EKF，不驱动 tracker。禁止用 `relative-ekf` 执行闭环飞行。
- FOV 门只验证起始时刻仿真投影与深度，不是实际图像检测，也**不保证全程可见**。
	若门控超时或任一控制器异常退出，脚本不放行或停止目标轨迹并请求降落。
- UDP `14601` 已占用时脚本拒绝启动，不会停止端口原有进程。
- 只读诊断可用 `./scripts/check_observation.sh --rgbd`；离线测试可用
	`PYTHONPATH=tracking:px4ctrl PX4-Autopilot/.venv/bin/python -m unittest discover -s tracking/tests -p 'test_*.py'`。

### PX4 安全检查失败时

若 PX4 报 `Arming denied: Resolve system health failures first`，不要反复强制解锁。
先结束本轮飞行控制脚本，确认两机均已落地、上锁，再在新终端执行：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/restart_px4_only.sh --execute
```

此命令只重启 Pegasus 管理的两套 PX4，**不重启 Isaac 场景**；脚本会检查两机状态并等待
重启后的心跳。心跳恢复不代表健康检查通过，重启也不会清除或绕过真实的健康故障。
如果场景是在该功能加入前启动的，需先重启场景一次加载新代码；此后才能单独重启 PX4。
若重启后仍有安全检查告警，查看场景终端中的 PX4 日志，必要时再运行
`./scripts/reset_dual_px4_scene.sh` 重建场景。