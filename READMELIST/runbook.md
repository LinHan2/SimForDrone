
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

当前编排入口因尚未接入真实像素与深度闭环而拒绝解锁。以下仅记录待验证的
低速参数组合，不是可执行的飞行步骤：

```bash
./scripts/run_target_shadow_gated.sh --execute --truth-baseline -- \
  --trajectory circle --trajectory-radius 10 --trajectory-cycles 2 \
  --max-speed 5.0 --max-accel 3.0
```
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