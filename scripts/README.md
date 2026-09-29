# 脚本入口

实验主流程见[核心实验流程](../READMELIST/runbook.md)。标定、观测、诊断和离线测试见
[测试与诊断脚本](../READMELIST/test_scripts.md)。

| 脚本 | 用途 |
|---|---|
| `start_dual_px4_scene.sh --onboard` | 启动 Isaac、Pegasus 和双 PX4 SITL，WebRTC 实时显示 tracker 机载前视相机 |
| `reset_dual_px4_scene.sh` | 停止并重启场景，清除 PX4 SITL health/EKF 残留状态 |
| `restart_px4_only.sh --execute` | 两机落地上锁后请求当前 Isaac 场景重启 Pegasus 管理的两套 PX4；不重建场景、不绕过健康检查 |
| `run_target_waypoints.sh` | 独占 `14540`，控制 target 并发布共享状态 |
| `run_tracker.sh` | 独占 `14541`，订阅 target 状态并控制 tracker |
| `run_shadow_relative_ekf.sh` | 只读 target PX4 姿态与 tracker RGB-D，验证相对 EKF，不发送飞控指令 |
| `run_target_shadow_gated.sh` | 场景就绪后，一键启动 target、truth tracker、FOV 门与只读 shadow EKF，通过门控才放行航迹（须显式 `--execute`） |
| `run_px4ctrl.sh` | `px4ctrl` 的单机控制、站位保持与标定入口 |
| `check_observation.sh [--rgbd]` | 只读 ROS 2 话题验收；`--rgbd` 额外采样并检查前视图像 |
| `show_latest_tracking_result.sh` | 汇总最新 tracker 运行结果 |

`scripts/env/` 下的三个环境 profile 只能通过 `source` 加载，不能直接执行。