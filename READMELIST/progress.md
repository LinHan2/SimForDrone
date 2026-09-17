# 项目进度

| 阶段 | 状态 | 证据/下一步 |
|---|---|---|
| P0：Isaac/PX4/WebRTC | 已运行 | Isaac Sim 5.1.0、Pegasus v5.1.0、两套 PX4 SITL 已启动；WebRTC 可查看。 |
| P1：双机观测 | 已通过最低验收 | 两机 `PoseStamped`、tracker RGB、CameraInfo、Depth 均已发布；RGB-D 内部同步已验证。 |
| P1.1：真实 Warehouse 视觉环境 | 已实现，待运行验收 | 官方 `full_warehouse.usd` 引用；重启后执行 `check_observer_rgb_scene.sh`。 |
| P2.0：起飞验证 | 已通过 | `TAKEOFF/ARM/LAND` 均接受；ROS 位姿最高 `2.515 m`、最终 `0.055 m`。 |
| P2.1：真值跟踪 | 未开始 | 先单独验证目标机，再实现真值相对位姿跟踪。 |
| P3：视觉位姿/EKF | 未开始 | 先解决图像与位姿时间基准不一致。 |

## 最近记录

- 双机 ROS 接口通过：`/target_uav_0/state/pose`、`/tracker_uav_1/state/pose` 及 tracker RGB-D 话题均可采样。
- RGB-D 样本可保存至 `logs/rgbd_samples/`；相机时间与 PX4 位姿时间未对齐，当前不得进入视觉闭环。
- 场景已从 `Curved Gridroom` 和程序化机库改为官方 Isaac `full_warehouse.usd`。它提供真实
  货架、叉车、工业材质和灯光；首次启动需缓存依赖。尚未运行验收，不能声称对 6D pose 有效。
- 场景超参数已集中到 `configs/dual_uav_hangar.yaml`；`src/simfordrone/config.py` 在启动时
  校验顶层结构。YAML 离线解析与场景代码静态编译已通过，尚未重启加载。
- P2.0 通过证据：控制 `logs/takeoff_validation/20260917-210831/` 的三项命令均接受；
  位姿 `logs/takeoff_validation/20260917-210917/` 满足 `z_max_after_ground_baseline=2.514618 m`
  与 `z_final=0.054701 m`。同轮 PX4 ULog 的峰值为 `2.459 m`，并已自动降落/解锁。
- `20260917-185323` 曾被旧验收器误判为通过：它记录的是场景启动时从 `2 m` 自然落地，
  不含与控制命令重叠的起飞过程。验收器现要求地面基线后才统计离地高度。
- `20260917-185253` 的控制日志中 ARM 被 PX4 拒绝；它发生在场景尚未稳定时，不能评价
  新推力标定。下一轮须先确认 tracker 已贴地且 PX4 显示 `Ready for takeoff!`。
- 1700 档有效复验中仅到 `z_max=0.073031 m`；2000 档确认能离地。启动时可通过
  `SIMFORDRONE_PX4_INPUT_SCALING` 单变量覆盖，下一步先延长保持时间而不改增益。

操作命令见 [运行手册](runbook.md)，整体技术路线见
[项目规划](uav_visual_tracking_project_plan.md)，两套仿真/自主框架的架构见
[Pegasus 与 Aerostack2 架构解读](pegasus_aerostack2_architecture.md)。
