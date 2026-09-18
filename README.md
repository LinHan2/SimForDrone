# SimForDrone

用于双无人机视觉跟踪的可复现实验工程。当前已通过“仿真、两套 PX4 SITL、ROS 2 观测接口”
的最低运行时验收，并已接入 **px4ctrl 控制环**（姿态+推力闭环，仿真/真机同一套控制器）；
尚未接入双机跟踪制导、视觉位姿估计或 EKF。

```text
SimForDrone/
├── scripts/       # 可执行入口：启动 Isaac Sim/Pegasus/PX4，以及 px4ctrl 控制入口
├── px4ctrl/       # ★ 唯一命令执行模块（控制环）：参数/坐标/控制器/状态机/连接/CLI
├── tracking/      # 双机跟踪制导与指标：先真值闭环，命令交给 px4ctrl
├── vision/        # RGB-D 到相对 6D 位姿与协方差，不直接控制飞行器
├── configs/       # 场景侧 YAML：双机、相机、推力增益、机库超参数
├── env/           # 项目运行环境：Isaac 内部 ROS 与系统 ROS 严格隔离
├── src/simfordrone/
│   ├── dual_uav_observation.py  # 项目拥有的双机场景定义
│   ├── industrial_hangar.py      # 官方 Isaac Warehouse USD 环境引用
│   └── pegasus_compat.py        # 不改第三方源码的兼容层
├── utils/         # 只读检查和小工具
├── READMELIST/    # 每阶段的操作、日志与验收标准
├── logs/          # 运行日志（不作为源码）
├── PegasusSimulator/ # 固定版本的第三方仿真接口
├── PX4-Autopilot/    # 固定版本的 PX4 SITL
├── Fast-Gamma/       # 上游参考实现（ROS 1 px4ctrl），只读参照，不参与运行
└── aerostack2/       # 后续控制闭环集成
```

**控制代码不得再分散**：所有飞控命令与设定点下发都必须经过 `px4ctrl/`，见其
[模块说明](px4ctrl/README.md)。

当前开发顺序为 `tracking` 真值双机跟踪（T3）→ 指标固化（T4）→ `vision` 替换真值输入
（T5）。视觉模块不得直接调用 `px4ctrl`，避免感知误差与控制误差互相掩盖。

所有项目模块变更必须更新 [项目进度](READMELIST/progress.md)；运行命令集中在
[运行手册](READMELIST/runbook.md)。

`env/activate_isaacsim_internal_ros.sh` 只能由 Isaac 启动脚本在子进程中加载；
`env/activate_system_ros2_jazzy.sh` 只能在验收或算法终端加载。二者不能在同一进程混用。

## 当前检查点：双机观测

先停止任何单独启动的 Isaac Sim 实例，然后在项目根目录启动：

```bash
./scripts/start_dual_px4_scene.sh
```

另开一个系统 ROS 2 Jazzy 终端，验证接口而非仅验证进程仍在运行：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./utils/check_dual_uav_observation.sh
```

预期有两台 PX4（`vehicle_id=0,1`）及以下 ROS 2 主题：目标机/跟踪机位姿，跟踪机前视 RGB、相机内参和深度图。详见 [运行手册](READMELIST/runbook.md)。

## 后续边界

观测主题与时间戳验证完成后，才添加一个只读取这些主题的真值/几何跟踪基线；随后再以视觉 6D 位姿替换真值，并将其作为带协方差的 EKF 量测。控制闭环不应绕过这些验收步骤。

单机飞行验证与站位保持命令见 [运行手册](READMELIST/runbook.md) 的 px4ctrl 一节。

**T3 之前的已知阻塞**：各机 PX4 EKF 的局部原点互相独立，实测同一高度的两机局部 z 相差
约 1.1 cm，因此双机相对位置不能用两条 `LOCAL_POSITION_NED` 相减，必须改用共享坐标系
（ROS 真值或全局原点/GPS）。
