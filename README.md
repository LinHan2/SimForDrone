# SimForDrone

用于双无人机视觉跟踪的可复现实验工程。当前已通过“仿真、两套 PX4 SITL 与 ROS 2 观测接口”的最低运行时验收；尚未接入跟踪控制、视觉位姿估计或 EKF。

```text
SimForDrone/
├── scripts/       # 可执行入口：启动 Isaac Sim、Pegasus、PX4
├── configs/       # 项目自有 YAML：双机、相机、推力、机库与灯光超参数
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
└── aerostack2/       # 后续控制闭环集成
```

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

P2.0 的单机起飞验证命令与安全条件见 [运行手册](READMELIST/runbook.md)。
