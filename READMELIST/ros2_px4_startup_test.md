# ROS 2 与 PX4 启动、测试流程

统一入口：

```bash
cd /data/disk2/home/hl/research/SimForDrone
./scripts/start_ros2_px4.sh <command>
```

## 首次构建

```bash
./scripts/start_ros2_px4.sh build
```

该步骤在 `../IsaacDrone/ros2_ws` 中构建：

- `px4_msgs`：ROS 2解释PX4消息所需的接口定义；
- `Micro-XRCE-DDS-Agent v2.4.3`：PX4 uORB与ROS 2 DDS之间的中间件Agent。

## 日常启动

服务器批量/接口开发默认使用Headless：

```bash
./scripts/start_ros2_px4.sh start
```

脚本严格按以下顺序启动：

1. Micro XRCE-DDS Agent监听UDP 8888；
2. 确认8888已经监听；
3. 启动PX4 SITL和Gazebo `gz_x500`；
4. PX4内置的`uxrce_dds_client`连接Agent；
5. Agent在ROS 2 DDS域中创建`/fmu/in/*`和`/fmu/out/*`。

通过ToDesk临时查看Gazebo GUI时：

```bash
./scripts/start_ros2_px4.sh start --gui
```

GUI模式会强制选择NVIDIA GLX，避免回退到Mesa `llvmpipe`。正式运行仍推荐Headless。

## 状态检查与通信验收

```bash
./scripts/start_ros2_px4.sh status
./scripts/start_ros2_px4.sh test
```

`test`自动兼容PX4当前的版本化话题，例如：

```text
/fmu/out/vehicle_status_v4
/fmu/out/vehicle_local_position_v1
```

验收通过条件：

- Agent监听UDP 8888；
- PX4与Gazebo进程存在；
- ROS 2发现`/fmu`话题；
- 能解码一条`VehicleStatus`；
- 能解码一条`VehicleLocalPosition`；
- PX4输入端订阅了`/fmu/in/offboard_control_mode`。

最后一项只证明ROS 2已经发现PX4输入Reader。后续还需由控制节点持续发布Offboard心跳、轨迹参考和VehicleCommand，才能完成实际控制验收。

## 观察终端

```bash
./scripts/start_ros2_px4.sh attach agent
./scripts/start_ros2_px4.sh attach px4
```

从tmux退出但不终止进程：按 `Ctrl-b`，再按 `d`。

查看保存日志：

```bash
./scripts/start_ros2_px4.sh logs agent
./scripts/start_ros2_px4.sh logs px4
```

## 停止

```bash
./scripts/start_ros2_px4.sh stop
```

不要直接关闭SSH窗口替代停止流程。脚本会先向PX4/Gazebo发送`Ctrl-C`，再停止Agent。

## 当前阶段边界

此流程只负责：

```text
Gazebo/PX4 <-> uXRCE-DDS Agent <-> ROS 2
```

当前不启动双机、位姿网络、状态估计器或Isaac Sim。下一阶段是在这个已验证链路上添加最小ROS 2 Offboard控制节点。
