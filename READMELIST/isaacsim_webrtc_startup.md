# Isaac Sim 5.1.0 与 Windows WebRTC 启动流程

本文只记录第一阶段：启动 Isaac Sim 流式服务，并从本地 Windows 连接查看画面。
双无人机场景、Pegasus 配置和 ROS 2 控制节点在此阶段之后单独验证。

## 1. 服务器端启动 Isaac Sim

服务器路径：`/data/disk2/home/hl/isaacsim-5.1.0`。

Isaac Sim 必须在干净环境中启动，不能在同一进程中混入 Conda、ROS 2 或外部 CUDA 的库：

```bash
cd /data/disk2/home/hl/isaacsim-5.1.0

env -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    -u CUDA_HOME \
    -u CONDA_PREFIX \
    -u CONDA_DEFAULT_ENV \
    -u CONDA_PROMPT_MODIFIER \
    -u CONDA_SHLVL \
    -u ROS_DISTRO \
    -u RMW_IMPLEMENTATION \
    -u AMENT_PREFIX_PATH \
    PYTHONNOUSERSITE=1 \
    ./isaac-sim.streaming.sh \
    --/app/livestream/publicEndpointAddress=10.134.88.113 \
    --/app/livestream/port=49100 \
    2>&1 | tee /tmp/isaacsim-streaming-$(date +%Y%m%d-%H%M%S).log
```

启动成功的最低验收标志是终端出现：

```text
Isaac Sim Full Streaming App is loaded.
```

服务器端检查监听端口：

```bash
ss -lntup | grep -E '49100|47998'
```

WebRTC 默认需要 `TCP 49100`（信令）和 `UDP 47998`（媒体）。两者都必须从校园网入口转发到服务器 `192.168.50.207`：

```text
10.134.88.113:49100/TCP  -> 192.168.50.207:49100
10.134.88.113:47998/UDP  -> 192.168.50.207:47998
```

## 2. Windows 本地连接前测试

在 Windows PowerShell 执行：

```powershell
Test-NetConnection 10.134.88.113 -Port 49100
```

只有下面结果为 `True`，TCP 信令才是可达的：

```text
TcpTestSucceeded : True
```

`Test-NetConnection` 不能可靠验证 UDP；UDP 是否可达需要用 WebRTC 实际连接确认。

## 3. Windows WebRTC 客户端

启动本地下载的 **Isaac Sim WebRTC Streaming Client**，填写：

```text
Server / IP：10.134.88.113
Port：49100
```

然后点击 **Connect**。Windows 防火墙需要允许 WebRTC Streaming Client；如果 TCP 测试通过但画面黑屏，优先检查 UDP `47998` 转发。

NVIDIA 官方说明：
<https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html>

## 4. 与 ROS 2 的终端隔离

Isaac Sim 使用上面的干净终端启动。ROS 2 在另一个终端单独加载：

```bash
source /opt/ros/jazzy/setup.bash
```

不要在 Isaac Sim 启动终端中再次 `conda activate` 或设置外部 `LD_LIBRARY_PATH`。

## 5. 本阶段完成后

确认以下三项后，再开始构建双无人机场景：

- Isaac Sim 日志出现 `Isaac Sim Full Streaming App is loaded.`；
- Windows TCP 49100 测试为 `True`；
- WebRTC 客户端能看到 Isaac Sim 画面。

下一阶段才添加两架无人机、跟随机相机和 Pegasus 场景，不在本文件中自动启动双机或控制节点。
