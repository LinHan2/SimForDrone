"""固定版本 Pegasus 的项目兼容层。

项目特有行为放在这里，不直接修改 ``PegasusSimulator`` 第三方目录，
以便版本更新时能清楚区分上游变化与项目变化。
"""

from __future__ import annotations

def enable_rgbd_ros2_depth_marker() -> None:
    """让 Pegasus 现有 ROS 2 后端发布相机深度图。
    ``MonocularCamera(depth=True)`` 已经创建 Isaac Sim 的深度缓冲区，
    但 Pegasus 的 ROS 2 后端仅在传感器返回字典含 ``"depth"`` 标记时
    才挂载深度 writer。本函数只补充该标记，不改动第三方源码。
    """
    from pegasus.simulator.logic.graphical_sensors.monocular_camera import (
        MonocularCamera,
    )

    if getattr(MonocularCamera, "_simfordrone_depth_marker_enabled", False):
        return

    # 保存上游带频率限制的 update 包装器；不重新实现相机采样逻辑。
    upstream_update = MonocularCamera.update

    def update_with_depth_marker(self, *args, **kwargs):
        # 上游会在最初 100 个渲染回调直接返回 None。该延迟与相机初始化无关，
        # 却会阻止 ROS 2 writer 创建，尤其在 Warehouse 的 headless 首帧阶段明显。
        self.counter = max(getattr(self, "counter", 100), 100)
        data = upstream_update(self, *args, **kwargs)
        if isinstance(data, dict) and getattr(self, "_depth", False):
            data["depth"] = True
        return data

    # 仅对当前 Isaac Sim 进程生效；同一进程重复调用不会重复包装。
    MonocularCamera.update = update_with_depth_marker
    MonocularCamera._simfordrone_depth_marker_enabled = True
