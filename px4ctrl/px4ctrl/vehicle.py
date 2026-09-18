"""载具身份与飞控端点。

把"角色 → MAVLink 端点 / system id / ROS 命名空间"集中在一处，避免端口与 system id
被复制到多个脚本后失去一致性——"命令发给了错误的无人机"是双机系统最危险也最常见的错误。

仿真端点来自 PX4 SITL 的固定规则（``ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink``）：
offboard 远端端口为 ``14540 + instance``，system id 为 ``instance + 1``。真机不使用这些
端点，而是通过 :class:`~px4ctrl.params.LinkParams` 给出实际连接串。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleRole:
    """单个载具的仿真身份与 ROS 话题前缀。"""

    name: str
    vehicle_id: int
    mavlink_port: int
    system_id: int
    ros_namespace: str

    @property
    def sim_connection(self) -> str:
        """仿真用的 MAVLink 连接串（监听 PX4 主动发来的 heartbeat）。"""

        return f"udpin:0.0.0.0:{self.mavlink_port}"

    @property
    def pose_topic(self) -> str:
        """ROS 2 真值位姿话题；命名空间末尾自带下划线。"""

        return f"/{self.ros_namespace}{self.vehicle_id}/state/pose"


_ROLES: dict[str, VehicleRole] = {
    "target": VehicleRole("target", 0, 14540, 1, "target_uav_"),
    "tracker": VehicleRole("tracker", 1, 14541, 2, "tracker_uav_"),
}


def resolve_role(name: str) -> VehicleRole:
    """按角色名取端点定义；未知角色立即报错，不做静默回退。"""

    try:
        return _ROLES[name]
    except KeyError:
        raise KeyError(
            f"未知角色 {name!r}；可用角色: {', '.join(available_roles())}"
        ) from None


def available_roles() -> tuple[str, ...]:
    return tuple(sorted(_ROLES))
