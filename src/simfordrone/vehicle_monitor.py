"""Isaac Sim UI panel for live dual-UAV state inspection."""

from __future__ import annotations

import math

import omni.ui as ui


class VehicleMonitorWindow:
    """Display stable vehicle identity plus live world-ENU state.

    只读展示：不参与控制，也不修改任何 USD 属性，因此不影响实验可复现性。
    静态身份信息（stage 路径 / 序号 / PX4 端点）在构建时写死到界面，
    动态位置和速度每帧从 vehicle.state 读取，便于对照"命令飞给了哪台机"。
    """

    def __init__(self, vehicles: dict[str, dict]) -> None:
        # 保存载具注册表的引用而非拷贝：载具句柄在场景生命周期内不变。
        self._vehicles = vehicles
        # role -> omni.ui 的字符串模型；每帧只更新模型值，避免重建控件。
        self._models: dict[str, object] = {}
        self.window = ui.Window("SimForDrone Vehicles", width=460, height=300, visible=True)
        # 与 Pegasus 自带窗口一致，停靠到 Property 面板旁边，方便同时查看 USD 属性。
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Live world state (ENU)", height=24)
                for role, entry in vehicles.items():
                    with ui.CollapsableFrame(role, collapsed=False):
                        with ui.VStack(spacing=3):
                            ui.Label(f"Stage: {entry['stage_path']}")
                            ui.Label(
                                f"vehicle_id={entry['vehicle_id']}  PX4 system_id={entry['system_id']}  "
                                f"MAVLink={entry['mavlink_port']}"
                            )
                            # 占位文本用于提示用户该行会持续刷新，而不是静态标签。
                            label = ui.Label("position=(waiting)  velocity=(waiting)", word_wrap=True)
                            self._models[role] = label.model

    def update(self) -> None:
        """每帧调用：把最新的世界系位置/速度写入已存在的标签模型。"""

        for role, entry in self._vehicles.items():
            # vehicle.state 由 Pegasus 在物理步进中更新，坐标系为 Isaac 世界 ENU。
            state = entry["vehicle"].state
            position = state.position
            velocity = state.linear_velocity
            # 速度模长便于一眼判断"是否真的在动"，无需心算三个分量。
            speed = math.sqrt(sum(float(value) ** 2 for value in velocity))
            self._models[role].set_value(
                f"position=({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}) m\n"
                f"velocity=({velocity[0]:+.3f}, {velocity[1]:+.3f}, {velocity[2]:+.3f}) m/s  "
                f"speed={speed:.3f} m/s"
            )
