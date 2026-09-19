"""Isaac Sim HUD：把双机实时属性叠加到 WebRTC 视频流上。

为什么用 omni.ui 窗口而不是终端打印：流式模式下操作员盯的是视频画面，
把状态画在画面上才能与实际飞行画面一一对应。前提是场景以“带 UI 的流”启动
（``isaacsim.exp.full.streaming.kit``，``hideUi = false``），
否则 UI 不会进入视频流。

只读展示：不参与控制、不修改任何 USD 属性，因此不影响实验可复现性。
"""

from __future__ import annotations

import math
from typing import Any

import omni.ui as ui

from simfordrone.view_control import HELP_LINES


class VehicleHudWindow:
    """在画面上叠加两机位置/速度、身份信息与当前视角。

    静态身份信息（stage 路径 / 序号 / PX4 端点）构建时写入，
    动态量与视角说明在 :meth:`update` 中刷新，便于对照“命令飞给了哪台机”。
    """

    def __init__(self, vehicles: dict[str, dict], view_control) -> None:
        # 保存载具注册表的引用而非拷贝：载具句柄在场景生命周期内不变。
        self._vehicles = vehicles
        self._view_control = view_control
        # role -> omni.ui 的字符串模型；每帧只更新模型值，避免重建控件。
        # 类型用 Any：omni.ui 的模型类在编辑器环境不可解析，写死会引入假报错。
        self._models: dict[str, Any] = {}
        self.window = ui.Window("SimForDrone HUD", width=560, height=380, visible=True)
        # 停靠到 Property 面板旁边，方便同时查看 USD 属性与实时状态。
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label("World state (ENU)", height=24)
                for role, entry in vehicles.items():
                    with ui.CollapsableFrame(role, collapsed=False):
                        with ui.VStack(spacing=3):
                            ui.Label(f"Stage: {entry['stage_path']}")
                            ui.Label(
                                f"vehicle_id={entry['vehicle_id']}  PX4 system_id={entry['system_id']}  "
                                f"MAVLink={entry['mavlink_port']}"
                            )
                            # 实时数值必须走模型：omni.ui 的 Label 只接受静态字符串，
                            # **没有 model 属性**（误用会抛 AttributeError，并连带把整个
                            # 场景拖到退出阶段的段错误）。因此用 SimpleStringModel +
                            # 只读多行 StringField 来承载每帧刷新的文本。
                            model = ui.SimpleStringModel("position=(waiting)")
                            ui.StringField(model=model, read_only=True, multiline=True, height=46)
                            self._models[role] = model
                # 视角行同理：切相机后必须能在画面上看到当前处于哪个视角。
                self._view_model = ui.SimpleStringModel("view=(waiting)")
                ui.StringField(model=self._view_model, read_only=True, height=20)
                for line in HELP_LINES:
                    ui.Label(line, height=16)

    def update(self) -> None:
        """每帧调用：把最新的世界系位置/速度写入已存在的标签模型。"""

        for role, entry in self._vehicles.items():
            # vehicle.state 由 Pegasus 在物理步进中更新，坐标系为 Isaac 世界 ENU。
            state = entry["vehicle"].state
            position = state.position
            velocity = state.linear_velocity
            # 速度模长便于一眼判断“是否真的在动”，无需心算三个分量。
            speed = math.sqrt(sum(float(value) ** 2 for value in velocity))
            self._models[role].set_value(
                f"position=({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}) m\n"
                f"velocity=({velocity[0]:+.3f}, {velocity[1]:+.3f}, {velocity[2]:+.3f}) m/s  "
                f"speed={speed:.3f} m/s"
            )
        self._view_model.set_value(f"view = {self._view_control.describe()}")
