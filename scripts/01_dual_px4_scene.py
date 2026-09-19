#!/usr/bin/env python3
"""双机场景的轻量 Isaac Sim 入口。

运行时环境固定在 ``start_dual_px4_scene.sh``；场景构建在
``src/simfordrone``，使其能独立于启动命令继续演进和测试。

三种显示模式
------------
==========  ==========================================================
本地桌面     ``SIMFORDRONE_ISAAC_GUI=1``，出现原生窗口与面板
带 UI 流    ``SIMFORDRONE_ISAAC_UI=1``，headless + 流式应用，
            HUD 与属性面板随 WebRTC 视频流一起发送
纯画面流    两者都为 0，只流渲染画面（无法叠加 HUD）
==========  ==========================================================

带 UI 的流式必须用官方 ``isaacsim.exp.full.streaming.kit``：该应用显式设置
``hideUi = false``，是“无窗口但 UI 随视频流发送”的官方方案。
仅用 ``base.python.kit`` 时即使解除隐藏，也没有可合成的窗口。
"""

import os
from pathlib import Path
import sys

from isaacsim import SimulationApp

gui_enabled = os.environ.get("SIMFORDRONE_ISAAC_GUI", "0") == "1"
ui_enabled = os.environ.get("SIMFORDRONE_ISAAC_UI", "0") == "1"
# 由启动脚本给出的流式应用路径；为空时使用 Isaac 默认应用。
experience = os.environ.get("SIMFORDRONE_ISAAC_EXPERIENCE", "")

launch_config = {
    "headless": not gui_enabled,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting",
    # 本地桌面与带 UI 流都需要绘制 UI；纯画面流则隐藏，
    # 避免在没有可合成窗口的情况下初始化 UI 扩展而报错。
    "hide_ui": not (gui_enabled or ui_enabled),
}

if experience and not gui_enabled:
    simulation_app = SimulationApp(launch_config, experience=experience)
else:
    simulation_app = SimulationApp(launch_config)

from isaacsim.core.utils.extensions import enable_extension

# 使用独立脚本的 WebRTC 扩展。GUI 必须显式启用，因为无窗口服务器在 UI
# 初始化阶段会失败。
enable_extension("omni.services.livestream.nvcf")
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_SOURCE_ROOT = PROJECT_ROOT / "src"
PEGASUS_EXTENSION_ROOT = (
    PROJECT_ROOT / "PegasusSimulator" / "extensions" / "pegasus.simulator"
)

for import_root in (PROJECT_SOURCE_ROOT, PEGASUS_EXTENSION_ROOT):
    if not import_root.is_dir():
        raise RuntimeError(f"Required source directory not found: {import_root}")
    sys.path.insert(0, str(import_root))

from simfordrone.dual_uav_observation import DualUavObservationApp


def main() -> None:
    DualUavObservationApp(simulation_app).run()


if __name__ == "__main__":
    main()
