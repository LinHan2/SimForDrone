#!/usr/bin/env python3
"""双机场景的轻量 Isaac Sim 入口。

运行时环境固定在 ``start_dual_px4_scene.sh``；场景构建在
``src/simfordrone``，使其能独立于启动命令继续演进和测试。
"""

from pathlib import Path
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": True,
        "hide_ui": True,
        "width": 1280,
        "height": 720,
        "renderer": "RayTracedLighting",
    }
)

from isaacsim.core.utils.extensions import enable_extension

# 使用独立脚本的 WebRTC 扩展。不能在 SimulationApp 内加载完整 GUI
# experience：服务器无窗口环境下曾在 UI 初始化阶段崩溃。
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
