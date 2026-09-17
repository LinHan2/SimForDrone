"""读取 SimForDrone 自有 YAML 配置，不读取或修改第三方 Pegasus 配置。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "dual_uav_hangar.yaml"


def load_dual_uav_config() -> dict[str, Any]:
    """加载场景参数；可用 SIMFORDRONE_CONFIG 指向同结构的实验配置。"""

    config_path = Path(os.environ.get("SIMFORDRONE_CONFIG", DEFAULT_CONFIG)).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f"找不到 SimForDrone YAML 配置: {config_path}")
    with config_path.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError(f"YAML 顶层必须是映射: {config_path}")
    for section in ("px4", "vehicles", "observer_camera", "environment", "viewport"):
        if section not in config:
            raise ValueError(f"YAML 缺少必需段落 {section}: {config_path}")
    return config
