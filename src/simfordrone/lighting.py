"""场景光照装置：按 YAML 显式建立环境光、太阳光与室内补光。

官方环境资产自带的灯光往往偏暗，直接用于图像闭环会得到对比度低、边缘密度不足的
画面，无法支撑后续视觉 6D 位姿估计。本模块把光照变成**可复现、可调、可审计**的
配置项，而不是依赖某个 USD 里碰巧存在的灯。

分工：``industrial_hangar`` 只负责“场景里有什么物体”，本模块只负责“怎么照亮它们”。
"""

from __future__ import annotations

from typing import Any, Mapping


#: 各段允许的键。未知键直接报错：拼错的 ``intensity`` 若被忽略，
#: 结果只是画面继续偏暗，排查成本很高。
SECTION_KEYS: dict[str, tuple[str, ...]] = {
    "lighting": ("root", "dome", "sun", "rect_lights"),
    "dome": ("enabled", "intensity", "color"),
    "sun": ("enabled", "intensity", "color", "angle_deg", "elevation_deg", "azimuth_deg"),
    "rect_light": (
        "prim_path",
        "enabled",
        "intensity",
        "color",
        "size",
        "position",
        "rotate_deg",
    ),
}


def _check_keys(section: Mapping[str, Any], name: str) -> None:
    allowed = SECTION_KEYS[name]
    unknown = sorted(set(section) - set(allowed))
    if unknown:
        raise ValueError(f"lighting.{name} 含未知参数 {unknown}；允许: {list(allowed)}")


def _positive(value: Any, name: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise ValueError(f"{name} 必须为正数，当前为 {value!r}")
    return number


def _color(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} 必须是 3 个分量的颜色，当前为 {value!r}")
    triple = tuple(float(component) for component in value)
    if any(component < 0.0 for component in triple):
        raise ValueError(f"{name} 的颜色分量不能为负: {value!r}")
    return triple  # type: ignore[return-value]


def _vector(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} 必须是 3 个分量，当前为 {value!r}")
    return tuple(float(component) for component in value)  # type: ignore[return-value]


def _pair(value: Any, name: str) -> tuple[float, float]:
    """2 分量正数，用于补光面板的宽高 ``(width, height)``。"""

    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{name} 必须是 2 个分量 (宽, 高)，当前为 {value!r}")
    pair = (float(value[0]), float(value[1]))
    if pair[0] <= 0.0 or pair[1] <= 0.0:
        raise ValueError(f"{name} 必须为正数，当前为 {value!r}")
    return pair


def validate_lighting_config(config: Mapping[str, Any]) -> None:
    """在创建任何 prim 之前完成校验，让配置错误早于渲染暴露。"""

    if not isinstance(config, Mapping):
        raise TypeError("environment.lighting 必须是映射")
    _check_keys(config, "lighting")

    enabled: list[str] = []
    for name in ("dome", "sun"):
        section = config.get(name)
        if section is None:
            continue
        if not isinstance(section, Mapping):
            raise TypeError(f"lighting.{name} 必须是映射")
        _check_keys(section, name)
        if not bool(section.get("enabled", False)):
            continue
        if "intensity" not in section:
            raise ValueError(f"lighting.{name} 已启用但缺少 intensity")
        _positive(section["intensity"], f"lighting.{name}.intensity")
        if "color" in section:
            _color(section["color"], f"lighting.{name}.color")
        enabled.append(name)

    rect_lights = config.get("rect_lights") or []
    if not isinstance(rect_lights, (list, tuple)):
        raise TypeError("lighting.rect_lights 必须是列表")
    for index, entry in enumerate(rect_lights):
        name = f"rect_light[{index}]"
        if not isinstance(entry, Mapping):
            raise TypeError(f"lighting.{name} 必须是映射")
        _check_keys(entry, "rect_light")
        if not bool(entry.get("enabled", True)):
            continue
        if not entry.get("prim_path"):
            raise ValueError(f"lighting.{name} 缺少 prim_path")
        if "intensity" not in entry:
            raise ValueError(f"lighting.{name} 缺少 intensity")
        _positive(entry["intensity"], f"lighting.{name}.intensity")
        if "color" in entry:
            _color(entry["color"], f"lighting.{name}.color")
        if "size" in entry:
            _pair(entry["size"], f"lighting.{name}.size")
        if "position" in entry:
            _vector(entry["position"], f"lighting.{name}.position")
        if "rotate_deg" in entry:
            _vector(entry["rotate_deg"], f"lighting.{name}.rotate_deg")
        enabled.append(name)

    if not enabled:
        raise ValueError("environment.lighting 未启用任何光源，图像闭环需要明确的光照")


class LightRig:
    """把 YAML 光照配置实例化为 Isaac 光源 prim。

    光源建立在 ``/World`` 下的独立子树中，不触碰环境资产内部节点，因此同一套光照
    可以配任意 ``preset``，也便于按实验对比“有无补光”。
    """

    def __init__(self, stage, config: Mapping[str, Any]) -> None:
        self.stage = stage
        self.config = config

    def build(self) -> list[str]:
        """建立所有启用的光源，返回其 prim 路径（用于启动日志取证）。"""

        from pxr import Gf, UsdGeom, UsdLux

        validate_lighting_config(self.config)
        root = str(self.config.get("root") or "/World/lights")
        if not root.startswith("/World/"):
            raise ValueError("lighting.root 必须位于 /World/ 下")
        if self.stage.GetPrimAtPath(root).IsValid():
            raise RuntimeError(f"光照根 prim 已存在，拒绝重复加载: {root}")

        created: list[str] = []

        dome_config = self.config.get("dome")
        if isinstance(dome_config, Mapping) and dome_config.get("enabled", False):
            path = f"{root}/dome"
            dome = UsdLux.DomeLight.Define(self.stage, path)
            dome.CreateIntensityAttr(float(dome_config["intensity"]))
            if "color" in dome_config:
                dome.CreateColorAttr(Gf.Vec3f(*_color(dome_config["color"], "lighting.dome.color")))
            created.append(path)

        sun_config = self.config.get("sun")
        if isinstance(sun_config, Mapping) and sun_config.get("enabled", False):
            path = f"{root}/sun"
            sun = UsdLux.DistantLight.Define(self.stage, path)
            sun.CreateIntensityAttr(float(sun_config["intensity"]))
            if "color" in sun_config:
                sun.CreateColorAttr(Gf.Vec3f(*_color(sun_config["color"], "lighting.sun.color")))
            if "angle_deg" in sun_config:
                sun.CreateAngleAttr(float(sun_config["angle_deg"]))
            # DistantLight 沿自身 -Z 方向照射：无旋转时垂直向下。
            # 绕 X 轴旋转 (90 - 仰角) 把光线从垂直压到目标仰角，再绕 Z 轴给出方位角。
            elevation = float(sun_config.get("elevation_deg", 90.0))
            if not 0.0 < elevation <= 90.0:
                raise ValueError("lighting.sun.elevation_deg 必须在 (0, 90] 内")
            azimuth = float(sun_config.get("azimuth_deg", 0.0))
            UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
                Gf.Vec3f(90.0 - elevation, 0.0, azimuth)
            )
            created.append(path)

        for index, entry in enumerate(self.config.get("rect_lights") or []):
            if not bool(entry.get("enabled", True)):
                continue
            path = str(entry["prim_path"])
            if not path.startswith("/World/"):
                raise ValueError(f"lighting.rect_light[{index}].prim_path 必须位于 /World/ 下")
            if self.stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(f"补光 prim 已存在: {path}")
            light = UsdLux.RectLight.Define(self.stage, path)
            light.CreateIntensityAttr(float(entry["intensity"]))
            if "color" in entry:
                light.CreateColorAttr(
                    Gf.Vec3f(*_color(entry["color"], f"lighting.rect_light[{index}].color"))
                )
            if "size" in entry:
                width, height = _pair(entry["size"], f"lighting.rect_light[{index}].size")
                light.CreateWidthAttr(width)
                light.CreateHeightAttr(height)
            xform = UsdGeom.Xformable(light.GetPrim())
            if "position" in entry:
                xform.AddTranslateOp().Set(Gf.Vec3f(*_vector(entry["position"], f"lighting.rect_light[{index}].position")))
            if "rotate_deg" in entry:
                xform.AddRotateXYZOp().Set(
                    Gf.Vec3f(*_vector(entry["rotate_deg"], f"lighting.rect_light[{index}].rotate_deg"))
                )
            created.append(path)

        return created
