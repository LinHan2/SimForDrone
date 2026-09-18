"""px4ctrl 参数层。

对应上游 ``Fast-Gamma/src/realflight_modules/px4ctrl/src/PX4CtrlParam.h``：把 YAML 变成
**不可变**的参数对象，并在加载时校验，避免非法参数在飞行中才暴露。

仿真与真机的差异被压缩到 :class:`LinkParams` 一处：控制器、状态机与制导逻辑对两者
完全无感知，这是"同一套控制器兼容仿真与真机"的实现方式。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class ParamError(ValueError):
    """参数缺失、类型错误或取值越界时抛出，保证失败发生在起飞之前。"""


@dataclass(frozen=True)
class TakeoffLandParams:
    """``auto_takeoff_land`` 段。"""

    enable: bool = True
    enable_auto_arm: bool = True
    no_rc: bool = False
    takeoff_height: float = 1.0
    takeoff_land_speed: float = 0.5


@dataclass(frozen=True)
class ThrustModelParams:
    """``thrust_model`` 段：把归一化推力信号 u(0~1) 映射为真实推力。"""

    accurate: bool = True
    print_value: bool = False
    k1: float = 0.7583
    k2: float = 1.6942
    k3: float = 0.6786
    hover_percentage: float = 0.30


@dataclass(frozen=True)
class GainParams:
    """``gain`` 段：串级 PID 增益，命名与上游一致以便对照调参。"""

    kp0: float = 6.0
    kp1: float = 6.0
    kp2: float = 5.0
    kv0: float = 4.0
    kv1: float = 4.0
    kv2: float = 5.0
    kang_r: float = 20.0
    kang_p: float = 20.0
    kang_y: float = 20.0


@dataclass(frozen=True)
class TimeoutParams:
    """``msg_timeout`` 段：各输入的新鲜度门限（秒）。

    任何输入超时都必须触发状态机降级，而不是继续用陈旧数据控制飞机。
    """

    odom: float = 0.5
    rc: float = 0.5
    cmd: float = 0.5
    imu: float = 0.5
    bat: float = 0.5


@dataclass(frozen=True)
class LinkParams:
    """飞控连接层参数——**仿真与真机的唯一差异点**。

    ``connection`` 采用 pymavlink 连接串：

    - 仿真：``udpin:0.0.0.0:14540``（PX4 SITL 的 offboard 远端端口）
    - 真机：``serial:/dev/ttyACM0:921600`` 或 ``udpout:192.168.1.10:14550``

    控制器在任何一侧都不需要改动。
    """

    connection: str
    target_system: int
    target_component: int = 1
    heartbeat_timeout: float = 15.0


@dataclass(frozen=True)
class SharedFrameParams:
    """``shared_frame`` 段：多机共享位置坐标系。

    各机 PX4 EKF 的 ``LOCAL_POSITION_NED`` 原点由**各自的**起飞点建立，因此两机的
    local 位置不在同一系里。启用本段后，位置改由 ``GLOBAL_POSITION_INT`` 相对同一个
    地理原点换算得到，两机即可比较。

    ``origin_*`` 必须是**所有载具共用**的原点：仿真里取 Pegasus 的世界原点
    （``PegasusSimulator/.../config/configs.yaml`` 的 ``global_coordinates``）；
    真机上取测量好的 home 点或 RTK 基准。用错原点不会破坏**相对**几何，但会让整组
    坐标整体平移。
    """

    enabled: bool = False
    origin_latitude: float = 0.0
    origin_longitude: float = 0.0
    origin_altitude: float = 0.0
    #: 反解所用的地球模型半径。``None``/0 表示用 WGS-84 椭球（**真机的正确选择**）。
    #: 仿真里应填 Pegasus 的 ``EARTH_RADIUS = 6353000.0``：它的重投影是球面，与椭球
    #: 不一致会带来约 0.5% 的系统性尺度差（5 m 基线约 2.6 cm），那属于人为误差。
    earth_radius: float | None = None


@dataclass(frozen=True)
class TaskDefaults:
    """``tasks`` 段：任务级默认值。

    集中放在 YAML 里，日常运行只需 ``--role`` / ``--execute``，不必重复书写高度、
    时长与频率；命令行同名参数仍可覆盖。
    """

    altitude: float = 2.0
    hold_seconds: float = 30.0
    rate_hz: float = 20.0
    ready_timeout: float = 60.0
    landing_timeout: float = 60.0
    disarm_timeout: float = 15.0
    offset_north: float = 0.0
    offset_east: float = 0.0


@dataclass(frozen=True)
class Params:
    """完整参数集合。"""

    mass: float
    gra: float
    ctrl_freq_max: float
    use_bodyrate_ctrl: bool
    max_angle: float
    max_manual_vel: float
    takeoff_land: TakeoffLandParams
    thrust_model: ThrustModelParams
    gain: GainParams
    timeouts: TimeoutParams
    link: LinkParams
    tasks: TaskDefaults
    shared_frame: SharedFrameParams

    @property
    def max_angle_rad(self) -> float:
        """倾角限幅（弧度）；负值表示不限制，与上游语义一致。"""

        import math

        return math.inf if self.max_angle < 0.0 else math.radians(self.max_angle)


def _require(section: dict, key: str) -> object:
    if key not in section:
        raise ParamError(f"缺少必需参数: {key}")
    return section[key]


def _positive(name: str, value: float) -> float:
    if value <= 0.0:
        raise ParamError(f"{name} 必须为正数，实际为 {value}")
    return float(value)


def load_params(path: str | Path) -> Params:
    """从 YAML 加载并校验参数。

    校验刻意放在加载期：任何越界值都应在起飞前失败，而不是在飞行中表现为姿态异常。
    """

    config_path = Path(path)
    if not config_path.is_file():
        raise ParamError(f"参数文件不存在: {config_path}")

    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ParamError(f"参数文件顶层必须是映射: {config_path}")

    tl = dict(raw.get("auto_takeoff_land") or {})
    tm = dict(raw.get("thrust_model") or {})
    gain = dict(raw.get("gain") or {})
    tmo = dict(raw.get("msg_timeout") or {})
    task = dict(raw.get("tasks") or {})
    shared = dict(raw.get("shared_frame") or {})
    link = dict(_require(raw, "link"))

    params = Params(
        mass=_positive("mass", float(_require(raw, "mass"))),
        gra=_positive("gra", float(raw.get("gra", 9.81))),
        ctrl_freq_max=_positive("ctrl_freq_max", float(raw.get("ctrl_freq_max", 100.0))),
        use_bodyrate_ctrl=bool(raw.get("use_bodyrate_ctrl", False)),
        max_angle=float(raw.get("max_angle", 15.0)),
        max_manual_vel=float(raw.get("max_manual_vel", 2.5)),
        takeoff_land=TakeoffLandParams(
            enable=bool(tl.get("enable", True)),
            enable_auto_arm=bool(tl.get("enable_auto_arm", True)),
            no_rc=bool(tl.get("no_rc", False)),
            takeoff_height=float(tl.get("takeoff_height", 1.0)),
            takeoff_land_speed=_positive(
                "auto_takeoff_land.takeoff_land_speed",
                float(tl.get("takeoff_land_speed", 0.5)),
            ),
        ),
        thrust_model=ThrustModelParams(
            accurate=bool(tm.get("accurate_thrust_model", True)),
            print_value=bool(tm.get("print_value", False)),
            k1=float(tm.get("K1", 0.7583)),
            k2=float(tm.get("K2", 1.6942)),
            k3=float(tm.get("K3", 0.6786)),
            hover_percentage=float(tm.get("hover_percentage", 0.30)),
        ),
        gain=GainParams(
            kp0=float(gain.get("Kp0", 6.0)),
            kp1=float(gain.get("Kp1", 6.0)),
            kp2=float(gain.get("Kp2", 5.0)),
            kv0=float(gain.get("Kv0", 4.0)),
            kv1=float(gain.get("Kv1", 4.0)),
            kv2=float(gain.get("Kv2", 5.0)),
            kang_r=float(gain.get("KAngR", 20.0)),
            kang_p=float(gain.get("KAngP", 20.0)),
            kang_y=float(gain.get("KAngY", 20.0)),
        ),
        timeouts=TimeoutParams(
            odom=float(tmo.get("odom", 0.5)),
            rc=float(tmo.get("rc", 0.5)),
            cmd=float(tmo.get("cmd", 0.5)),
            imu=float(tmo.get("imu", 0.5)),
            bat=float(tmo.get("bat", 0.5)),
        ),
        link=LinkParams(
            connection=str(_require(link, "connection")),
            target_system=int(_require(link, "target_system")),
            target_component=int(link.get("target_component", 1)),
            heartbeat_timeout=float(link.get("heartbeat_timeout", 15.0)),
        ),
        tasks=TaskDefaults(
            altitude=float(task.get("altitude", 2.0)),
            hold_seconds=float(task.get("hold_seconds", 30.0)),
            rate_hz=float(task.get("rate_hz", 20.0)),
            ready_timeout=float(task.get("ready_timeout", 60.0)),
            landing_timeout=float(task.get("landing_timeout", 60.0)),
            disarm_timeout=float(task.get("disarm_timeout", 15.0)),
            offset_north=float(task.get("offset_north", 0.0)),
            offset_east=float(task.get("offset_east", 0.0)),
        ),
        shared_frame=SharedFrameParams(
            enabled=bool(shared.get("enabled", False)),
            origin_latitude=float(shared.get("origin_latitude", 0.0)),
            origin_longitude=float(shared.get("origin_longitude", 0.0)),
            origin_altitude=float(shared.get("origin_altitude", 0.0)),
            earth_radius=(
                float(shared["earth_radius"])
                if shared.get("earth_radius")
                else None
            ),
        ),
    )

    if params.takeoff_land.takeoff_height <= 0.0:
        raise ParamError("auto_takeoff_land.takeoff_height 必须为正数")
    if not 0.0 < params.thrust_model.hover_percentage <= 1.0:
        raise ParamError("thrust_model.hover_percentage 必须位于 (0, 1]")
    if params.ctrl_freq_max <= 2.0:
        # PX4 在 Offboard 下要求设定点流 >2 Hz，否则直接触发 failsafe。
        raise ParamError("ctrl_freq_max 必须大于 2.0 Hz，否则 PX4 会因设定点流中断触发 failsafe")

    return params
