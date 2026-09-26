"""px4ctrl 统一入口。

本模块是命令执行的**唯一**入口：所有对飞控的操作都经过 :mod:`px4ctrl.link`，所有控制
计算都经过 :mod:`px4ctrl.controller` 与 :mod:`px4ctrl.fsm`。默认 dry-run，只有显式
``--execute`` 才会真正飞行。

任务
----
``probe``              只连接并打印遥测，不发任何命令
``measure-hover``      用位置模式悬停，记录执行器指令以标定 ``hover_percentage``
``takeoff-hover-land`` 用姿态+推力控制完成 自动起飞 → 悬停 → 降落 → 上锁
``hold``               保持相对起点的站位（T2 的最小可用形态）
``step-response``      单轴位置阶跃，用于辨识位置/姿态级联闭环并整定参数

用法::

    python -m px4ctrl.cli probe --role target
    python -m px4ctrl.cli takeoff-hover-land --role target --execute --hold-seconds 30
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from px4ctrl.controller import LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData, vlen, yaw_from_quaternion
from px4ctrl.link import MavlinkLink
from px4ctrl.params import Params, ParamError, load_params
from px4ctrl.plotting import PICTURE_DIR, write_response_plot
from px4ctrl.vehicle import available_roles, resolve_role

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "sim.yaml"
COMMAND_STREAM_HZ_MIN = 2.0
#: 常规上锁后等待其生效的时间（s）；仍未生效则尝试强制上锁。
DISARM_SETTLE_S = 5.0
#: 收尾时等待上锁生效的默认时长（s）；由任务参数 ``disarm_timeout`` 覆盖。
DISARM_TIMEOUT_S = 15.0
#: 保留的历史运行目录数量，避免日志无限占用磁盘。
#: 单次运行仅约 30 KB，因此保留 20 次仍可忽略；轮转过早会让文档引用的证据消失。
KEEP_RUNS = 20

#: 站位保持的稳态误差门槛（m），取自规划文档对 T2 的验收标准。
HOLD_STEADY_TOLERANCE_M = 0.5
STEP_RESPONSE_TOLERANCE_RATIO = 0.1


def prune_runs(root: Path, keep: int = KEEP_RUNS) -> None:
    """只保留最近 ``keep`` 个运行目录。"""

    if not root.is_dir():
        return
    runs = sorted((p for p in root.iterdir() if p.is_dir()), key=lambda p: p.name, reverse=True)
    for stale in runs[keep:]:
        shutil.rmtree(stale, ignore_errors=True)


def write_log(output_dir: Path, payload: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "run.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def thrust_model_record(fsm: PX4CtrlFSM) -> dict[str, Any]:
    """导出推力模型的最终状态，供离线核对在线估计的收敛情况。

    只记录结果是不够的：``stats`` 能区分“没开估计”、“开了但窗口从未命中（控制频率
    过低）”、“更新被越界拒绝（IMU 符号/标定可疑）”这三种完全不同的情况。
    """

    controller = fsm.controller
    model = fsm.params.thrust_model
    return {
        "thr2acc_initial": fsm.params.gra / model.hover_percentage,
        "thr2acc_final": controller.thrust_to_accel,
        "suggested_hover_percentage": controller.suggested_hover_percentage(),
        "online_estimate": model.online_estimate,
        "tilt_compensation": model.tilt_compensation,
        "delay_window_s": [model.estimate_delay_min_s, model.estimate_delay_max_s],
        "estimate_period_s": controller.estimate_period_s,
        "stats": controller.estimate_stats.as_dict(),
    }


def control_params_record(params: Params) -> dict[str, Any]:
    """记录本轮实际生效的关键控制参数，供仿真/真机调参横向比较。"""

    return {
        "max_angle_deg": params.max_angle,
        "gain": {
            "Kp": [params.gain.kp0, params.gain.kp1, params.gain.kp2],
            "Kv": [params.gain.kv0, params.gain.kv1, params.gain.kv2],
            "KAng": [params.gain.kang_r, params.gain.kang_p, params.gain.kang_y],
        },
        "so3": {
            "enabled": params.use_bodyrate_ctrl,
            "rate_damping": params.so3.rate_damping,
            "max_bodyrate_rad_s": params.so3.max_bodyrate,
        },
        "hover_percentage": params.thrust_model.hover_percentage,
    }


def step_response_metrics(samples: list[dict[str, float]], amplitude: float) -> dict[str, float | None]:
    """从阶跃后的采样计算可用于整定的响应指标。

    ``position`` 是相对阶跃前悬停点、沿测试轴的位移。指标以阶跃方向归一化，故正、负
    阶跃可以直接比较；不足以达到 90% 目标时上升时间和整定时间明确记为 ``None``。
    """

    if not samples or amplitude == 0.0:
        return {
            "rise_time_s": None,
            "settling_time_s": None,
            "overshoot_m": None,
            "final_error_m": None,
        }

    direction = 1.0 if amplitude > 0.0 else -1.0
    magnitude = abs(amplitude)
    response = [direction * sample["position"] for sample in samples]
    times = [sample["t"] for sample in samples]
    rise_time = next((time_value for time_value, value in zip(times, response) if value >= 0.9 * magnitude), None)
    overshoot = max(0.0, max(response) - magnitude)
    final_error = magnitude - (sum(response[-min(len(response), 20):]) / min(len(response), 20))
    tolerance = STEP_RESPONSE_TOLERANCE_RATIO * magnitude
    last_outside = max(
        (index for index, value in enumerate(response) if abs(value - magnitude) > tolerance),
        default=-1,
    )
    settling_time = None if last_outside == len(samples) - 1 else times[last_outside + 1]
    return {
        "rise_time_s": rise_time,
        "settling_time_s": settling_time,
        "overshoot_m": overshoot,
        "final_error_m": final_error,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "task",
        choices=["probe", "measure-hover", "takeoff-hover-land", "hold", "step-response"],
        help="要执行的任务",
    )
    parser.add_argument("--role", choices=available_roles(), default="target")
    parser.add_argument("--profile", choices=["sim", "real"], default="sim")
    parser.add_argument("--config", type=Path, default=None, help="覆盖参数文件路径")
    parser.add_argument("--execute", action="store_true", help="显式授权控制飞控；缺省仅探测")
    # 以下运行参数缺省为 None，表示"取 YAML 的 tasks 段"，命令行只作覆盖。
    parser.add_argument("--rate-hz", type=float, default=None, help="设定点流频率，必须 >2 Hz")
    parser.add_argument("--altitude", type=float, default=None, help="任务高度（m）")
    parser.add_argument("--offset-north", type=float, default=None)
    parser.add_argument("--offset-east", type=float, default=None)
    parser.add_argument("--hold-seconds", type=float, default=None)
    parser.add_argument(
        "--step-axis",
        choices=("east", "north", "up"),
        default="east",
        help="step-response 的 ENU 测试轴",
    )
    parser.add_argument("--step-amplitude", type=float, default=0.5, help="阶跃幅值（m，可为负）")
    parser.add_argument("--step-delay", type=float, default=5.0, help="进入悬停后施加阶跃前的等待时间（s）")
    parser.add_argument("--ready-timeout", type=float, default=None)
    parser.add_argument("--landing-timeout", type=float, default=None)
    parser.add_argument("--disarm-timeout", type=float, default=None)
    parser.add_argument(
        "--online-estimate",
        action="store_true",
        help="本次运行启用 RLS 在线估计 thr2acc（油门模型）；覆盖 YAML 的在线估计开关",
    )
    parser.add_argument("--output-root", type=Path, default=Path("logs/px4ctrl"))
    return parser.parse_args(argv)


def apply_task_defaults(args: argparse.Namespace, params: Params) -> None:
    """用 YAML 的 ``tasks`` 段补齐未在命令行给出的运行参数，并做取值校验。"""

    defaults = params.tasks
    for name, fallback in (
        ("rate_hz", defaults.rate_hz),
        ("altitude", defaults.altitude),
        ("offset_north", defaults.offset_north),
        ("offset_east", defaults.offset_east),
        ("hold_seconds", defaults.hold_seconds),
        ("ready_timeout", defaults.ready_timeout),
        ("landing_timeout", defaults.landing_timeout),
        ("disarm_timeout", defaults.disarm_timeout),
    ):
        if getattr(args, name) is None:
            setattr(args, name, fallback)

    if args.rate_hz <= 2.0:
        raise ParamError("rate_hz 必须大于 2.0，否则 PX4 会因设定点流中断触发 failsafe")
    if args.hold_seconds < 0.0:
        raise ParamError("hold_seconds 必须非负")
    if args.altitude is None or args.altitude <= 0.0:
        raise ParamError("altitude 必须为正数")


def resolve_params(args: argparse.Namespace) -> tuple[Params, Any]:
    """加载参数并把角色端点与参数中的 system id 覆盖合并。"""

    config_path = args.config
    if config_path is None:
        config_path = Path(__file__).resolve().parents[1] / "config" / f"{args.profile}.yaml"
    params = load_params(config_path)
    apply_task_defaults(args, params)

    # 命令行只允许**打开**在线估计（标定用），不允许关闭 YAML 里已打开的开关：
    # 用一个开关把正在使用的模型悄悄关掉，比多跑一次标定危险得多。
    if args.online_estimate and not params.thrust_model.online_estimate:
        params = replace(
            params, thrust_model=replace(params.thrust_model, online_estimate=True)
        )

    # 配对窗口与控制频率必须一致：周期大于窗口上界时窗口内永远不会出现样本。
    if params.thrust_model.online_estimate:
        model = params.thrust_model
        period = 1.0 / args.rate_hz
        if period > model.estimate_delay_max_s:
            print(
                f"提示：控制周期 {period * 1000.0:.0f} ms 大于配对窗口上界 "
                f"{model.estimate_delay_max_s * 1000.0:.0f} ms，在线估计将使用降级配对"
                f"（{'已允许' if model.estimate_allow_degraded else '已禁用，估计不会更新'}）。"
                "真机标定建议 --rate-hz 50 以上。"
            )

    role = resolve_role(args.role)
    if params.link.target_system:
        role = replace(role, system_id=params.link.target_system)
    return params, role


def wait_ready(link: MavlinkLink, fsm: PX4CtrlFSM, timeout: float, log: Callable[[str], None]) -> None:
    """等到载具在地面且遥测可用。

    就绪判定只使用遥测：onboard 链路不下发 ``STATUSTEXT``，因此不能用
    "Ready for takeoff" 文本作为门限。
    """

    log("等待载具就绪（在地面 + 遥测可用）...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        fsm.tick()
        link.pump()
        if link.odom.recv_time > 0.0 and link.is_landed():
            log(f"就绪：ENU={link.odom.p}")
            return
        time.sleep(0.01)
    raise RuntimeError(
        f"在 {timeout:.1f}s 内未就绪：LOCAL_POSITION_NED="
        f"{'已收到' if link.odom.recv_time > 0.0 else '未收到'}，"
        f"landed_state={link.extended_state.landed_state}（需为 ON_GROUND=1）"
    )


def enter_offboard(link: MavlinkLink, fsm: PX4CtrlFSM, log: Callable[[str], None]) -> None:
    """按 MAVSDK 的既定顺序进入 Offboard：先预热设定点流，再解锁，最后切模式。"""

    fsm.enable()
    log("预热设定点流 2.0s")
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        fsm.tick()
        time.sleep(0.01)

    if link.params.force_arm:
        log("ARM：仿真配置跳过 PX4 preflight 检查")
    log(f"ARM: {link.arm(force=link.params.force_arm, tick=fsm.tick)}")
    log(f"OFFBOARD: {link.set_offboard(tick=fsm.tick)}")

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        fsm.tick()
        if link.is_offboard():
            log("OFFBOARD 已确认")
            return
        time.sleep(0.01)
    raise RuntimeError("切换后未在 HEARTBEAT 中观测到 Offboard 主模式；PX4 可能已回落")


def finish(
    link: MavlinkLink,
    fsm: PX4CtrlFSM,
    landing_timeout: float,
    log: Callable[[str], None],
    disarm_timeout: float = DISARM_TIMEOUT_S,
) -> dict[str, Any]:
    """兜底收尾：请求降落 → 确认落地 → 上锁。所有退出路径都必须调用。

    上锁分**两级**，因为 PX4 会拒绝在地面判定未成立时的常规上锁（实测日志：
    ``Disarming denied: not landed``）。旧实现只记录不重试，于是收尾结果不再反映
    我们的指令是否被接受——最后一次实验里 target 实际是靠 PX4 自身的
    ``Failsafe: blind land`` 落地并上锁的，而我们的 ``run.json`` 只能记一个 WARN。
    """

    result: dict[str, Any] = {
        "land_command": None,
        "on_ground": False,
        "disarmed": False,
        "disarm_forced": False,
        "disarm_attempts": [],
    }
    try:
        result["land_command"] = link.land(tick=fsm.tick)
        log("LAND 已被接受")
    except Exception as error:  # 收尾失败不能掩盖原始故障
        result["land_error"] = str(error)
        log(f"WARN: 降落命令失败: {error}")

    fsm.disable()
    deadline = time.monotonic() + landing_timeout
    while time.monotonic() < deadline:
        link.pump()
        if link.is_landed():
            result["on_ground"] = True
            break
        time.sleep(0.02)

    # 第一级：常规上锁。
    attempt: dict[str, Any] = {"force": False, "accepted": False, "error": None}
    try:
        link.disarm(tick=None)
        attempt["accepted"] = True
        result["disarm_command"] = "accepted"
    except Exception as error:
        attempt["error"] = str(error)
        result["disarm_error"] = str(error)
        log(f"WARN: 常规上锁被拒: {error}")
    result["disarm_attempts"].append(attempt)

    # 常规上锁后仍可能因为状态机延迟而尚未生效，因此先给它一段有限等待。
    deadline = time.monotonic() + DISARM_SETTLE_S
    while time.monotonic() < deadline:
        link.pump()
        if not link.state.armed:
            break
        time.sleep(0.02)

    # 第二级：仍未上锁则强制上锁（PX4 魔术参数 21196）。宁可多一次带强制的尝试，
    # 也不要让飞机停在"已落地但保持解锁"的状态、把收尾交给飞控 failsafe。
    if link.state.armed:
        log("常规上锁未生效，尝试强制上锁")
        attempt = {"force": True, "accepted": False, "error": None}
        try:
            link.disarm(force=True, tick=None)
            attempt["accepted"] = True
            result["disarm_forced"] = True
        except Exception as error:
            attempt["error"] = str(error)
            log(f"WARN: 强制上锁失败: {error}")
        result["disarm_attempts"].append(attempt)

    deadline = time.monotonic() + disarm_timeout
    while time.monotonic() < deadline:
        link.pump()
        if not link.state.armed:
            result["disarmed"] = True
            break
        time.sleep(0.02)
    log(
        f"收尾完成：on_ground={result['on_ground']}, disarmed={result['disarmed']}"
        f"{'（强制）' if result['disarm_forced'] else ''}"
    )
    return result


def run_probe(link: MavlinkLink, fsm: PX4CtrlFSM, args: argparse.Namespace) -> dict[str, Any]:
    """只收集一小段遥测，用于确认端点与坐标系转换。"""

    samples: list[dict[str, float]] = []
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        fsm.tick()
        if link.odom.recv_time > 0.0:
            samples.append(
                {
                    "z": link.odom.p[2],
                    "v": vlen(link.odom.v),
                    "thrust": link.mean_actuator or 0.0,
                }
            )
        time.sleep(0.02)
    return {"samples": samples[-10:], "landed_state": link.extended_state.landed_state}


def run_measure_hover(
    link: MavlinkLink, fsm: PX4CtrlFSM, args: argparse.Namespace, log: Callable[[str], None]
) -> dict[str, Any]:
    """用位置模式悬停，记录执行器指令，用于标定 ``hover_percentage``。

    位置模式是已验收的通路；本任务不涉及姿态控制，因此可以安全地先做推力标定。
    """

    altitude = args.altitude
    home = link.odom.p
    # 保持起飞时的机头朝向，避免标定过程混入一次无意义的偏航旋转。
    hold_yaw = yaw_from_quaternion(link.odom.q)
    target = (home[0], home[1], home[2] + altitude)
    log(f"位置模式爬升到 ENU={target}（保持偏航 {hold_yaw:.3f} rad）")

    fsm.enable()
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        link.send_position_target(home, hold_yaw)
        link.pump()
        time.sleep(1.0 / args.rate_hz)

    log(f"ARM: {link.arm()}")
    log(f"OFFBOARD: {link.set_offboard()}")

    samples: list[float] = []
    altitudes: list[float] = []
    series: list[dict[str, float]] = []
    started = time.monotonic()
    rate = 1.0 / args.rate_hz
    deadline = time.monotonic() + 8.0 + args.hold_seconds
    while time.monotonic() < deadline:
        link.send_position_target(target, hold_yaw)
        link.pump()
        if link.odom.recv_time > 0.0 and link.mean_actuator is not None:
            altitude_now = link.odom.p[2] - home[2]
            altitudes.append(altitude_now)
            series.append(
                {
                    "t": time.monotonic() - started,
                    "altitude": altitude_now,
                    "actuator": link.mean_actuator if link.mean_actuator is not None else 0.0,
                }
            )
            # 只有接近目标高度、且垂直速度很小的样本才用于标定。
            if altitude_now > altitude - 0.3 and abs(link.odom.v[2]) < 0.15:
                samples.append(link.mean_actuator)
        time.sleep(rate)

    suggested = (sum(samples) / len(samples)) if samples else None
    log(f"标定样本 {len(samples)} 个；建议 hover_percentage = {suggested}")

    # 标定任务同样必须收尾降落，否则载具会被留在 Offboard 悬停状态。
    # 这里保持流开启交给 finish：它内部的 tick 需要流不中断才能安全切到 AUTO_LAND。
    result = finish(link, fsm, args.landing_timeout, log, args.disarm_timeout)

    return {
        "altitude_samples": len(altitudes),
        "max_altitude": max(altitudes) if altitudes else None,
        "hover_samples": len(samples),
        "suggested_hover_percentage": suggested,
        "samples": series[:: max(1, len(series) // 300)],
        **result,
    }


def run_takeoff_hover_land(
    link: MavlinkLink, fsm: PX4CtrlFSM, args: argparse.Namespace, log: Callable[[str], None]
) -> dict[str, Any]:
    """自动起飞 → 悬停 → 降落，全部使用姿态+推力控制（px4ctrl 的控制输出）。"""

    enter_offboard(link, fsm, log)
    fsm.request_takeoff(time.monotonic())

    samples: list[dict[str, float]] = []
    errors: list[float] = []
    steady_errors: list[float] = []
    hold_target = (
        link.odom.p[0] + args.offset_north,
        link.odom.p[1] + args.offset_east,
        link.odom.p[2] + fsm.params.takeoff_land.takeoff_height,
    )
    start = time.monotonic()
    hover_seen_at: float | None = None
    rate = 1.0 / args.rate_hz

    while True:
        now = time.monotonic()
        output = fsm.process(now)
        if link.odom.recv_time > 0.0:
            altitude = link.odom.p[2] - fsm.takeoff_land.start_pose[2]
            samples.append(
                {
                    "t": now - start,
                    "altitude": altitude,
                    "thrust": output.thrust,
                    "thr2acc": fsm.controller.thrust_to_accel,
                }
            )
            if fsm.state == State.AUTO_HOVER:
                if hover_seen_at is None:
                    hover_seen_at = now
                error = math.dist(link.odom.p, fsm.hover_pose)
                errors.append(error)
                if now - hover_seen_at >= 8.0:
                    steady_errors.append(error)
                if now - hover_seen_at >= args.hold_seconds:
                    break
            if now - start > 120.0:
                raise RuntimeError("起飞后在 120s 内未进入悬停并完成保持")
        time.sleep(rate)

    max_altitude = max((s["altitude"] for s in samples), default=0.0)
    result = finish(link, fsm, args.landing_timeout, log, args.disarm_timeout)
    return {
        "max_altitude_m": max_altitude,
        "hold_error_mean_m": (sum(errors) / len(errors)) if errors else None,
        "hold_error_max_m": max(errors) if errors else None,
        "hold_samples": len(errors),
        "steady_error_mean_m": (sum(steady_errors) / len(steady_errors)) if steady_errors else None,
        "steady_error_max_m": max(steady_errors) if steady_errors else None,
        "steady_samples": len(steady_errors),
        "settle_skip_s": 8.0,
        "samples": samples[:: max(1, len(samples) // 200)],
        **result,
    }


def run_hold(
    link: MavlinkLink, fsm: PX4CtrlFSM, args: argparse.Namespace, log: Callable[[str], None]
) -> dict[str, Any]:
    """保持相对起点的站位（T2 的最小形态，单机）。"""

    enter_offboard(link, fsm, log)
    start_pose = link.odom.p
    fsm.takeoff_land.start_pose = start_pose
    altitude = args.altitude
    target = (
        start_pose[0] + args.offset_north,
        start_pose[1] + args.offset_east,
        start_pose[2] + altitude,
    )
    log(f"HOLD：目标 ENU={target}")

    fsm.request_takeoff(time.monotonic())
    rate = 1.0 / args.rate_hz
    errors: list[float] = []
    #: 时间序列：t(相对悬停起点) / 位置误差 / 高度 / 推力。
    #: 整定需要区分"进入悬停时的瞬态"与"稳态"，只看聚合值无法判断该调哪个增益。
    series: list[dict[str, float]] = []
    deadline = time.monotonic() + 30.0 + args.hold_seconds
    settled_at: float | None = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        output = fsm.process(now)
        if fsm.state == State.AUTO_HOVER:
            if settled_at is None:
                settled_at = now
            error = math.dist(link.odom.p, fsm.hover_pose)
            errors.append(error)
            series.append(
                {
                    "t": now - settled_at,
                    "error": error,
                    "altitude": fsm.altitude,
                    "thrust": output.thrust,
                    # 在线推力模型的收敛过程：标定时看这一个序列就够了。
                    "thr2acc": fsm.controller.thrust_to_accel,
                }
            )
            if now - settled_at >= args.hold_seconds:
                break
        time.sleep(rate)

    result = finish(link, fsm, args.landing_timeout, log, args.disarm_timeout)

    # 稳态统计：去掉前 SETTLE_SKIP 秒，避免把爬升/收敛过程算进保持精度。
    settle_skip = 8.0
    steady = [s["error"] for s in series if s["t"] >= settle_skip]
    return {
        "hold_error_mean_m": (sum(errors) / len(errors)) if errors else None,
        "hold_error_max_m": max(errors) if errors else None,
        "hold_samples": len(errors),
        "steady_error_mean_m": (sum(steady) / len(steady)) if steady else None,
        "steady_error_max_m": max(steady) if steady else None,
        "steady_samples": len(steady),
        "settle_skip_s": settle_skip,
        "samples": series[:: max(1, len(series) // 300)],
        **result,
    }


def run_step_response(
    link: MavlinkLink, fsm: PX4CtrlFSM, args: argparse.Namespace, log: Callable[[str], None]
) -> dict[str, Any]:
    """执行单轴位置阶跃并记录可重复的闭环响应。

    阶跃在起飞并稳定悬停后才施加，前置等待阶段用于分离起飞交接瞬态。测试期间仍逐周期
    刷新 ``CMD_CTRL``，保留既有倾角饱和回退；触发回退时停止评估响应但仍走正常降落收尾。
    """

    axis_index = {"east": 0, "north": 1, "up": 2}[args.step_axis]
    enter_offboard(link, fsm, log)
    fsm.request_takeoff(time.monotonic())
    rate = 1.0 / args.rate_hz
    takeoff_deadline = time.monotonic() + 45.0
    while time.monotonic() < takeoff_deadline:
        fsm.process(time.monotonic())
        if fsm.state == State.AUTO_HOVER:
            break
        time.sleep(rate)
    else:
        raise RuntimeError("阶跃测试起飞后 45s 内未进入 AUTO_HOVER")

    log(f"STEP：悬停预稳 {args.step_delay:.1f}s，随后沿 ENU {args.step_axis} 阶跃 {args.step_amplitude:+.2f} m")
    pre_step_end = time.monotonic() + args.step_delay
    while time.monotonic() < pre_step_end:
        fsm.process(time.monotonic())
        time.sleep(rate)

    origin = link.odom.p
    target_p = (
        origin[0] + (args.step_amplitude if axis_index == 0 else 0.0),
        origin[1] + (args.step_amplitude if axis_index == 1 else 0.0),
        origin[2] + (args.step_amplitude if axis_index == 2 else 0.0),
    )
    fsm.request_command_control()
    started = time.monotonic()
    deadline = started + args.hold_seconds
    samples: list[dict[str, float]] = []
    fallback = False
    while time.monotonic() < deadline:
        now = time.monotonic()
        fsm.set_command(CommandData(p=target_p, yaw=fsm.hover_yaw), now=now)
        output = fsm.process(now)
        if fsm.state != State.CMD_CTRL:
            fallback = True
            log("WARN: 阶跃测试触发保护性悬停，停止采样并准备降落")
            break
        relative_position = link.odom.p[axis_index] - origin[axis_index]
        samples.append(
            {
                "t": now - started,
                "position": relative_position,
                "velocity": link.odom.v[axis_index],
                "roll_rad": fsm.controller.debug.roll,
                "pitch_rad": fsm.controller.debug.pitch,
                "bodyrate_roll": output.bodyrates[0],
                "bodyrate_pitch": output.bodyrates[1],
                "bodyrate_yaw": output.bodyrates[2],
                "tilt_saturated": float(fsm.controller.debug.tilt_saturated),
            }
        )
        time.sleep(rate)

    metrics = step_response_metrics(samples, args.step_amplitude)
    result = finish(link, fsm, args.landing_timeout, log, args.disarm_timeout)
    max_tilt_deg = max(
        (math.degrees(math.hypot(sample["roll_rad"], sample["pitch_rad"])) for sample in samples),
        default=0.0,
    )
    max_bodyrate = max(
        (
            max(abs(sample["bodyrate_roll"]), abs(sample["bodyrate_pitch"]), abs(sample["bodyrate_yaw"]))
            for sample in samples
        ),
        default=0.0,
    )
    return {
        "step": {
            "axis": args.step_axis,
            "amplitude_m": args.step_amplitude,
            "delay_s": args.step_delay,
            "target_local_enu": target_p,
            "origin_local_enu": origin,
        },
        "response": metrics,
        "max_tilt_deg": max_tilt_deg,
        "max_bodyrate_rad_s": max_bodyrate,
        "tilt_saturated_cycles": sum(int(sample["tilt_saturated"]) for sample in samples),
        "safety_fallback": fallback,
        "sample_count": len(samples),
        "samples": samples[:: max(1, len(samples) // 300)],
        **result,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.step_amplitude == 0.0:
        raise ParamError("step-amplitude 不能为 0")
    if args.step_delay < 0.0:
        raise ParamError("step-delay 必须非负")
    params, role = resolve_params(args)
    output_dir = args.output_root / f"{args.task}-{role.name}-{time.strftime('%Y%m%d-%H%M%S')}"
    prune_runs(args.output_root)

    record: dict[str, Any] = {
        "task": args.task,
        "role": role.name,
        "profile": args.profile,
        "execute": args.execute,
        "rate_hz": args.rate_hz,
        "control_mode": "so3_bodyrate" if params.use_bodyrate_ctrl else "quaternion_attitude",
        "so3_max_bodyrate_rad_s": params.so3.max_bodyrate,
        "control_params": control_params_record(params),
        "connection": params.link.connection or role.sim_connection,
        "expected_system_id": role.system_id,
    }

    link = MavlinkLink(params.link, role, shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link)
    try:
        link.open()
        record["heartbeat_system_id"] = link.connection.target_system
        if args.task == "probe" or not args.execute:
            outcome = run_probe(link, fsm, args)
            record.update(outcome)
            record["result"] = "dry_run_probe_ok"
            print("DRY RUN PASS: 未发送任何控制命令。使用 --execute 才会飞行。")
            return 0

        wait_ready(link, fsm, args.ready_timeout, print)

        if args.task == "measure-hover":
            record.update(run_measure_hover(link, fsm, args, print))
        elif args.task == "takeoff-hover-land":
            record.update(run_takeoff_hover_land(link, fsm, args, print))
        elif args.task == "hold":
            record.update(run_hold(link, fsm, args, print))
        elif args.task == "step-response":
            record.update(run_step_response(link, fsm, args, print))

        # 推力模型的最终值与统计：无论是否开启在线估计都记录，便于对比。
        record["thrust_model"] = thrust_model_record(fsm)
        record["result"] = "completed"
        # 所有任务都必须安全收尾；标定任务还必须有可用样本，否则"通过"是无意义的。
        passed = bool(record.get("on_ground")) and bool(record.get("disarmed"))
        if args.task == "measure-hover":
            passed = passed and record.get("suggested_hover_percentage") is not None
        elif args.task in ("takeoff-hover-land", "hold"):
            passed = passed and bool(record.get("hold_samples"))
            # 规划文档对 T2 的验收是"稳定后站位误差 < 0.5 m"。用稳态误差而非聚合值
            # 判定，否则爬升/交接瞬态会掩盖真实的保持精度。
            steady = record.get("steady_error_mean_m")
            passed = passed and steady is not None and steady <= HOLD_STEADY_TOLERANCE_M
        elif args.task == "step-response":
            response = record["response"]
            passed = (
                passed
                and not record["safety_fallback"]
                and record["sample_count"] > 0
                and response["rise_time_s"] is not None
            )
        record["passed"] = passed
        print(f"{'PASS' if passed else 'FAIL'}: {args.task}")
        return 0 if passed else 1
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: 正在执行兜底降落", file=sys.stderr)
        try:
            finish(link, fsm, args.landing_timeout, print, args.disarm_timeout)
        except Exception as error:  # 兜底路径不能再抛，否则掩盖中断
            record["failsafe_error"] = str(error)
        return 130
    except Exception as error:
        record["result"] = "failed"
        record["error"] = str(error)
        print(f"FAIL: {error}", file=sys.stderr)
        try:
            if fsm.stream_enabled:
                finish(link, fsm, args.landing_timeout, print, args.disarm_timeout)
        except Exception as inner:  # 兜底路径不能再抛
            record["failsafe_error"] = str(inner)
        return 1
    finally:
        write_log(output_dir, record)
        plot_path = write_response_plot(output_dir, record, picture_dir=PICTURE_DIR)
        if plot_path is not None:
            record["response_plot"] = plot_path.name
            record["picture_plot"] = str(PICTURE_DIR / f"{output_dir.name}.png")
            write_log(output_dir, record)
            print(f"PLOT: {plot_path}；归档: {record['picture_plot']}")
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())
