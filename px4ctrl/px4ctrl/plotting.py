"""飞行任务结束后的无界面响应绘图。"""

from __future__ import annotations

import math
import logging
from pathlib import Path
from typing import Any

PICTURE_DIR = Path(__file__).resolve().parents[2] / "picture"


def write_response_plot(
    output_dir: Path,
    record: dict[str, Any],
    picture_dir: Path | None = None,
) -> Path | None:
    """生成运行记录图，并可额外归档到独立图片目录。

    ``output_dir/response.png`` 保留与 ``run.json`` 一一对应的原始证据；归档图以运行目录名
    命名，避免 target 与 tracker 的同名响应图相互覆盖。
    """

    samples = record.get("samples")
    if not isinstance(samples, list) or not samples:
        return None

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot
    except ImportError:
        logging.getLogger(__name__).warning("无法绘制 %s：matplotlib 未安装", output_dir)
        return None

    task = record.get("task")
    try:
        if task == "step-response":
            figure = _plot_step_response(plot, record, samples)
        elif task == "tracker-v0":
            figure = _plot_tracker_response(plot, samples, _target_samples(record, samples))
        elif task in {"target-waypoints", "target-trajectory"}:
            figure = _plot_target_response(plot, samples)
        elif task in {"measure-hover", "takeoff-hover-land", "hold"}:
            figure = _plot_single_vehicle_response(plot, record, samples)
        else:
            return None
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / "response.png"
        figure.savefig(path, dpi=150, bbox_inches="tight")
        if picture_dir is not None:
            picture_dir.mkdir(parents=True, exist_ok=True)
            figure.savefig(picture_dir / f"{output_dir.name}.png", dpi=150, bbox_inches="tight")
        plot.close(figure)
        return path
    except Exception:
        # 飞行已经结束：绘图只是诊断产物，不能覆盖已完成任务的结果或收尾状态。
        logging.getLogger(__name__).exception("绘制飞行响应图失败：%s", output_dir)
        return None


def _time(samples: list[dict[str, float]]) -> list[float]:
    start = samples[0]["t"]
    return [sample["t"] - start for sample in samples]


def _valid_pairs(
    time: list[float], values: list[float]
) -> tuple[list[float], list[float]]:
    """过滤 NaN（如量测丢帧），保留可绘制点。"""

    finite = []
    finite_time = []
    for timestamp, value in zip(time, values):
        if math.isfinite(value):
            finite.append(value)
            finite_time.append(timestamp)
    return finite_time, finite


def _plot_estimation_axis(
    plot: Any,
    output_dir: Path,
    target_samples: list[dict[str, float]],
    quantity: str,
    axis: str,
) -> Path | None:
    """生成单轴估计对比图：真值、量测（如有）、EKF 估计。"""

    time = _time(target_samples)
    if quantity == "position":
        truth_key, estimate_key, measurement_key = f"target_{axis}", f"estimate_{axis}", f"measurement_{axis}"
    elif quantity == "velocity":
        truth_key, estimate_key, measurement_key = f"target_v{axis}", f"estimate_v{axis}", None
    else:
        truth_key, estimate_key, measurement_key = f"target_a{axis}", f"estimate_a{axis}", None
    if truth_key not in target_samples[0] or estimate_key not in target_samples[0]:
        return None
    axis_label = {"e": "east", "n": "north", "u": "up"}[axis]
    figure, axes = plot.subplots(1, 1, figsize=(9, 4))
    truth_label = "trajectory reference" if quantity == "acceleration" else "target truth"
    truth = [sample[truth_key] for sample in target_samples]
    if any(math.isfinite(value) for value in truth):
        finite_time, finite = _valid_pairs(time, truth)
        axes.plot(finite_time, finite, label=truth_label, linewidth=1.2)
    if measurement_key is not None and measurement_key in target_samples[0]:
        measured = [sample.get(measurement_key, float("nan")) for sample in target_samples]
        if any(math.isfinite(value) for value in measured):
            finite_time, finite = _valid_pairs(time, measured)
            axes.plot(finite_time, finite, ".", markersize=2.5, alpha=0.6, label="measurement")
    axes.plot(time, [sample[estimate_key] for sample in target_samples], label="EKF estimate", linewidth=1.2)
    units = {"position": "[m]", "velocity": "[m/s]", "acceleration": "[m/s²]"}[quantity]
    axes.set_title(f"{axis_label} {quantity} estimation")
    axes.set_xlabel("time [s]")
    axes.set_ylabel(f"{axis_label} {quantity} {units}")
    axes.grid(True, alpha=0.3)
    axes.legend(loc="best")
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{quantity}_{axis_label}.png"
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plot.close(figure)
    return path


def _plot_tracker_trajectory_3d(
    plot: Any, samples: list[dict[str, float]], target_samples: list[dict[str, float]]
) -> Any:
    """共享 ENU 双机航迹；Z 已朝上，不再翻转或冒充 EKF 重建轨迹。"""

    figure = plot.figure(figsize=(10, 7), layout="constrained")
    axis = figure.add_subplot(111, projection="3d")
    trajectories = (
        (target_samples, "target", "Target (PX4 shared ENU)", "tab:red", "-"),
        (samples, "tracker", "Tracker (PX4 shared ENU)", "tab:blue", "-"),
        (samples, "desired", "Desired tracker", "tab:green", "--"),
    )
    coordinates = [[], [], []]
    for stream, prefix, label, color, style in trajectories:
        values = [[sample[f"{prefix}_{key}"] for sample in stream] for key in ("e", "n", "u")]
        axis.plot(*values, label=label, color=color, linestyle=style, linewidth=1.5)
        axis.scatter(*(values[index][0] for index in range(3)), color=color, marker="o", s=25)
        axis.scatter(*(values[index][-1] for index in range(3)), color=color, marker="x", s=35)
        for dimension, points in zip(coordinates, values):
            dimension.extend(value for value in points if math.isfinite(value))
    spans = []
    for dimension, set_limits in zip(coordinates, (axis.set_xlim, axis.set_ylim, axis.set_zlim)):
        low, high = min(dimension), max(dimension)
        span = max(high - low, 1.0)
        center = (low + high) / 2
        set_limits(center - span * 0.55, center + span * 0.55)
        spans.append(span)
    axis.set_box_aspect(spans)
    axis.view_init(elev=25, azim=-65)
    axis.set_xlabel("X / East (m)")
    axis.set_ylabel("Y / North (m)")
    axis.set_zlabel("Z / Up (m)")
    axis.set_title("3D trajectories (shared ENU; circle=start, cross=end)")
    axis.legend(loc="upper left", fontsize=9)
    return figure


def write_tracker_estimation_plots(output_dir: Path, record: dict[str, Any]) -> list[Path]:
    """为 tracker 运行生成九张单轴估计图和四张跟踪器图。

    九张图：位置/速度/加速度 × 东/北/上；跟踪器图：3D/水平轨迹、三轴位置跟踪、误差。
    返回生成的文件路径列表；调用方负责写入 run.json。
    """

    samples = record.get("samples")
    if not isinstance(samples, list) or not samples or record.get("task") != "tracker-v0":
        return []
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot
    except ImportError:
        return []

    target_samples = _target_samples(record, samples)
    paths: list[Path] = []
    try:
        figures_dir = output_dir / "figures"
        figures_dir.mkdir(parents=True, exist_ok=True)
        trajectory_3d = figures_dir / "tracker_trajectory_3d.png"
        figure = _plot_tracker_trajectory_3d(plot, samples, target_samples)
        figure.savefig(trajectory_3d, dpi=150, bbox_inches="tight")
        plot.close(figure)
        paths.append(trajectory_3d)
        for quantity in ("position", "velocity", "acceleration"):
            for axis in ("e", "n", "u"):
                path = _plot_estimation_axis(plot, figures_dir, target_samples, quantity, axis)
                if path is not None:
                    paths.append(path)
        time = _time(samples)
        tracker_plan = figures_dir / "tracker_horizontal_trajectory.png"
        figure, axes = plot.subplots(1, 1, figsize=(7, 7))
        axes.plot(
            [sample["target_e"] for sample in target_samples],
            [sample["target_n"] for sample in target_samples],
            label="target truth",
        )
        axes.plot(
            [sample["desired_e"] for sample in samples],
            [sample["desired_n"] for sample in samples],
            linestyle="--",
            label="desired tracker",
        )
        axes.plot(
            [sample["tracker_e"] for sample in samples],
            [sample["tracker_n"] for sample in samples],
            label="tracker actual",
        )
        axes.set_title("tracker horizontal trajectory (shared ENU)")
        axes.set_xlabel("east [m]")
        axes.set_ylabel("north [m]")
        axes.set_aspect("equal", adjustable="box")
        axes.grid(True, alpha=0.3)
        axes.legend(loc="best")
        figure.savefig(tracker_plan, dpi=150, bbox_inches="tight")
        plot.close(figure)
        paths.append(tracker_plan)

        tracker_positions = figures_dir / "tracker_position_tracking.png"
        figure, axes = plot.subplots(3, 1, figsize=(9, 8), sharex=True)
        for axis, (label, key) in zip(axes, (("east", "e"), ("north", "n"), ("up", "u"))):
            axis.plot(_time(target_samples), [sample[f"target_{key}"] for sample in target_samples], label="target")
            axis.plot(time, [sample[f"desired_{key}"] for sample in samples], label="desired")
            axis.plot(time, [sample[f"tracker_{key}"] for sample in samples], label="tracker")
            axis.set_ylabel(f"{label} [m]")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best")
        axes[-1].set_xlabel("time [s]")
        figure.savefig(tracker_positions, dpi=150, bbox_inches="tight")
        plot.close(figure)
        paths.append(tracker_positions)

        tracker_errors = figures_dir / "tracker_error.png"
        figure, axes = plot.subplots(1, 1, figsize=(9, 4))
        axes.plot(time, [sample["error"] for sample in samples], label="tracking error")
        axes.plot(
            time,
            [sample["measurement_error"] for sample in samples],
            label="measurement error",
        )
        if "estimate_error" in samples[0]:
            axes.plot(
                time,
                [sample["estimate_error"] for sample in samples],
                label="estimate error",
            )
        axes.set_title("tracker tracking error")
        axes.set_xlabel("time [s]")
        axes.set_ylabel("error [m]")
        axes.grid(True, alpha=0.3)
        axes.legend(loc="best")
        figure.savefig(tracker_errors, dpi=150, bbox_inches="tight")
        plot.close(figure)
        paths.append(tracker_errors)
        return paths
    except Exception:
        # 绘图失败不得覆盖飞行结论；返回已生成的图，缺失图不阻断运行记录。
        return paths


def _target_samples(record: dict[str, Any], fallback: list[dict[str, float]]) -> list[dict[str, float]]:
    """返回唯一目标报文流；兼容旧日志时退回控制周期样本。"""

    target_samples = record.get("target_samples")
    return target_samples if isinstance(target_samples, list) and target_samples else fallback


def _plot_step_response(plot: Any, record: dict[str, Any], samples: list[dict[str, float]]) -> Any:
    step = record["step"]
    amplitude = float(step["amplitude_m"])
    time = _time(samples)
    figure, axes = plot.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(time, [sample["position"] for sample in samples], label="measured")
    axes[0].axhline(amplitude, color="tab:red", linestyle="--", label="target")
    axes[0].axhline(amplitude * 0.9, color="tab:red", linestyle=":", linewidth=1)
    axes[0].axhline(amplitude * 1.1, color="tab:red", linestyle=":", linewidth=1)
    axes[0].set_ylabel("axis position [m]")
    axes[0].set_title(f"{record['role']} {step['axis']} step {amplitude:+.2f} m")
    axes[0].legend(loc="best")

    axes[1].plot(time, [sample["velocity"] for sample in samples], label="velocity")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("axis velocity [m/s]")

    axes[2].plot(time, [sample["roll_rad"] for sample in samples], label="roll request")
    axes[2].plot(time, [sample["pitch_rad"] for sample in samples], label="pitch request")
    axes[2].plot(time, [sample["bodyrate_roll"] for sample in samples], label="roll rate request")
    axes[2].plot(time, [sample["bodyrate_pitch"] for sample in samples], label="pitch rate request")
    axes[2].plot(time, [sample["bodyrate_yaw"] for sample in samples], label="yaw rate request")
    axes[2].set_xlabel("time after step [s]")
    axes[2].set_ylabel("rad / rad/s")
    axes[2].legend(loc="best", ncol=2)
    for axis in axes:
        axis.grid(True, alpha=0.3)
    return figure


def _plot_tracker_response(
    plot: Any, samples: list[dict[str, float]], target_samples: list[dict[str, float]]
) -> Any:
    time = _time(samples)
    figure, axes = plot.subplots(3, 2, figsize=(12, 12))
    plan_axis = axes[0, 0]
    plan_axis.plot(
        [sample["target_e"] for sample in target_samples],
        [sample["target_n"] for sample in target_samples],
        label="target truth",
    )
    plan_axis.plot(
        [sample["desired_e"] for sample in samples],
        [sample["desired_n"] for sample in samples],
        linestyle="--",
        label="desired tracker",
    )
    plan_axis.plot(
        [sample["tracker_e"] for sample in samples],
        [sample["tracker_n"] for sample in samples],
        label="tracker actual",
    )
    plan_axis.set_title("shared ENU horizontal trajectory")
    plan_axis.set_xlabel("east [m]")
    plan_axis.set_ylabel("north [m]")
    plan_axis.set_aspect("equal", adjustable="box")
    plan_axis.grid(True, alpha=0.3)
    plan_axis.legend(loc="best")
    distance_axis = axes[0, 1]
    if "horizontal_distance" in samples[0] and "desired_distance" in samples[0]:
        distance_axis.plot(time, [sample["horizontal_distance"] for sample in samples], label="actual")
        distance_axis.plot(time, [sample["desired_distance"] for sample in samples], label="desired")
        distance_axis.set_title("horizontal follow distance")
        distance_axis.set_xlabel("time [s]")
        distance_axis.set_ylabel("distance [m]")
        distance_axis.grid(True, alpha=0.3)
        distance_axis.legend(loc="best")
    else:
        distance_axis.axis("off")
    labels = (("east", "e"), ("north", "n"), ("up", "u"))
    for axis, (label, key) in zip((axes[1, 0], axes[1, 1], axes[2, 0]), labels):
        axis.plot(_time(target_samples), [sample[f"target_{key}"] for sample in target_samples], label="target")
        axis.plot(time, [sample[f"desired_{key}"] for sample in samples], label="desired tracker")
        axis.plot(time, [sample[f"tracker_{key}"] for sample in samples], label="tracker")
        axis.set_title(f"shared {label}")
        axis.set_ylabel("position [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    error_axis = axes[2, 1]
    error_axis.plot(time, [sample["error"] for sample in samples], label="tracking error")
    error_axis.plot(time, [sample["measurement_error"] for sample in samples], label="measurement error")
    if "estimate_error" in samples[0]:
        error_axis.plot(time, [sample["estimate_error"] for sample in samples], label="estimate error")
    error_axis.set_title("tracking error")
    error_axis.set_ylabel("error [m]")
    error_axis.grid(True, alpha=0.3)
    error_axis.legend(loc="best")
    for axis in (axes[1, 0], axes[1, 1], axes[2, 0], axes[2, 1]):
        axis.set_xlabel("time [s]")
    return figure


def _plot_target_response(plot: Any, samples: list[dict[str, float]]) -> Any:
    time = _time(samples)
    figure, axes = plot.subplots(3, 2, figsize=(12, 12))
    plan_axis = axes[0, 0]
    plan_axis.plot(
        [sample["reference_e"] for sample in samples],
        [sample["reference_n"] for sample in samples],
        linestyle="--",
        label="reference",
    )
    plan_axis.plot(
        [sample["actual_e"] for sample in samples],
        [sample["actual_n"] for sample in samples],
        label="target actual",
    )
    plan_axis.set_title("target local ENU horizontal trajectory")
    plan_axis.set_xlabel("east [m]")
    plan_axis.set_ylabel("north [m]")
    plan_axis.set_aspect("equal", adjustable="box")
    plan_axis.grid(True, alpha=0.3)
    plan_axis.legend(loc="best")
    speed_axis = axes[0, 1]
    if "actual_speed" in samples[0] and "reference_speed" in samples[0]:
        speed_axis.plot(time, [sample["actual_speed"] for sample in samples], label="target actual")
        speed_axis.plot(time, [sample["reference_speed"] for sample in samples], label="reference")
        speed_axis.axhline(2.0, color="gray", linestyle=":", label="2 m/s")
        speed_axis.set_title("target speed")
        speed_axis.set_xlabel("time [s]")
        speed_axis.set_ylabel("speed [m/s]")
        speed_axis.grid(True, alpha=0.3)
        speed_axis.legend(loc="best")
    else:
        speed_axis.axis("off")
    labels = (("East position", "e"), ("North position", "n"), ("Altitude", "u"))
    for axis, (label, key) in zip((axes[1, 0], axes[1, 1], axes[2, 0]), labels):
        axis.plot(time, [sample[f"actual_{key}"] for sample in samples], label="target")
        axis.plot(time, [sample[f"reference_{key}"] for sample in samples], label="reference")
        axis.set_title(label)
        axis.set_ylabel("position [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    error_axis = axes[2, 1]
    error_axis.plot(time, [sample["error"] for sample in samples], label="waypoint error")
    error_axis.set_title("waypoint error")
    error_axis.set_ylabel("error [m]")
    error_axis.grid(True, alpha=0.3)
    error_axis.legend(loc="best")
    for axis in (axes[1, 0], axes[1, 1], axes[2, 0], axes[2, 1]):
        axis.set_xlabel("time [s]")
    return figure


def _plot_single_vehicle_response(plot: Any, record: dict[str, Any], samples: list[dict[str, float]]) -> Any:
    time = _time(samples)
    figure, axes = plot.subplots(3, 1, figsize=(10, 8), sharex=True)
    altitude = [sample.get("altitude", 0.0) for sample in samples]
    axes[0].plot(time, altitude, label="altitude")
    axes[0].set_title(f"{record['role']} {record['task']} response")
    axes[0].set_ylabel("relative altitude [m]")
    axes[0].legend(loc="best")

    errors = [sample.get("error") for sample in samples]
    if any(error is not None for error in errors):
        axes[1].plot(time, [0.0 if error is None else error for error in errors], label="position error")
        axes[1].legend(loc="best")
    axes[1].set_ylabel("error [m]")

    thrust_key = "thrust" if "thrust" in samples[0] else "actuator"
    axes[2].plot(time, [sample.get(thrust_key, 0.0) for sample in samples], label=thrust_key)
    axes[2].set_ylabel("normalized thrust")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(loc="best")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    return figure
