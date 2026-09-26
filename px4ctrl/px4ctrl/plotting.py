"""飞行任务结束后的无界面响应绘图。"""

from __future__ import annotations

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
        return None

    task = record.get("task")
    try:
        if task == "step-response":
            figure = _plot_step_response(plot, record, samples)
        elif task == "tracker-v0":
            figure = _plot_tracker_response(plot, samples)
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
        return None


def _time(samples: list[dict[str, float]]) -> list[float]:
    start = samples[0]["t"]
    return [sample["t"] - start for sample in samples]


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


def _plot_tracker_response(plot: Any, samples: list[dict[str, float]]) -> Any:
    time = _time(samples)
    figure, axes = plot.subplots(3, 2, figsize=(12, 12))
    plan_axis = axes[0, 0]
    plan_axis.plot(
        [sample["target_e"] for sample in samples],
        [sample["target_n"] for sample in samples],
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
    axes[0, 1].axis("off")
    labels = (("east", "e"), ("north", "n"), ("up", "u"))
    for axis, (label, key) in zip((axes[1, 0], axes[1, 1], axes[2, 0]), labels):
        axis.plot(time, [sample[f"target_{key}"] for sample in samples], label="target")
        axis.plot(time, [sample[f"desired_{key}"] for sample in samples], label="desired tracker")
        axis.plot(time, [sample[f"tracker_{key}"] for sample in samples], label="tracker")
        axis.set_title(f"shared {label}")
        axis.set_ylabel("position [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    error_axis = axes[2, 1]
    error_axis.plot(time, [sample["error"] for sample in samples], label="tracking error")
    error_axis.plot(time, [sample["measurement_error"] for sample in samples], label="measurement error")
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
    axes[0, 1].axis("off")
    labels = (("east", "e"), ("north", "n"), ("up", "u"))
    for axis, (label, key) in zip((axes[1, 0], axes[1, 1], axes[2, 0]), labels):
        axis.plot(time, [sample[f"actual_{key}"] for sample in samples], label="target")
        axis.plot(time, [sample[f"reference_{key}"] for sample in samples], label="reference")
        axis.set_title(f"local {label}")
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
