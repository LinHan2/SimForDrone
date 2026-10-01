"""只读 shadow 日志绘图；不依赖 ROS，不连接飞控。"""

from pathlib import Path

import numpy as np


def _plot_nine_state_response(plot, record):
    """相对位置/速度与目标加速度九维图，保留缺测和采样间断。"""

    samples = record["samples"]
    if not samples:
        raise ValueError("没有有效观测样本，无法绘图")
    times = np.asarray([sample["t"] for sample in samples])
    segments = np.split(np.arange(len(times)), np.flatnonzero(np.diff(times) > 0.5) + 1)
    figure, axes = plot.subplots(3, 3, figsize=(14, 10), sharex=True)
    quantities = (("relative_position", "Relative position", "m"),
                  ("relative_velocity", "Relative velocity", "m/s"),
                  ("target_acceleration", "Target acceleration", "m/s^2"))
    legend = {}
    for row, (quantity, title, unit) in enumerate(quantities):
        for column, direction in enumerate(("X / East", "Y / North", "Z / Up")):
            axis = axes[row, column]
            curves = [
                ([sample.get("truth", {}).get(quantity) for sample in samples],
                 "ROS pose (scoring)" if row == 0 else "Host-clock derivative (proxy)",
                 "black", "-"),
                ([sample.get("estimate", {}).get(quantity) for sample in samples],
                 "Shadow EKF", "tab:blue", "--"),
            ]
            if row == 0:
                curves.insert(1, ([sample.get("measurement_relative_position") for sample in samples],
                                  "Depth measurement", "tab:orange", ":"))
            for vectors, label, color, style in curves:
                values = np.asarray([vector[column] if vector is not None else np.nan for vector in vectors],
                                    dtype=float)
                values[~np.isfinite(values)] = np.nan
                for segment in segments:
                    line, = axis.plot(times[segment], values[segment], color=color, linestyle=style,
                                      linewidth=1.3, label=label if segment[0] == 0 else None)
                    if segment[0] == 0:
                        legend[label] = line
            axis.set_title(f"{direction}: {title}", fontsize=11)
            axis.set_ylabel(unit)
            axis.grid(alpha=0.25)
            if row == 2:
                axis.set_xlabel("Shadow elapsed time (host s)")
    figure.suptitle("Nine-state response (ENU): relative position / relative velocity / target acceleration\n"
                   "Velocity/acceleration references are host-clock proxies, not physical-time ground truth",
                   fontsize=13)
    figure.legend(legend.values(), legend.keys(), loc="lower center", ncol=4, fontsize=9)
    figure.tight_layout(rect=(0, 0.05, 1, 0.92))
    return figure


def plot_observation(record, output: Path) -> None:
    """保留原始观测图，并把运动学图扩展为 3x3 九维响应。"""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plot

    samples = record["samples"]
    if not samples:
        raise ValueError("没有有效观测样本，无法绘图")
    times = np.asarray([sample["t"] for sample in samples])
    positions = np.asarray([sample["truth"]["relative_position"] for sample in samples])
    measured = np.asarray([sample["measurement_relative_position"] for sample in samples])
    estimated = np.asarray([sample["estimate"]["relative_position"] for sample in samples])
    gaps = np.flatnonzero(np.diff(times) > 0.5)
    segments = np.split(np.arange(len(times)), gaps + 1)
    figure, axes = plot.subplots(4, 1, figsize=(11, 11), sharex=True)
    try:
        for index, direction in enumerate(("east", "north", "up")):
            for segment in segments:
                axes[index].plot(times[segment], positions[segment, index], color="black",
                                 label="ROS truth (scoring)" if segment[0] == 0 else None)
                axes[index].plot(times[segment], measured[segment, index], color="tab:orange", alpha=0.55,
                                 label="RGB-D measurement" if segment[0] == 0 else None)
                axes[index].plot(times[segment], estimated[segment, index], color="tab:blue",
                                 label="EKF estimate" if segment[0] == 0 else None)
            axes[index].set_ylabel(f"relative {direction} (m)")
        errors = np.asarray([sample["position_error_m"] for sample in samples])
        for segment in segments:
            axes[3].plot(times[segment], errors[segment], color="tab:red",
                         label="position error" if segment[0] == 0 else None)
        axes[3].set_ylabel("position error (m)")
        axes[3].set_xlabel("shadow elapsed time (host s)")
        for axis in axes:
            axis.grid(alpha=0.25)
        axes[0].legend(loc="best", fontsize=8)
        axes[3].legend(loc="best", fontsize=8)
        figure.suptitle("Shadow depth observation and metric EKF (host-clock timeline)")
        figure.tight_layout()
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=150)
    finally:
        plot.close(figure)

    figure = _plot_nine_state_response(plot, record)
    try:
        figure.savefig(output.with_name(output.stem.replace("-observation", "-kinematics") + ".png"), dpi=150)
    finally:
        plot.close(figure)