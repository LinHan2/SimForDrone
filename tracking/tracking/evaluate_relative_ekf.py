"""在物理一致的合成机动上扫描 yaw-invariant 相对 EKF 参数。

此工具刻意不启动 PX4 或控制器。目标相对位置在固定半径圆周上运动，因此当前过渡
``relative_position -> unit bearing`` 前端满足 ``delta_p = alpha * p_bar``；同时目标推力轴
按 ``h = normalize(a_target - g)`` 构造，严格满足非合作多旋翼动力学约束。
"""

from __future__ import annotations

import argparse
import itertools
import math
from pathlib import Path

import numpy as np

from tracking.relative_ekf import RelativePoseMeasurement, RelativeTargetEKF

GRAVITY_ENU = np.array((0.0, 0.0, -9.81))
ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0, help="每组参数的仿真时长（s）")
    parser.add_argument("--dt", type=float, default=0.05, help="采样周期（s）")
    parser.add_argument("--radius", type=float, default=6.0, help="相对圆周半径（m）")
    parser.add_argument("--angular-rate", type=float, default=0.35, help="圆周角速度（rad/s）")
    parser.add_argument("--bearing-noise", type=float, default=0.05, help="相对位置代理噪声标准差（m）")
    parser.add_argument("--tilt-noise-deg", type=float, default=2.0, help="推力轴切空间噪声标准差（deg）")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "logs" / "relative-ekf-evaluation",
        help="观测器真值/估计对比图输出目录",
    )
    return parser.parse_args()


def circle_truth(timestamp: float, radius: float, angular_rate: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    phase = angular_rate * timestamp
    position = radius * np.array((math.cos(phase), math.sin(phase), 0.0))
    velocity = radius * angular_rate * np.array((-math.sin(phase), math.cos(phase), 0.0))
    acceleration = -(radius * angular_rate**2) * np.array((math.cos(phase), math.sin(phase), 0.0))
    thrust_direction = acceleration - GRAVITY_ENU
    thrust_direction /= np.linalg.norm(thrust_direction)
    return position, velocity, acceleration, thrust_direction


def noisy_unit_direction(direction: np.ndarray, standard_deviation_rad: float, random_source: np.random.Generator) -> np.ndarray:
    basis = RelativeTargetEKF._tangent_basis(direction)
    perturbation = basis @ random_source.normal(0.0, standard_deviation_rad, size=2)
    noisy = direction + perturbation
    return noisy / np.linalg.norm(noisy)


def evaluate(
    position_std: float,
    attitude_constraint_std: float,
    acceleration_process_std: float,
    args: argparse.Namespace,
    tilt_direction_std: float | None = None,
) -> dict[str, float]:
    metrics, _ = simulate(
        position_std,
        attitude_constraint_std,
        acceleration_process_std,
        args,
        tilt_direction_std,
    )
    return metrics


def simulate(
    position_std: float,
    attitude_constraint_std: float,
    acceleration_process_std: float,
    args: argparse.Namespace,
    tilt_direction_std: float | None = None,
) -> tuple[dict[str, float], dict[str, np.ndarray]]:
    random_source = np.random.default_rng(args.seed)
    estimator = RelativeTargetEKF(
        position_std=position_std,
        attitude_constraint_std=attitude_constraint_std,
        acceleration_process_std=acceleration_process_std,
        tilt_direction_std=tilt_direction_std,
    )
    squared_position_error: list[float] = []
    squared_velocity_error: list[float] = []
    squared_acceleration_error: list[float] = []
    normalized_estimation_errors: list[float] = []
    times: list[float] = []
    truth_positions: list[np.ndarray] = []
    truth_velocities: list[np.ndarray] = []
    truth_accelerations: list[np.ndarray] = []
    estimate_positions: list[np.ndarray] = []
    estimate_velocities: list[np.ndarray] = []
    estimate_accelerations: list[np.ndarray] = []
    for step in range(round(args.duration / args.dt) + 1):
        timestamp = step * args.dt
        position, velocity, acceleration, thrust_direction = circle_truth(
            timestamp, args.radius, args.angular_rate
        )
        measured_position = position + random_source.normal(0.0, args.bearing_noise, size=3)
        measured_thrust_direction = noisy_unit_direction(
            thrust_direction, math.radians(args.tilt_noise_deg), random_source
        )
        estimate = estimator.update(
            RelativePoseMeasurement(
                timestamp=timestamp,
                relative_position=tuple(measured_position),
                target_thrust_direction=tuple(measured_thrust_direction),
            ),
            observer_acceleration=(0.0, 0.0, 0.0),
        )
        times.append(timestamp)
        truth_positions.append(position)
        truth_velocities.append(velocity)
        truth_accelerations.append(acceleration)
        estimate_positions.append(np.asarray(estimate.relative_position))
        estimate_velocities.append(np.asarray(estimate.relative_velocity))
        estimate_accelerations.append(np.asarray(estimate.target_acceleration))
        if step < 40:
            continue
        position_error = np.asarray(estimate.relative_position) - position
        velocity_error = np.asarray(estimate.relative_velocity) - velocity
        acceleration_error = np.asarray(estimate.target_acceleration) - acceleration
        squared_position_error.append(float(position_error @ position_error))
        squared_velocity_error.append(float(velocity_error @ velocity_error))
        squared_acceleration_error.append(float(acceleration_error @ acceleration_error))
        covariance = estimator.covariance
        normalized_estimation_errors.append(
            float(position_error @ np.linalg.solve(covariance[:3, :3], position_error))
        )
    metrics = {
        "position_rmse_m": math.sqrt(float(np.mean(squared_position_error))),
        "velocity_rmse_mps": math.sqrt(float(np.mean(squared_velocity_error))),
        "acceleration_rmse_mps2": math.sqrt(float(np.mean(squared_acceleration_error))),
        "position_nees": float(np.mean(normalized_estimation_errors)),
        "minimum_covariance_eigenvalue": float(np.min(np.linalg.eigvalsh(estimator.covariance))),
    }
    history = {
        "time_s": np.asarray(times),
        "position_truth": np.asarray(truth_positions),
        "velocity_truth": np.asarray(truth_velocities),
        "acceleration_truth": np.asarray(truth_accelerations),
        "position_estimate": np.asarray(estimate_positions),
        "velocity_estimate": np.asarray(estimate_velocities),
        "acceleration_estimate": np.asarray(estimate_accelerations),
    }
    return metrics, history


def write_comparison_plots(output_dir: Path, histories: dict[str, dict[str, np.ndarray]]) -> list[Path]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot
    except ImportError:
        return []

    output_dir.mkdir(parents=True, exist_ok=True)
    truth = histories["fixed_Rh"]
    labels = {
        "fixed_Rh": "fixed $R_h$",
        "no_tilt": "no tilt constraint",
        "direction_noise": "direction-noise $R_h$",
    }
    colors = {"fixed_Rh": "tab:blue", "no_tilt": "tab:orange", "direction_noise": "tab:green"}
    paths: list[Path] = []
    figure, axes = plot.subplots(3, 3, figsize=(13, 9), sharex=True)
    for row, (quantity, unit) in enumerate(
        (("position", "m"), ("velocity", "m/s"), ("acceleration", "m/s²"))
    ):
        for column, axis_name in enumerate(("east", "north", "up")):
            axis = axes[row, column]
            axis.plot(
                truth["time_s"], truth[f"{quantity}_truth"][:, column], color="black", linewidth=1.4,
                label="truth" if row == 0 and column == 0 else None,
            )
            for name, history in histories.items():
                axis.plot(
                    history["time_s"], history[f"{quantity}_estimate"][:, column],
                    color=colors[name], linewidth=1.0, alpha=0.85,
                    label=labels[name] if row == 0 and column == 0 else None,
                )
            axis.set_title(f"{axis_name} {quantity}")
            axis.set_ylabel(f"[{unit}]")
            axis.grid(True, alpha=0.3)
    for axis in axes[-1]:
        axis.set_xlabel("time [s]")
    axes[0, 0].legend(loc="best", fontsize=8)
    figure.suptitle("Relative EKF: truth versus observer estimates")
    figure.tight_layout()
    states_path = output_dir / "truth_vs_estimate_states.png"
    figure.savefig(str(states_path), dpi=160, bbox_inches="tight")
    plot.close(figure)
    paths.append(states_path)

    figure, axis = plot.subplots(1, 1, figsize=(7, 6))
    axis.plot(truth["position_truth"][:, 0], truth["position_truth"][:, 1], color="black", linewidth=1.4, label="truth")
    for name, history in histories.items():
        axis.plot(
            history["position_estimate"][:, 0], history["position_estimate"][:, 1],
            color=colors[name], linewidth=1.0, alpha=0.85, label=labels[name],
        )
    axis.set_title("Relative horizontal trajectory")
    axis.set_xlabel("east [m]")
    axis.set_ylabel("north [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    trajectory_path = output_dir / "truth_vs_estimate_trajectory.png"
    figure.savefig(str(trajectory_path), dpi=160, bbox_inches="tight")
    plot.close(figure)
    paths.append(trajectory_path)
    return paths


def main() -> int:
    args = parse_args()
    if args.duration <= 2.0 or args.dt <= 0.0 or args.radius <= 0.0 or args.angular_rate <= 0.0:
        raise ValueError("duration、dt、radius 和 angular-rate 必须为正，且 duration 大于 2 s")
    if args.bearing_noise < 0.0 or args.tilt_noise_deg < 0.0:
        raise ValueError("噪声标准差不得为负")

    candidates = itertools.product((0.05, 0.102, 0.15), (0.015, 0.03, 0.06), (0.15, 0.2449489743, 0.4))
    results = []
    for position_std, attitude_std, acceleration_std in candidates:
        metrics = evaluate(position_std, attitude_std, acceleration_std, args)
        score = metrics["position_rmse_m"] + metrics["velocity_rmse_mps"] + metrics["acceleration_rmse_mps2"]
        results.append((score, position_std, attitude_std, acceleration_std, metrics))
    results.sort(key=lambda result: result[0])

    default_metrics, fixed_history = simulate(0.102, 0.03, 0.2449489743, args)
    no_tilt_metrics, no_tilt_history = simulate(0.102, 1_000.0, 0.2449489743, args)
    direction_results = []
    for tilt_direction_std in (0.005, 0.01, 0.02, 0.035, 0.05, 0.07):
        metrics = evaluate(0.102, 0.03, 0.2449489743, args, tilt_direction_std)
        score = metrics["position_rmse_m"] + metrics["velocity_rmse_mps"] + metrics["acceleration_rmse_mps2"]
        direction_results.append((score, tilt_direction_std, metrics))
    direction_results.sort(key=lambda result: result[0])
    selected_direction_std = direction_results[0][1]
    direction_metrics, direction_history = simulate(
        0.102, 0.03, 0.2449489743, args, selected_direction_std
    )
    plot_paths = write_comparison_plots(
        args.output_dir,
        {
            "fixed_Rh": fixed_history,
            "no_tilt": no_tilt_history,
            "direction_noise": direction_history,
        },
    )
    print("scenario: constant-range circle, physics-consistent thrust axis")
    print(f"radius={args.radius:.2f} m, angular_rate={args.angular_rate:.3f} rad/s, bearing_noise={args.bearing_noise:.3f} m, tilt_noise={args.tilt_noise_deg:.2f} deg")
    print("rank  pos_std  attitude_std  acc_process_std  p_rmse  v_rmse  a_rmse  p_nees  min_eig(P)")
    for rank, (_, position_std, attitude_std, acceleration_std, metrics) in enumerate(results[:8], start=1):
        print(
            f"{rank:>4}  {position_std:>7.3f}  {attitude_std:>12.3f}  {acceleration_std:>15.3f}"
            f"  {metrics['position_rmse_m']:>6.3f}  {metrics['velocity_rmse_mps']:>6.3f}"
            f"  {metrics['acceleration_rmse_mps2']:>6.3f}  {metrics['position_nees']:>6.3f}"
            f"  {metrics['minimum_covariance_eigenvalue']:.2e}"
        )
    print(f"default_fixed_Rh: {default_metrics}")
    print(f"no_tilt_constraint: {no_tilt_metrics}")
    print(f"selected_direction_noise_Rh(std={selected_direction_std:.3f} rad): {direction_metrics}")
    print("rank  tilt_direction_std(rad)  p_rmse  v_rmse  a_rmse  p_nees  min_eig(P)")
    for rank, (_, tilt_direction_std, metrics) in enumerate(direction_results, start=1):
        print(
            f"{rank:>4}  {tilt_direction_std:>23.3f}  {metrics['position_rmse_m']:>6.3f}"
            f"  {metrics['velocity_rmse_mps']:>6.3f}  {metrics['acceleration_rmse_mps2']:>6.3f}"
            f"  {metrics['position_nees']:>6.3f}  {metrics['minimum_covariance_eigenvalue']:.2e}"
        )
    for path in plot_paths:
        print(f"plot: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())