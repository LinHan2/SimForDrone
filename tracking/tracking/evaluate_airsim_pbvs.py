"""Offline radial approach check using Airsim2box PBVS and px4ctrl.LinearControl.

Uses a scalar acceleration plant; not an Isaac, PX4 firmware, or visual-EKF validation.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml


CONTROLLER_PATH = Path(__file__).resolve().parents[2] / "Airsim2box/src/control/tracker.py"
SIM_CONFIG = Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"
TRACKING_CONFIG = Path(__file__).resolve().parents[1] / "config/tracker.yaml"


def pbvs_parameters(path: Path = TRACKING_CONFIG) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = config["pbvs_shadow"]
    if not isinstance(params, dict):
        raise ValueError("pbvs_shadow must be a mapping")
    return params


def read_control_samples(path: Path) -> list[dict]:
    record = json.loads(path.read_text(encoding="utf-8"))
    samples = record.get("control_samples")
    if not samples:
        raise ValueError("No full-rate CMD_CTRL control_samples; cannot identify delay from this log")
    return samples


def plot_log_diagnostics(path: Path, output: Path) -> bool:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    record = json.loads(path.read_text(encoding="utf-8"))
    control_samples = record.get("control_samples") or []
    fig, axes = plt.subplots(3 if control_samples else 2, 1,
                             figsize=(10, 9 if control_samples else 7), sharex=True)
    if control_samples:
        times = [sample["t"] for sample in control_samples]
        axes[0].plot(times, [math.hypot(*(sample["reference_p"][index] - sample["odom_p"][index]
                                          for index in (0, 1))) for sample in control_samples],
                     label="horizontal position error")
        for axis_index, direction in enumerate(("east", "north")):
            axes[1].plot(times, [sample["reference_v"][axis_index] for sample in control_samples],
                         linestyle="--", label=f"reference {direction}")
            axes[1].plot(times, [sample["odom_v"][axis_index] for sample in control_samples],
                         label=f"odom {direction}")
            if all(sample.get("pbvs_shadow_v") is not None for sample in control_samples):
                axes[1].plot(times, [sample["pbvs_shadow_v"][axis_index] for sample in control_samples],
                             linestyle=":", label=f"PBVS shadow {direction}")
        axes[0].set_ylabel("horizontal position error (m)")
        axes[1].set_ylabel("local ENU velocity (m/s)")
        axes[2].plot(times, [sample["t"] - sample["odom_recv_t"] for sample in control_samples],
                     label="odom receive age")
        axes[2].set_ylabel("odom receive age (s)")
    else:
        samples = record.get("samples") or []
        if not samples:
            raise ValueError("Log has no tracking samples to plot")
        times = [sample["t"] for sample in samples]
        axes[0].plot(times, [sample["horizontal_distance"] for sample in samples], label="actual distance")
        axes[0].plot(times, [sample["desired_distance"] for sample in samples], label="desired distance")
        axes[0].axhline(5.0, color="black", linestyle=":", label="5 m target")
        axes[0].set_ylabel("horizontal distance (m)")
        axes[1].step(times, [sample["fsm_cmd_ctrl"] for sample in samples], where="post", label="CMD_CTRL active")
        axes[1].set_ylabel("control state")
        fig.suptitle("Historical log: no full-rate control samples; delay unidentifiable")
    axes[-1].set_xlabel("tracking time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output), dpi=150)
    plt.close(fig)
    return bool(control_samples)


def plot_pitch_diagnostics(path: Path, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    samples = read_control_samples(path)
    if not all("output_q" in sample for sample in samples):
        raise ValueError("No attitude commands in control_samples")

    def forward_elevation(quaternion) -> float:
        x, y, z, w = quaternion
        forward_up = 2 * (x * z - w * y)
        forward_east = 1 - 2 * (y * y + z * z)
        forward_north = 2 * (x * y + w * z)
        return math.degrees(math.atan2(forward_up, math.hypot(forward_east, forward_north)))

    times = [sample["t"] for sample in samples]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, [forward_elevation(sample["output_q"]) for sample in samples],
                 label="commanded camera-forward elevation")
    if all("odom_q" in sample for sample in samples):
        axes[0].plot(times, [forward_elevation(sample["odom_q"]) for sample in samples],
                     label="measured camera-forward elevation")
    axes[0].set_ylabel("forward elevation (deg)")
    axes[1].plot(times, [sample["odom_p"][2] for sample in samples], label="measured altitude")
    axes[1].plot(times, [sample["reference_p"][2] for sample in samples],
                 linestyle="--", label="V0 altitude reference")
    axes[1].set_ylabel("local ENU altitude (m)")
    axes[1].set_xlabel("tracking time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output), dpi=150)
    plt.close(fig)


def measured_attitude_samples(controls: list[dict]) -> list[dict]:
    return [sample for index, sample in enumerate(controls)
            if "attitude_recv_monotonic_s" not in sample or
            (sample["attitude_recv_monotonic_s"] > 0 and
             (index == 0 or sample["attitude_recv_monotonic_s"] != controls[index - 1]["attitude_recv_monotonic_s"]))]


def plot_fov_alignment(tracker_path: Path, shadow_path: Path, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    controls = read_control_samples(tracker_path)
    frames = json.loads(shadow_path.read_text(encoding="utf-8")).get("fov_samples", [])
    if not frames or not all("monotonic_s" in sample and "odom_q" in sample for sample in controls):
        raise ValueError("Need synchronized tracker attitude and shadow FOV samples")
    start = controls[0]["monotonic_s"]
    controls = [sample for sample in controls if frames[0]["monotonic_s"] <= sample["monotonic_s"] <= frames[-1]["monotonic_s"]]
    if not controls:
        raise ValueError("Tracker control and shadow FOV timestamps do not overlap")

    def forward_elevation(quaternion) -> float:
        x, y, z, w = quaternion
        return math.degrees(math.atan2(2 * (x * z - w * y),
                             math.hypot(1 - 2 * (y * y + z * z), 2 * (x * y + w * z))))

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    measured = measured_attitude_samples(controls)
    axes[0].plot([sample.get("attitude_recv_monotonic_s", sample["monotonic_s"]) - start
                  for sample in measured],
                 [forward_elevation(sample["odom_q"]) for sample in measured],
                 label="measured forward elevation (attitude receipt)" if "attitude_recv_monotonic_s" in controls[0]
                 else "measured forward elevation (control loop, legacy log)")
    axes[0].plot([sample["monotonic_s"] - start for sample in controls],
                 [forward_elevation(sample["output_q"]) for sample in controls], alpha=0.6,
                 label="commanded forward elevation")
    axes[0].set_ylabel("forward elevation (deg)")
    for reason, marker in (("ready", "o"), ("target_outside_image", "x")):
        selected = [frame for frame in frames if frame["reason"] == reason and frame["pixel_v"] is not None]
        axes[1].scatter([frame["monotonic_s"] - start for frame in selected],
                        [frame["pixel_v"] / frame["image_height"] for frame in selected],
                        s=9, marker=marker, label=reason)
    axes[1].axhspan(0, 1, color="green", alpha=0.08, label="image height")
    axes[1].set_ylabel("projected vertical pixel / height")
    axes[1].set_xlabel("tracker time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output), dpi=150)
    plt.close(fig)


def load_controller():
    spec = importlib.util.spec_from_file_location("airsim_pbvs_controller", CONTROLLER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load controller: {CONTROLLER_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RelativeTrackingController


def shadow_velocity(controller, target_p, target_v, observer_p, observer_v) -> tuple[float, float, float]:
    relative_p = np.asarray(target_p, dtype=float) - np.asarray(observer_p, dtype=float)
    own_v = np.asarray(observer_v, dtype=float)
    relative_v = np.asarray(target_v, dtype=float) - own_v
    return tuple(float(value) for value in controller.compute(relative_p, relative_v, own_v))


def px4_acceleration(controller, position: float, velocity: float, reference: float,
                     desired_velocity: float) -> tuple[float, bool]:
    from px4ctrl.controller import DesiredState
    from px4ctrl.frames import quat_rotate
    from px4ctrl.inputs import ImuData, OdomData

    output = controller.calculate_control(
        DesiredState(p=(reference, 0.0, 2.0), v=(desired_velocity, 0.0, 0.0)),
        OdomData(p=(position, 0.0, 2.0), v=(velocity, 0.0, 0.0)),
        ImuData(),
    )
    thrust_acceleration = output.thrust * controller.params.gra / controller.params.thrust_model.hover_percentage
    return thrust_acceleration * quat_rotate(output.q, (0.0, 0.0, 1.0))[0], controller.debug.tilt_saturated


def simulate(initial_distance: float, duration: float, dt: float, response_seconds: float,
             trace: list[tuple[float, float, float, float]] | None = None,
             kp: float = 1.4, kd: float = 0.22) -> dict:
    if initial_distance <= 0 or duration <= 0 or dt <= 0 or response_seconds < 0:
        raise ValueError("distance, duration and dt must be positive; response must be nonnegative")
    if kp < 0 or kd < 0:
        raise ValueError("controller gains must be nonnegative")
    controller = load_controller()(
        kp=kp, kd=kd, ki_rad=0.03, i_rad_limit=1.2,
        follow_dist=5.0, min_follow_dist=4.0, max_follow_dist=6.2,
        kv_tan=0.40, kv_ff_abs=0.20, max_speed=5.0,
        max_accel=3.2, lpf_alpha=0.76, dt=dt,
    )
    position = np.array([-initial_distance, 0.0, 0.0])
    velocity = np.zeros(3)
    minimum_distance = initial_distance
    maximum_speed = 0.0
    gate_time = None
    tail_error = 0.0
    if trace is not None:
        trace.append((0.0, initial_distance, 0.0, 0.0))
    for step in range(int(duration / dt)):
        relative_position = -position
        relative_velocity = -velocity
        command = controller.compute(relative_position, relative_velocity, velocity)
        if response_seconds == 0:
            velocity = command
        else:
            velocity += (command - velocity) * min(1.0, dt / response_seconds)
        position += velocity * dt
        distance = float(np.linalg.norm(position[:2]))
        minimum_distance = min(minimum_distance, distance)
        maximum_speed = max(maximum_speed, float(np.linalg.norm(velocity[:2])))
        if trace is not None:
            trace.append(((step + 1) * dt, distance, float(velocity[0]), float(command[0])))
        if gate_time is None and abs(distance - 5.0) <= 0.5:
            gate_time = (step + 1) * dt
        if (step + 1) * dt >= duration - 20.0:
            tail_error = max(tail_error, abs(distance - 5.0))
    return {
        "kp": kp,
        "kd": kd,
        "response_seconds": response_seconds,
        "final_distance_m": round(float(np.linalg.norm(position[:2])), 3),
        "minimum_distance_m": round(minimum_distance, 3),
        "maximum_speed_mps": round(maximum_speed, 3),
        "tail_max_error_m": round(tail_error, 3),
        "first_gate_s": round(gate_time, 2) if gate_time is not None else None,
    }


def simulate_px4(initial_distance: float, duration: float, dt: float, response_seconds: float,
                 delay_seconds: float = 0.0, kp: float | None = None, kd: float | None = None,
                 trace: list[tuple[float, float, float, float]] | None = None,
                 inner_gain_scale: float = 1.0, tracking_config: Path = TRACKING_CONFIG) -> dict:
    sys.path.insert(0, str(SIM_CONFIG.parents[2] / "px4ctrl"))
    from px4ctrl.controller import LinearControl
    from px4ctrl.params import load_params

    if min(initial_distance, duration, dt, inner_gain_scale) <= 0 or min(response_seconds, delay_seconds) < 0:
        raise ValueError("distance, duration, dt must be positive; lag and delay must be nonnegative")
    params_pbvs = pbvs_parameters(tracking_config)
    if kp is not None:
        params_pbvs["kp"] = kp
    if kd is not None:
        params_pbvs["kd"] = kd
    pbvs = load_controller()(**params_pbvs, dt=dt)
    params = load_params(SIM_CONFIG)
    gains = replace(params.gain, kp0=params.gain.kp0 * inner_gain_scale,
                    kv0=params.gain.kv0 * inner_gain_scale)
    controller = LinearControl(replace(params, gain=gains, max_angle=20.0))
    position = -initial_distance
    reference = position
    velocity = acceleration = 0.0
    delay_steps = round(delay_seconds / dt)
    delayed = deque([0.0] * delay_steps)
    minimum_distance = initial_distance
    tail_error = 0.0
    gate_time = None
    saturated = 0
    if trace is not None:
        trace.append((0.0, initial_distance, 0.0, 0.0))
    for step in range(int(duration / dt)):
        command = float(pbvs.compute(
            np.array([-position, 0.0, 0.0]), np.array([-velocity, 0.0, 0.0]),
            np.array([velocity, 0.0, 0.0]),
        )[0])
        reference += command * dt
        requested, tilt_saturated = px4_acceleration(controller, position, velocity, reference, command)
        saturated += tilt_saturated
        if delay_steps:
            delayed.append(requested)
            requested = delayed.popleft()
        acceleration += (requested - acceleration) * (1.0 if response_seconds == 0 else min(1.0, dt / response_seconds))
        velocity += acceleration * dt
        position += velocity * dt
        distance = abs(position)
        minimum_distance = min(minimum_distance, distance)
        elapsed = (step + 1) * dt
        if trace is not None:
            trace.append((elapsed, distance, velocity, command))
        if gate_time is None and abs(distance - 5.0) <= 0.5:
            gate_time = elapsed
        if elapsed >= duration - 20.0:
            tail_error = max(tail_error, abs(distance - 5.0))
    return {
        "kp": pbvs.kp, "kd": pbvs.kd, "lag_s": response_seconds, "delay_s": delay_seconds,
        "inner_gain_scale": inner_gain_scale,
        "final_distance_m": round(abs(position), 3),
        "minimum_distance_m": round(minimum_distance, 3),
        "tail_max_error_m": round(tail_error, 3),
        "tilt_saturated_steps": saturated,
        "first_gate_s": round(gate_time, 2) if gate_time is not None else None,
    }


def plot_responses(traces: list[tuple[float | str, list[tuple[float, float, float, float]]]], output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for response_seconds, trace in traces:
        times, distances, speeds, commands = zip(*trace)
        label = (f"response={response_seconds:g}s" if isinstance(response_seconds, float)
             else response_seconds)
        axes[0].plot(times, distances, label=label)
        axes[1].plot(times, speeds, label=label)
        axes[1].plot(times, commands, linestyle="--", alpha=0.5)
    axes[0].axhline(5.0, color="black", linestyle=":", label="5m target")
    axes[0].axhspan(4.5, 5.5, color="green", alpha=0.08)
    axes[0].set_ylabel("horizontal distance (m)")
    axes[0].legend()
    axes[1].set_ylabel("east velocity (m/s)\nsolid: actual, dashed: command")
    axes[1].set_xlabel("time (s)")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output), dpi=150)
    plt.close(fig)


def tune(initial_distance: float, duration: float, dt: float, responses: list[float]) -> tuple[tuple[float, float], list[dict]]:
    candidates = []
    for kp in (0.5, 0.8, 1.1, 1.4):
        for kd in (0.22, 0.4, 0.7):
            results = [simulate(initial_distance, duration, dt, response, kp=kp, kd=kd)
                       for response in responses]
            worst_tail = max(result["tail_max_error_m"] for result in results)
            worst_minimum = min(result["minimum_distance_m"] for result in results)
            slowest_gate = max((result["first_gate_s"] or duration) for result in results)
            # Keep at least 4.5 m in every modeled response; prefer a settled, then prompt approach.
            score = (worst_minimum < 4.5, worst_tail, slowest_gate)
            candidates.append((score, kp, kd, results))
    _, kp, kd, results = min(candidates, key=lambda candidate: candidate[0])
    return (kp, kd), results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-distance", type=float, default=21.38)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--dt", type=float, default=1.0 / 60.0)
    parser.add_argument("--response-seconds", type=float, nargs="+", default=[0.0, 0.3, 1.0, 2.0])
    parser.add_argument("--tune", action="store_true", help="Compare radial kp/kd across all response models")
    parser.add_argument("--px4-loop", action="store_true", help="Compare px4ctrl.LinearControl in an offline delayed acceleration model")
    parser.add_argument("--tracking-config", type=Path, default=TRACKING_CONFIG,
                        help="PBVS shadow settings used in offline cascade")
    parser.add_argument("--inspect-log", type=Path, help="Plot control diagnostics from an existing tracker run.json (no flight)")
    parser.add_argument("--inspect-shadow", type=Path, help="Align an Isaac shadow FOV JSON with --inspect-log")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "logs/tracking/offline_pbvs/response.png")
    args = parser.parse_args()
    if args.inspect_shadow is not None:
        if args.inspect_log is None:
            parser.error("--inspect-shadow requires --inspect-log")
        output = args.output.with_name("isaac_fov_alignment.png")
        plot_fov_alignment(args.inspect_log, args.inspect_shadow, output)
        print(f"Isaac FOV alignment plot: {output}")
        return
    if args.inspect_log is not None:
        output = args.output.with_name("control_log_diagnostics.png")
        full_rate = plot_log_diagnostics(args.inspect_log, output)
        print(f"Diagnostic plot: {output}")
        if full_rate and all("output_q" in sample for sample in read_control_samples(args.inspect_log)):
            pitch_output = output.with_name("pitch_altitude_diagnostics.png")
            plot_pitch_diagnostics(args.inspect_log, pitch_output)
            print(f"Pitch/altitude plot: {pitch_output}")
        if full_rate:
            print("CMD_CTRL samples present; validate timestamps and excitation before fitting")
        else:
            print("Delay unidentifiable: no full-rate CMD_CTRL samples")
        return
    traces = []
    for response_seconds in args.response_seconds:
        trace = []
        print(simulate(args.initial_distance, args.duration, args.dt, response_seconds, trace))
        traces.append((response_seconds, trace))
    plot_responses(traces, args.output)
    print(f"Response plot: {args.output}")
    if args.tune:
        (kp, kd), results = tune(args.initial_distance, args.duration, args.dt, args.response_seconds)
        print(f"Best offline candidate: kp={kp:g} kd={kd:g}")
        tuned_traces = []
        for response_seconds, result in zip(args.response_seconds, results):
            trace = []
            simulate(args.initial_distance, args.duration, args.dt, response_seconds, trace, kp, kd)
            tuned_traces.append((response_seconds, trace))
            print(result)
        tuned_output = args.output.with_name("tuned_response.png")
        plot_responses(tuned_traces, tuned_output)
        print(f"Tuned response plot: {tuned_output}")
    if args.px4_loop:
        cases = ((0.0, 0.0, 1.0), (0.3, 0.0, 1.0), (0.3, 0.15, 1.0), (0.3, 0.15, 0.3))
        fig_traces = []
        for lag, delay, scale in cases:
            trace = []
            result = simulate_px4(args.initial_distance, args.duration, args.dt, lag, delay,
                                  trace=trace, inner_gain_scale=scale, tracking_config=args.tracking_config)
            print(result)
            fig_traces.append((f"lag={lag:g}s delay={delay:g}s gains={scale:g}x", trace))
        loop_output = args.output.with_name("px4_loop_response.png")
        plot_responses(fig_traces, loop_output)
        print(f"PX4 loop response plot: {loop_output}")


if __name__ == "__main__":
    main()