import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("tracking_experiment", ROOT / "scripts/tracking_experiment.py")
experiment = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(experiment)


class ExperimentTest(unittest.TestCase):
    def test_mocked_sequence_releases_only_after_both_controller_launches(self):
        self.check_sequence(tracker_exit_code=0)

    def test_failed_controller_keeps_scene_running(self):
        self.check_sequence(tracker_exit_code=1)

    def check_sequence(self, tracker_exit_code):
        launched = []
        config_path = ROOT / "tracking/config/experiment.yaml"
        with tempfile.TemporaryDirectory() as directory:
            temporary_root = Path(directory)

            class Process:
                def __init__(self, role):
                    self.role = role
                    self.pid = 100 + len(launched)
                    self.returncode = None
                    text = {"target": "target 已悬停\n", "tracker": "TRACK t=0\n",
                            "pixel_bridge": ""}[role]
                    self.stdout = io.StringIO(text)

                def poll(self):
                    if self.role == "pixel_bridge":
                        return self.returncode
                    if list(temporary_root.glob("logs/experiments/*/trajectory.release")):
                        return tracker_exit_code if self.role == "tracker" else 0
                    return self.returncode

                def wait(self, timeout):
                    self.returncode = tracker_exit_code if self.role == "tracker" else 0
                    return self.returncode

            def launch(command, **kwargs):
                role = {"run_target_waypoints.sh": "target",
                    "run_tracker.sh": "tracker",
                    "run_image_pixel_bridge.sh": "pixel_bridge"}[Path(command[0]).name]
                launched.append(role)
                self.assertFalse(list(temporary_root.glob("logs/experiments/*/trajectory.release")))
                return Process(role)

            with patch.object(experiment, "ROOT", temporary_root), patch.object(experiment.subprocess, "Popen", side_effect=launch), patch.object(experiment.subprocess, "run", return_value=type("Result", (), {"returncode": 1})()), patch.object(experiment.os, "killpg") as killpg, patch("sys.argv", ["experiment", "--config", str(config_path)]):
                self.assertEqual(experiment.main(), tracker_exit_code)
                self.assertEqual(killpg.call_count, 1)
            self.assertEqual(launched, ["pixel_bridge", "target", "tracker"])
            self.assertEqual(len(list(temporary_root.glob("logs/experiments/*/trajectory.release"))), 1)

    def test_default_commands_are_accepted_by_existing_parsers(self):
        from tracking.target_waypoints import parse_args as target_args
        from tracking.run_tracker import parse_args as tracker_args

        config, commands = experiment.load_commands(ROOT / "tracking/config/experiment.yaml", ROOT / "logs/test/release")
        with patch("sys.argv", commands["target"]):
            target = target_args()
        with patch("sys.argv", commands["tracker"]):
            tracker = tracker_args()
        self.assertEqual(target.start_delay, 0)
        self.assertEqual(target.start_gate_file, ROOT / "logs/test/release")
        self.assertEqual(tracker.state_source, "truth")
        self.assertEqual(tracker.duration, 0)
        self.assertFalse(tracker.visibility_optimization)
        self.assertEqual(tracker.visual_tracking_mode, "predictive")
        self.assertEqual(set(commands), {"target", "tracker", "pixel_bridge"})

    def test_dry_run_never_starts_processes(self):
        with patch("sys.argv", ["experiment", "--dry-run"]), patch.object(experiment.subprocess, "Popen") as popen, patch.object(experiment.subprocess, "run") as run:
            self.assertEqual(experiment.main(), 0)
        popen.assert_not_called()
        run.assert_not_called()

    def test_invalid_handshake_settings_are_rejected(self):
        config = yaml.safe_load((ROOT / "tracking/config/experiment.yaml").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for key, value in (("duration", 60), ("status_period", 0)):
                with self.subTest(key=key):
                    modified = {**config, "tracker": {**config["tracker"], key: value}}
                    path.write_text(yaml.safe_dump(modified))
                    with self.assertRaises(ValueError):
                        experiment.load_commands(path, Path(directory) / "release")