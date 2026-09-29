"""场景环境与光照配置的离线回归。

这些测试不启动 Isaac：只校验“配置选择”和“参数校验”这两层纯逻辑。
它们保护的是本次修复的核心失败模式——YAML 看起来换了场景，但实际仍渲染空网格；
以及光照参数写错后被静默忽略、画面继续偏暗。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from simfordrone.industrial_hangar import resolve_environment_asset  # noqa: E402
from simfordrone.lighting import validate_lighting_config  # noqa: E402


FAKE_ENVIRONMENTS = {
    "Default Environment": "https://example.invalid/Grid/default_environment.usd",
    "Full Warehouse": "https://example.invalid/Simple_Warehouse/full_warehouse.usd",
}

#: 空网格预设名。出现在配置里就意味着场景没有物体，图像闭环不可能成立。
EMPTY_GRID_PRESET = "Default Environment"


class ResolveEnvironmentAssetTest(unittest.TestCase):
    def test_preset_and_asset_url_are_mutually_exclusive(self) -> None:
        with self.assertRaises(ValueError):
            resolve_environment_asset(
                {"preset": "Full Warehouse", "asset_url": "https://example.invalid/a.usd"},
                FAKE_ENVIRONMENTS,
            )

    def test_requires_one_asset_source(self) -> None:
        with self.assertRaises(ValueError):
            resolve_environment_asset({}, FAKE_ENVIRONMENTS)

    def test_unknown_preset_lists_available_options(self) -> None:
        with self.assertRaises(ValueError) as context:
            resolve_environment_asset({"preset": "Not A Scene"}, FAKE_ENVIRONMENTS)
        message = str(context.exception)
        self.assertIn("Not A Scene", message)
        self.assertIn("Full Warehouse", message)

    def test_preset_resolves_through_injected_table(self) -> None:
        label, url = resolve_environment_asset({"preset": "Full Warehouse"}, FAKE_ENVIRONMENTS)
        self.assertEqual(label, "Full Warehouse")
        self.assertTrue(url.endswith("full_warehouse.usd"))

    def test_explicit_asset_url_passes_through(self) -> None:
        label, url = resolve_environment_asset(
            {"asset_url": "/abs/path/custom.usd"}, FAKE_ENVIRONMENTS
        )
        self.assertEqual(label, "/abs/path/custom.usd")
        self.assertEqual(url, "/abs/path/custom.usd")

    def test_unknown_environment_key_is_rejected(self) -> None:
        # 拼错的键必须报错：此前正是“改了 YAML 但键名不生效”才留下空场景。
        with self.assertRaises(ValueError):
            resolve_environment_asset(
                {"preset": "Full Warehouse", "asset_uri": "typo"}, FAKE_ENVIRONMENTS
            )


class LightingConfigTest(unittest.TestCase):
    def test_requires_at_least_one_enabled_light(self) -> None:
        with self.assertRaises(ValueError):
            validate_lighting_config({"dome": {"enabled": False}})

    def test_rejects_unknown_key(self) -> None:
        with self.assertRaises(ValueError):
            validate_lighting_config({"dome": {"enabled": True, "intensity": 10.0, "lux": 5.0}})

    def test_rejects_non_positive_intensity(self) -> None:
        with self.assertRaises(ValueError):
            validate_lighting_config({"dome": {"enabled": True, "intensity": 0.0}})

    def test_rect_light_requires_prim_path_and_intensity(self) -> None:
        with self.assertRaises(ValueError):
            validate_lighting_config(
                {"dome": {"enabled": True, "intensity": 1.0}, "rect_lights": [{"intensity": 5.0}]}
            )
        with self.assertRaises(ValueError):
            validate_lighting_config(
                {
                    "dome": {"enabled": True, "intensity": 1.0},
                    "rect_lights": [{"prim_path": "/World/lights/fill"}],
                }
            )

    def test_accepts_full_configuration(self) -> None:
        validate_lighting_config(
            {
                "root": "/World/lights",
                "dome": {"enabled": True, "intensity": 1000.0, "color": [1.0, 1.0, 1.0]},
                "sun": {
                    "enabled": True,
                    "intensity": 1500.0,
                    "color": [1.0, 0.97, 0.92],
                    "angle_deg": 1.5,
                    "elevation_deg": 55.0,
                    "azimuth_deg": 135.0,
                },
                "rect_lights": [
                    {
                        "prim_path": "/World/lights/fill_south",
                        "intensity": 900.0,
                        "size": [8.0, 8.0],
                        "position": [-4.0, -4.0, 6.0],
                    }
                ],
            }
        )


class ShippedConfigTest(unittest.TestCase):
    """出厂 YAML 必须指向有物体的环境，且光照配置完整。"""

    def setUp(self) -> None:
        try:
            import yaml
        except ImportError:  # pragma: no cover - 取决于运行环境
            self.skipTest("PyYAML 不可用，跳过出厂配置检查")
        config_path = PROJECT_ROOT / "configs" / "dual_uav_hangar.yaml"
        with config_path.open(encoding="utf-8") as handle:
            self.config = yaml.safe_load(handle)

    def test_environment_is_not_the_empty_grid(self) -> None:
        environment = self.config["environment"]
        label, url = resolve_environment_asset(environment, FAKE_ENVIRONMENTS)
        self.assertNotEqual(label, EMPTY_GRID_PRESET)
        self.assertNotIn("Grid/default_environment.usd", url)

    def test_lighting_section_is_valid(self) -> None:
        validate_lighting_config(self.config["environment"]["lighting"])


if __name__ == "__main__":
    unittest.main()
