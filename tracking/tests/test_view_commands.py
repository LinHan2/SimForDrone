"""视角命令解析的单元测试。

本测试只覆盖纯函数与模式切换逻辑，不导入 Isaac：
`simfordrone.view_control` 把 Isaac 相关导入放在方法内部，正是为了这一点。

将仓库 ``src/`` 加入 ``sys.path``，使算法测试与场景代码共用一个可测入口。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parents[2] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from simfordrone.view_control import (  # noqa: E402  (需先设置 sys.path)
    ALL_MODES,
    FOLLOW_TARGET,
    FOLLOW_TRACKER,
    FREE,
    ONBOARD_TRACKER,
    OVERVIEW,
    ViewControl,
    parse_view_command,
)


class FakeState:
    """最小载具状态替身：只提供跟随相机需要的 position。"""

    def __init__(self, position: tuple[float, float, float]) -> None:
        self.position = position


class FakeVehicle:
    def __init__(self, position: tuple[float, float, float]) -> None:
        self.state = FakeState(position)


def make_vehicles() -> dict[str, dict]:
    return {
        "target": {"vehicle": FakeVehicle((3.0, 0.0, 2.0))},
        "tracker": {"vehicle": FakeVehicle((-3.0, 0.0, 2.0))},
    }


class ParseViewCommandTest(unittest.TestCase):
    def test_accepts_numbers_and_words(self) -> None:
        self.assertEqual(parse_view_command("3"), FOLLOW_TRACKER)
        self.assertEqual(parse_view_command("tracker"), FOLLOW_TRACKER)
        self.assertEqual(parse_view_command("  CAMERA \n"), ONBOARD_TRACKER)
        self.assertEqual(parse_view_command("1"), OVERVIEW)
        self.assertEqual(parse_view_command("2"), FOLLOW_TARGET)

    def test_free_mode_is_reachable_and_default(self) -> None:
        # 手动模式必须可识别，且是默认：否则相机会被程序每帧覆盖，鼠标无法调整。
        self.assertEqual(parse_view_command("0"), FREE)
        self.assertEqual(parse_view_command("free"), FREE)
        self.assertEqual(ViewControl(make_vehicles()).mode, FREE)

    def test_rejects_unknown_command(self) -> None:
        self.assertIsNone(parse_view_command("fly"))
        self.assertIsNone(parse_view_command(""))


class ViewControlTest(unittest.TestCase):
    def test_set_mode_rejects_invalid_mode(self) -> None:
        control = ViewControl(make_vehicles(), initial_mode=OVERVIEW)

        self.assertFalse(control.set_mode("teleport"))
        self.assertEqual(control.mode, OVERVIEW)

    def test_onboard_falls_back_when_camera_missing(self) -> None:
        # 没有机载相机时必须退回**手动**视角，而不是停在错误相机或让别人接管相机。
        control = ViewControl(make_vehicles(), initial_mode=FOLLOW_TRACKER)

        control.set_mode(ONBOARD_TRACKER)
        control.apply()

        self.assertEqual(control.mode, FREE)

    def test_status_line_lists_both_vehicles(self) -> None:
        control = ViewControl(make_vehicles(), initial_mode=FOLLOW_TRACKER)

        line = control.status_line()

        self.assertIn("view=follow_tracker", line)
        self.assertIn("target=(+3.00,+0.00,+2.00)", line)
        self.assertIn("tracker=(-3.00,+0.00,+2.00)", line)

    def test_all_modes_are_reachable(self) -> None:
        control = ViewControl(make_vehicles(), initial_mode=OVERVIEW)

        for mode in ALL_MODES:
            self.assertTrue(control.set_mode(mode), mode)


if __name__ == "__main__":
    unittest.main()
