"""target 航点任务的无阻塞诊断与输入解析测试。"""

import queue
import unittest

from tracking.target_waypoints import parse_interactive_command, put_latest


class TargetWaypointTest(unittest.TestCase):
    def test_diagnostics_keep_only_the_latest_message(self) -> None:
        messages: queue.Queue[str] = queue.Queue(maxsize=1)

        put_latest(messages, "old")
        put_latest(messages, "new")

        self.assertEqual(messages.get_nowait(), "new")
        self.assertTrue(messages.empty())

    def test_parse_interactive_waypoint(self) -> None:
        self.assertEqual(parse_interactive_command("1 2 3"), ("waypoint", (1.0, 2.0, 3.0)))


if __name__ == "__main__":
    unittest.main()