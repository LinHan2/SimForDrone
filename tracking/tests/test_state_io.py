import json
import socket
import time
import unittest

from tracking.guidance import TargetState
from tracking.state_io import TargetStatePublisher, TargetStateSubscriber
from tracking.target_waypoints import parse_interactive_command


def unused_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class TargetStateTransportTest(unittest.TestCase):
    def test_publishes_and_receives_latest_state(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        self.addCleanup(subscriber.close)
        self.addCleanup(publisher.close)

        publisher.publish(TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3)), timestamp=10.0)
        received = subscriber.poll()

        self.assertIsNotNone(received)
        self.assertEqual(received.timestamp, 10.0)
        self.assertEqual(received.as_target_state(), TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3)))

    def test_rejects_stale_state(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        self.addCleanup(subscriber.close)
        self.addCleanup(publisher.close)

        publisher.publish(TargetState(p=(0.0, 0.0, 0.0)), timestamp=time.monotonic() - 1.0)

        with self.assertRaisesRegex(RuntimeError, "target 状态超时"):
            subscriber.require_fresh(time.monotonic(), timeout=0.1)

    def test_parses_manual_waypoint_and_commands(self) -> None:
        self.assertEqual(parse_interactive_command("3 -2 1.5"), ("waypoint", (3.0, -2.0, 1.5)))
        self.assertEqual(parse_interactive_command("status"), ("status", None))
        self.assertEqual(parse_interactive_command("land"), ("land", None))
        with self.assertRaises(ValueError):
            parse_interactive_command("3 2")

    def test_corrupt_datagram_is_dropped_without_raising(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for resource in (subscriber, publisher, sender):
            self.addCleanup(resource.close)
        address = ("127.0.0.1", port)

        # 一个畸形报文不得终止 tracker：旧实现里 json 异常会直接穿出主循环。
        sender.sendto(b"{ not json", address)
        self.assertIsNone(subscriber.poll())
        self.assertEqual(subscriber.dropped, 1)

        # 结构合法但形状/数值非法的报文同样必须被丢弃。
        sender.sendto(json.dumps({"timestamp": 1.0, "p": [0.0, 0.0], "v": [0.0] * 3}).encode(), address)
        sender.sendto(
            json.dumps({"timestamp": 2.0, "p": [1.0, float("nan"), 3.0], "v": [0.0] * 3}).encode(),
            address,
        )
        self.assertIsNone(subscriber.poll())
        self.assertEqual(subscriber.dropped, 3)
        self.assertIsNotNone(subscriber.last_drop_reason)

        # 丢弃畸形包之后，正常报文仍应被正常接收。
        publisher.publish(TargetState(p=(1.0, 2.0, 3.0)), timestamp=5.0)
        received = subscriber.poll()

        self.assertIsNotNone(received)
        self.assertEqual(received.p, (1.0, 2.0, 3.0))
        self.assertEqual(subscriber.dropped, 3)


if __name__ == "__main__":
    unittest.main()