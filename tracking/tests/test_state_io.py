import json
import socket
import time
import unittest

from tracking.guidance import TargetState
from tracking.state_io import (
    MirroredTargetStatePublisher,
    StampedTargetState,
    TargetStatePublisher,
    TargetStateSubscriber,
)
from tracking.target_waypoints import normalize_sample_times, parse_interactive_command


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

        publisher.publish(
            TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3)),
            timestamp=10.0,
            attitude=(0.0, 0.0, 0.5, 0.5),
            reference_acceleration=(0.4, 0.5, 0.6),
            attitude_timestamp=9.85,
        )
        received = subscriber.poll()

        self.assertIsNotNone(received)
        assert received is not None
        self.assertEqual(received.timestamp, 10.0)
        self.assertEqual(received.sequence, 1)
        self.assertEqual(
            received.as_target_state(),
            TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3), a=(0.4, 0.5, 0.6)),
        )
        self.assertEqual(received.q, (0.0, 0.0, 0.5, 0.5))
        self.assertEqual(received.reference_a, (0.4, 0.5, 0.6))
        self.assertEqual(received.attitude_timestamp, 9.85)

    def test_missing_reference_acceleration_stays_zero(self) -> None:
        state = StampedTargetState(timestamp=1.0, sequence=1, p=(0.0,) * 3, v=(1.0, 0.0, 0.0))

        self.assertEqual(state.as_target_state().a, (0.0, 0.0, 0.0))

    def test_publisher_sequence_is_strictly_increasing(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        self.addCleanup(subscriber.close)
        self.addCleanup(publisher.close)

        publisher.publish(TargetState(p=(1.0, 0.0, 0.0)), timestamp=10.0)
        publisher.publish(TargetState(p=(2.0, 0.0, 0.0)), timestamp=10.1)
        received = subscriber.poll()

        self.assertIsNotNone(received)
        assert received is not None
        self.assertEqual(received.sequence, 2)
        self.assertEqual(received.timestamp, 10.1)
        self.assertIsNone(received.reference_a)
        self.assertIsNone(received.attitude_timestamp)

    def test_mirrored_publisher_reaches_control_and_shadow_ports(self) -> None:
        control_port = unused_udp_port()
        shadow_port = unused_udp_port()
        control_subscriber = TargetStateSubscriber(port=control_port)
        shadow_subscriber = TargetStateSubscriber(port=shadow_port)
        publisher = MirroredTargetStatePublisher(
            TargetStatePublisher(port=control_port),
            TargetStatePublisher(port=shadow_port),
        )
        for resource in (control_subscriber, shadow_subscriber, publisher):
            self.addCleanup(resource.close)

        publisher.publish(
            TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3)),
            timestamp=10.0,
            attitude=(0.0, 0.0, 0.5, 0.5),
            attitude_timestamp=9.85,
        )

        control_state = control_subscriber.poll()
        shadow_state = shadow_subscriber.poll()
        self.assertIsNotNone(control_state)
        self.assertEqual(control_state, shadow_state)
        self.assertEqual(shadow_state.attitude_timestamp, 9.85)

    def test_rejects_stale_state(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        self.addCleanup(subscriber.close)
        self.addCleanup(publisher.close)

        publisher.publish(TargetState(p=(0.0, 0.0, 0.0)), timestamp=time.monotonic() - 1.0)

        with self.assertRaisesRegex(RuntimeError, "target 状态超时"):
            subscriber.require_fresh(time.monotonic(), timeout=0.1)

    def test_receives_completed_terminal_event_without_overwriting_state(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        publisher = TargetStatePublisher(port=port)
        self.addCleanup(subscriber.close)
        self.addCleanup(publisher.close)

        publisher.publish(TargetState(p=(1.0, 2.0, 3.0)), timestamp=10.0)
        publisher.publish_terminal("completed", timestamp=11.0)
        received = subscriber.poll()

        self.assertIsNotNone(received)
        assert received is not None
        self.assertEqual(received.p, (1.0, 2.0, 3.0))
        self.assertIsNotNone(subscriber.terminal)
        assert subscriber.terminal is not None
        self.assertEqual(subscriber.terminal.result, "completed")
        self.assertEqual(subscriber.terminal.timestamp, 11.0)

    def test_rejects_unknown_terminal_event(self) -> None:
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(subscriber.close)
        self.addCleanup(sender.close)

        sender.sendto(json.dumps({"timestamp": 1.0, "result": "unknown"}).encode(), ("127.0.0.1", port))
        self.assertIsNone(subscriber.poll())
        self.assertEqual(subscriber.dropped, 1)

    def test_second_subscriber_on_same_port_fails_with_actionable_error(self) -> None:
        # 串行工作流最常见的启动失败就是端口被上一进程占用：错误信息必须直接
        # 给出端口号与排查命令，而不是裸 OSError。
        port = unused_udp_port()
        subscriber = TargetStateSubscriber(port=port)
        self.addCleanup(subscriber.close)
        with self.assertRaises(RuntimeError) as caught:
            TargetStateSubscriber(port=port)
        self.assertIn(str(port), str(caught.exception))
        self.assertIn("ss -ulpn", str(caught.exception))

    def test_parses_manual_waypoint_and_commands(self) -> None:
        self.assertEqual(parse_interactive_command("3 -2 1.5"), ("waypoint", (3.0, -2.0, 1.5)))
        self.assertEqual(parse_interactive_command("status"), ("status", None))
        self.assertEqual(parse_interactive_command("land"), ("land", None))
        with self.assertRaises(ValueError):
            parse_interactive_command("3 2")

    def test_normalizes_target_log_times(self) -> None:
        normalized = normalize_sample_times(
            [{"t": 100.0, "error": 0.2}, {"t": 100.25, "error": 0.1}]
        )

        self.assertEqual([sample["t"] for sample in normalized], [0.0, 0.25])

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
        sender.sendto(
            json.dumps({"timestamp": 3.0, "p": [1.0, 2.0, 3.0], "v": [0.0] * 3, "q": [0.0] * 4}).encode(),
            address,
        )
        self.assertIsNone(subscriber.poll())
        self.assertEqual(subscriber.dropped, 4)
        self.assertIsNotNone(subscriber.last_drop_reason)

        # 丢弃畸形包之后，正常报文仍应被正常接收。
        publisher.publish(TargetState(p=(1.0, 2.0, 3.0)), timestamp=5.0)
        received = subscriber.poll()

        self.assertIsNotNone(received)
        assert received is not None
        self.assertEqual(received.p, (1.0, 2.0, 3.0))
        self.assertEqual(subscriber.dropped, 4)


if __name__ == "__main__":
    unittest.main()