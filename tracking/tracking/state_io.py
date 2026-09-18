"""Local UDP transport for target state between independent test processes.

为何不用单一进程直连两台 PX4：UDP 端口（14540/14541）同一时刻只能被一个
客户端有效读取，两个进程同时连接会互相抢包。因此把"控制 target"与"控制
tracker"拆成两个进程，进程间只用本机回环传状态，不传任何飞控命令。

该模块是 V0 的临时适配层；后续换成 ROS 2 话题时只需替换本文件，
`guidance` 与 `px4ctrl` 均无需修改。
"""

from __future__ import annotations

import json
import socket
import time
from dataclasses import asdict, dataclass

from tracking.guidance import TargetState, Vector3

DEFAULT_STATE_HOST = "127.0.0.1"
DEFAULT_STATE_PORT = 14600


@dataclass(frozen=True)
class StampedTargetState:
    """带上发布时刻的目标状态，用于接收端判定新鲜度。"""

    # 注意时间基准：timestamp 取发布进程的 time.monotonic()，两进程同机运行，
    # 所以该时刻与订阅端可直接相减；跨机部署时需换为统一时间源。
    timestamp: float
    p: Vector3
    v: Vector3

    def as_target_state(self) -> TargetState:
        return TargetState(p=self.p, v=self.v)


class TargetStatePublisher:
    def __init__(self, host: str = DEFAULT_STATE_HOST, port: int = DEFAULT_STATE_PORT) -> None:
        self._destination = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, state: TargetState, timestamp: float | None = None) -> None:
        message = StampedTargetState(
            timestamp=time.monotonic() if timestamp is None else timestamp,
            p=state.p,
            v=state.v,
        )
        self._socket.sendto(json.dumps(asdict(message), separators=(",", ":")).encode(), self._destination)

    def close(self) -> None:
        self._socket.close()


class TargetStateSubscriber:
    def __init__(self, host: str = DEFAULT_STATE_HOST, port: int = DEFAULT_STATE_PORT) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._socket.setblocking(False)
        self.latest: StampedTargetState | None = None

    def poll(self) -> StampedTargetState | None:
        """非阻塞排空收包队列，返回最新一帧（无新包时返回上一次缓存）。"""

        while True:
            try:
                payload, _address = self._socket.recvfrom(4096)
            except BlockingIOError:
                # 队列已空。返回缓存而非 None，避免调用方把"本轮无新包"误判为丢包。
                return self.latest
            # 只保留最后一帧：跟踪控制用的是当前目标状态，积压的旧帧没有价值。
            data = json.loads(payload.decode())
            self.latest = StampedTargetState(
                timestamp=float(data["timestamp"]),
                p=tuple(float(value) for value in data["p"]),
                v=tuple(float(value) for value in data["v"]),
            )

    def require_fresh(self, now: float, timeout: float) -> StampedTargetState:
        """要求状态在 timeout 内更新过，否则报错。

        安全关键：target 进程崩溃或暂停时，陈旧目态会让 tracker 持续飞向
        一个不再存在的目标；上层捕获此异常后会转入下降。
        """

        state = self.poll()
        if state is None:
            raise RuntimeError("尚未收到 target 状态；请先启动 target 航点任务")
        if now - state.timestamp > timeout:
            raise RuntimeError(f"target 状态超时 {now - state.timestamp:.3f}s（门限 {timeout:.3f}s）")
        return state

    def close(self) -> None:
        self._socket.close()
