"""独立测试进程之间传递目标状态的本地 UDP 通道。

为何不用单一进程直连两台 PX4：UDP 端口（14540/14541）同一时刻只能被一个
客户端有效读取，两个进程同时连接会互相抢包。因此把"控制 target"与"控制
tracker"拆成两个进程，进程间只用本机回环传状态，不传任何飞控命令。

该模块是 V0 的临时适配层；后续换成 ROS 2 话题时只需替换本文件，
`guidance` 与 `px4ctrl` 均无需修改。
"""

from __future__ import annotations

import json
import math
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
        #: 被丢弃的畸形报文计数与最后一条原因。UDP 无连接、无顺序、无校验，
        #: 截断或错位的数据报是常态；计数用于区分“网络问题”与“目标进程没在发”。
        self.dropped = 0
        self.last_drop_reason: str | None = None

    def poll(self) -> StampedTargetState | None:
        """非阻塞排空收包队列，返回最新一帧（无新包时返回上一次缓存）。

        单个畸形报文不得终止调用方：旧实现里 ``json.loads`` 的异常会直接穿出 tracker
        的主循环，一个坏包就能让整个跟踪任务带着异常退出。
        """

        while True:
            try:
                payload, _address = self._socket.recvfrom(4096)
            except BlockingIOError:
                # 队列已空。返回缓存而非 None，避免调用方把"本轮无新包"误判为丢包。
                return self.latest
            try:
                state = self._decode(payload)
            except (ValueError, TypeError, KeyError, UnicodeDecodeError) as error:
                self.dropped += 1
                self.last_drop_reason = f"{type(error).__name__}: {error}"
                continue
            # 只保留最后一帧：跟踪控制用的是当前目标状态，积压的旧帧没有价值。
            self.latest = state

    @staticmethod
    def _decode(payload: bytes) -> StampedTargetState:
        """解析一帧并将字段校验到位，任何异常都由 :meth:`poll` 统一处理。"""

        data = json.loads(payload.decode("utf-8"))
        if not isinstance(data, dict):
            raise TypeError("报文顶层必须是对象")
        position = tuple(float(value) for value in data["p"])
        velocity = tuple(float(value) for value in data["v"])
        if len(position) != 3 or len(velocity) != 3:
            raise ValueError("p/v 必须是三元组")
        timestamp = float(data["timestamp"])
        # 非有限值（NaN/Inf）会直接进控制律并让整架飞机发散，必须在入口拦下。
        if not all(math.isfinite(value) for value in position + velocity + (timestamp,)):
            raise ValueError("报文含非有限数值")
        return StampedTargetState(
            timestamp=timestamp,
            p=position,  # type: ignore[arg-type]
            v=velocity,  # type: ignore[arg-type]
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
