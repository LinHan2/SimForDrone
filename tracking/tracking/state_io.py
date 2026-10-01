"""独立测试进程之间传递目标状态的本地 UDP 通道。

为何不用单一进程直连两台 PX4：UDP 端口（14540/14541）同一时刻只能被一个
客户端有效读取，两个进程同时连接会互相抢包。因此把"控制 target"与"控制
tracker"拆成两个进程，进程间只用本机回环传状态，不传任何飞控命令。

该模块是 V0 的临时适配层；后续换成 ROS 2 话题时只需替换本文件，
`guidance` 与 `px4ctrl` 均无需修改。
"""

from __future__ import annotations

import errno
import json
import math
import socket
import time
from dataclasses import asdict, dataclass
from typing import Protocol

from tracking.guidance import TargetState, Vector3

DEFAULT_STATE_HOST = "127.0.0.1"
DEFAULT_STATE_PORT = 14600
Quaternion = tuple[float, float, float, float]
_IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class StampedTargetState:
    """带上发布时刻的目标状态，用于接收端判定新鲜度。"""

    # 注意时间基准：timestamp 取发布进程的 time.monotonic()，两进程同机运行，
    # 所以该时刻与订阅端可直接相减；跨机部署时需换为统一时间源。
    timestamp: float
    # 发布端严格递增的序号。时间戳用于跨进程新鲜度判断；序号用于日志审计缓存保持。
    sequence: int
    p: Vector3
    v: Vector3
    # 解析轨迹给出的加速度参考；非解析任务显式为 None，不能由遥测速差分伪造。
    reference_a: Vector3 | None = None
    # 目标机 PX4 EKF 的 ENU/FLU 姿态。相对 EKF 用它约束目标加速度方向；旧发送端
    # 缺少该字段时按单位四元数处理，以保持状态通道的向后兼容。
    q: Quaternion = _IDENTITY_QUATERNION
    attitude_timestamp: float | None = None

    def as_target_state(self) -> TargetState:
        # reference_a 缺失时保持零加速度：非解析任务不得用遥测速度差分伪造前馈。
        return TargetState(p=self.p, v=self.v, a=self.reference_a or (0.0, 0.0, 0.0))


@dataclass(frozen=True)
class TargetTerminalEvent:
    """target 任务的显式终态；不能由状态超时推断。"""

    timestamp: float
    result: str


class TargetStatePublisher:
    def __init__(self, host: str = DEFAULT_STATE_HOST, port: int = DEFAULT_STATE_PORT) -> None:
        self._destination = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = 0

    def publish(
        self,
        state: TargetState,
        timestamp: float | None = None,
        attitude: Quaternion = _IDENTITY_QUATERNION,
        reference_acceleration: Vector3 | None = None,
        attitude_timestamp: float | None = None,
    ) -> None:
        self._sequence += 1
        message = StampedTargetState(
            timestamp=time.monotonic() if timestamp is None else timestamp,
            sequence=self._sequence,
            p=state.p,
            v=state.v,
            reference_a=reference_acceleration,
            q=attitude,
            attitude_timestamp=attitude_timestamp,
        )
        self._socket.sendto(json.dumps(asdict(message), separators=(",", ":")).encode(), self._destination)

    def publish_terminal(self, result: str, timestamp: float | None = None) -> None:
        """发布任务终态；重复发送降低无连接 UDP 丢失单包的影响。"""

        if result not in {"completed", "failed", "interrupted"}:
            raise ValueError(f"不支持的 target 终态: {result}")
        message = TargetTerminalEvent(
            timestamp=time.monotonic() if timestamp is None else timestamp,
            result=result,
        )
        payload = json.dumps(asdict(message), separators=(",", ":")).encode()
        for _ in range(3):
            self._socket.sendto(payload, self._destination)

    def close(self) -> None:
        self._socket.close()


class TargetStatePublisherProtocol(Protocol):
    """target 状态发布器的最小接口，供单播和镜像发布共用。"""

    def publish(
        self,
        state: TargetState,
        timestamp: float | None = None,
        attitude: Quaternion = _IDENTITY_QUATERNION,
        reference_acceleration: Vector3 | None = None,
        attitude_timestamp: float | None = None,
    ) -> None: ...

    def publish_terminal(self, result: str, timestamp: float | None = None) -> None: ...

    def close(self) -> None: ...


class MirroredTargetStatePublisher:
    """将同一 target 状态帧镜像发布到独立的控制与影子验证端口。"""

    def __init__(self, *publishers: TargetStatePublisher) -> None:
        if not publishers:
            raise ValueError("至少需要一个 target 状态发布器")
        self._publishers = publishers

    def publish(
        self,
        state: TargetState,
        timestamp: float | None = None,
        attitude: Quaternion = _IDENTITY_QUATERNION,
        reference_acceleration: Vector3 | None = None,
        attitude_timestamp: float | None = None,
    ) -> None:
        for publisher in self._publishers:
            publisher.publish(state, timestamp, attitude, reference_acceleration, attitude_timestamp)

    def publish_terminal(self, result: str, timestamp: float | None = None) -> None:
        for publisher in self._publishers:
            publisher.publish_terminal(result, timestamp)

    def close(self) -> None:
        for publisher in self._publishers:
            publisher.close()


class TargetStateSubscriber:
    def __init__(self, host: str = DEFAULT_STATE_HOST, port: int = DEFAULT_STATE_PORT) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((host, port))
        except OSError as error:
            # 订阅端必须独占端口：Linux 上 UDP 单播不允许两个 socket 绑定同一端口；
            # SO_REUSEPORT 虽可放开限制，但内核会按哈希把数据报拆分给各 socket，
            # 每个订阅方都会静默丢包，对控制闭环不可接受，因此刻意不用。
            # 串行工作流下 EADDRINUSE 几乎总是上一次的订阅进程（run_tracker 或
            # 另一个观测器）没有退出，属操作问题而非网络故障，报错要给出排查命令。
            if error.errno == errno.EADDRINUSE:
                self._socket.close()
                raise RuntimeError(
                    f"UDP 状态端口 {host}:{port} 已被占用：存在另一个订阅进程未退出"
                    f"（如 run_tracker 或上一次的观测器）。请先执行"
                    f" `ss -ulpn | grep {port}` 找到占用者并停止它，再重新启动本进程。"
                ) from error
            raise
        self._socket.setblocking(False)
        self.latest: StampedTargetState | None = None
        self.terminal: TargetTerminalEvent | None = None
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
                message = self._decode(payload)
            except (ValueError, TypeError, KeyError, UnicodeDecodeError) as error:
                self.dropped += 1
                self.last_drop_reason = f"{type(error).__name__}: {error}"
                continue
            if isinstance(message, TargetTerminalEvent):
                self.terminal = message
            else:
                # 只保留最后一帧：跟踪控制用的是当前目标状态，积压的旧帧没有价值。
                self.latest = message

    @staticmethod
    def _decode(payload: bytes) -> StampedTargetState | TargetTerminalEvent:
        """解析一帧并将字段校验到位，任何异常都由 :meth:`poll` 统一处理。"""

        data = json.loads(payload.decode("utf-8"))
        if not isinstance(data, dict):
            raise TypeError("报文顶层必须是对象")
        if "result" in data:
            timestamp = float(data["timestamp"])
            result = data["result"]
            if result not in {"completed", "failed", "interrupted"}:
                raise ValueError("未知 target 终态")
            if not math.isfinite(timestamp):
                raise ValueError("报文含非有限数值")
            return TargetTerminalEvent(timestamp=timestamp, result=result)
        position = tuple(float(value) for value in data["p"])
        velocity = tuple(float(value) for value in data["v"])
        reference_acceleration = data.get("reference_a")
        if reference_acceleration is not None:
            reference_acceleration = tuple(float(value) for value in reference_acceleration)
        attitude = tuple(float(value) for value in data.get("q", _IDENTITY_QUATERNION))
        if (
            len(position) != 3
            or len(velocity) != 3
            or len(attitude) != 4
            or (reference_acceleration is not None and len(reference_acceleration) != 3)
        ):
            raise ValueError("p/v/reference_a 必须是三元组，q 必须是四元组")
        timestamp = float(data["timestamp"])
        attitude_timestamp = data.get("attitude_timestamp")
        if attitude_timestamp is not None:
            attitude_timestamp = float(attitude_timestamp)
        sequence = int(data.get("sequence", 0))
        # 非有限值（NaN/Inf）会直接进控制律并让整架飞机发散，必须在入口拦下。
        values = position + velocity + attitude + (timestamp,)
        if attitude_timestamp is not None:
            values += (attitude_timestamp,)
        if reference_acceleration is not None:
            values += reference_acceleration
        if sequence < 0 or not all(math.isfinite(value) for value in values):
            raise ValueError("报文含非有限数值")
        if sum(value * value for value in attitude) < 1e-12:
            raise ValueError("姿态四元数不得为零")
        return StampedTargetState(
            timestamp=timestamp,
            sequence=sequence,
            p=position,  # type: ignore[arg-type]
            v=velocity,  # type: ignore[arg-type]
            reference_a=reference_acceleration,  # type: ignore[arg-type]
            q=attitude,  # type: ignore[arg-type]
            attitude_timestamp=attitude_timestamp,
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
