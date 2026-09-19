"""视口视角控制与操作命令解析。

本模块分两层，目的是让"命令解析"可以在没有 Isaac 运行时的环境里被单测：

- :func:`parse_view_command`：纯函数，把终端输入映射为视角模式名；
- :class:`ViewControl`：把模式作用到真实 viewport，Isaac 相关导入全部放在方法内部。

支持的视角：

===============  ====================================================
``overview``     场景固定全局取景（与 YAML 的 viewport 段一致）
``follow_target``  相机跟随目标机，从机体后上方观察
``follow_tracker`` 相机跟随跟随机，从机体后上方观察
``onboard_tracker`` 直接切到跟随机机载前视相机（真实第一视角）
===============  ====================================================
"""

from __future__ import annotations

import select
import sys

# 视角模式名。用字符串而非枚举，便于直接打印到终端与 HUD 上。
#: ``free`` 表示**不碰相机**：viewport 由鼠标操作，跟随模式只在显式选择时才生效。
#: 这是默认模式——跟随模式每帧都会写相机位姿，会立刻覆盖掉手动拖动，
#: 因此“默认自动跟随”会让人以为相机被锁死。
FREE = "free"
OVERVIEW = "overview"
FOLLOW_TARGET = "follow_target"
FOLLOW_TRACKER = "follow_tracker"
ONBOARD_TRACKER = "onboard_tracker"
ALL_MODES = (FREE, OVERVIEW, FOLLOW_TARGET, FOLLOW_TRACKER, ONBOARD_TRACKER)

#: 跟随相机的世界系 ENU 偏移（米）。取西南方向、抬高 3 m：
#: 与场景默认取景方向一致，能同时看到机体、航向和两机分离过程。
FOLLOW_OFFSET = (-5.0, -5.0, 3.0)

#: 场景固定全局取景，与 configs/dual_uav_hangar.yaml 的 viewport 段保持一致。
OVERVIEW_EYE = (10.0, -10.0, 7.0)
OVERVIEW_TARGET = (0.0, 0.0, 1.5)

#: Isaac Sim 默认透视相机；从机载相机切回普通视角时必须显式指定它。
DEFAULT_CAMERA_PATH = "/OmniverseKit_Persp"

# 终端命令 → 模式。允许数字和单词两种输入，避免记忆负担。
#: ``0``/``free`` 回到手动控制（不做任何自动写入）。
_COMMANDS: dict[str, str] = {
    "0": FREE,
    "f": FREE,
    "free": FREE,
    "manual": FREE,
    "1": OVERVIEW,
    "o": OVERVIEW,
    "overview": OVERVIEW,
    "2": FOLLOW_TARGET,
    "t": FOLLOW_TARGET,
    "target": FOLLOW_TARGET,
    "3": FOLLOW_TRACKER,
    "r": FOLLOW_TRACKER,
    "tracker": FOLLOW_TRACKER,
    "4": ONBOARD_TRACKER,
    "cam": ONBOARD_TRACKER,
    "camera": ONBOARD_TRACKER,
    "onboard": ONBOARD_TRACKER,
}

#: 终端里打印的命令提示，保持与 :data:`_COMMANDS` 同步。
HELP_LINES = (
    "视角命令（在场景终端输入）:",
    "  0 / free      手动控制相机（默认，鼠标自由拖动）",
    "  1 / overview  固定全局取景",
    "  2 / target    跟随目标机 (target)",
    "  3 / tracker   跟随跟随机 (tracker)",
    "  4 / onboard   跟随机机载相机第一视角",
    "  v             打印当前视角与两机位置",
)


def parse_view_command(text: str) -> str | None:
    """把一行终端输入解析为视角模式名；无法识别时返回 ``None``。

    纯字符串处理，不做任何 Isaac 调用，因此可被单元测试直接覆盖。
    """

    return _COMMANDS.get(text.strip().lower())


class ViewControl:
    """把视角模式应用到活动 viewport，并解析操作员终端命令。"""

    def __init__(
        self,
        vehicles: dict[str, dict],
        *,
        initial_mode: str = FREE,
        onboard_camera_path: str | None = None,
    ) -> None:
        # 保存载具注册表引用：每帧读取 vehicle.state 计算跟随相机位姿。
        self._vehicles = vehicles
        self._mode = initial_mode
        self._onboard_camera_path = onboard_camera_path
        # 记录已应用的模式，避免每帧重复执行"切换相机"这类一次性命令。
        self._applied_mode: str | None = None
        self._warned_missing_camera = False

    @property
    def mode(self) -> str:
        return self._mode

    def set_onboard_camera_path(self, camera_prim_path: str | None) -> None:
        """补充机载相机路径。

        Pegasus 在传感器 start() 阶段才真正创建相机 prim，而场景是先在
        __init__ 里构建 UI，因此一开始可能拿不到路径；找到后由此接口回填，
        否则操作员会一直得到"未找到机载相机"。
        """

        self._onboard_camera_path = camera_prim_path

    def set_mode(self, mode: str) -> bool:
        """切换视角模式；模式非法时返回 ``False`` 且不改变现状。"""

        if mode not in ALL_MODES:
            return False
        self._mode = mode
        return True

    def describe(self) -> str:
        """返回一行人类可读的当前视角说明。"""

        camera = self._onboard_camera_path or "(未找到机载相机)"
        if self._mode == ONBOARD_TRACKER:
            return f"{self._mode} @ {camera}"
        if self._mode == FREE:
            return f"{self._mode}（相机由鼠标控制，程序不写入）"
        return self._mode

    # ------------------------------------------------------------------ 应用

    def apply(self) -> None:
        """每个仿真步调用一次，把当前模式作用到 viewport。

        ``FREE`` 模式下**不写入任何相机状态**：这是让鼠标 / 键盘能自由调整视角的前提。
        跟随模式则必须每帧写入，否则相机不会跟着载具运动。
        """

        if self._mode == FREE:
            # 从机载相机返回手动时，需要先把 viewport 挂回默认透视相机，
            # 否则相机会一直“贴”在载具上，看起来仍然被锁死。
            if self._applied_mode == ONBOARD_TRACKER:
                self._restore_default_camera()
            self._applied_mode = self._mode
            return

        if self._mode == ONBOARD_TRACKER:
            # 机载相机是载具的子 prim，会随载具自动运动，
            # 因此这里只需切换一次，绝不能再用 set_camera_view 去改它的位姿
            # （那会把载具自己的相机挪走，破坏视觉观测）。
            self._apply_onboard_once()
            return

        if self._applied_mode == ONBOARD_TRACKER:
            # 从机载视角返回时，必须先把 viewport 挂回默认透视相机，
            # 否则 set_camera_view 移动的是载具相机而不是观察视角。
            self._restore_default_camera()

        if self._mode == OVERVIEW:
            if self._applied_mode != OVERVIEW:
                self._set_camera_view(OVERVIEW_EYE, OVERVIEW_TARGET)
        else:
            role = "target" if self._mode == FOLLOW_TARGET else "tracker"
            self._apply_follow(role)

        self._applied_mode = self._mode

    def _apply_follow(self, role: str) -> None:
        """让观察相机跟随指定载具：相机位置 = 载具位置 + 固定世界系偏移。"""

        entry = self._vehicles.get(role)
        if entry is None:
            return
        position = entry["vehicle"].state.position
        # 逐分量显式构造，保证类型是长度 3 的元组（相机 API 要求三维向量）。
        eye = (
            float(position[0]) + FOLLOW_OFFSET[0],
            float(position[1]) + FOLLOW_OFFSET[1],
            float(position[2]) + FOLLOW_OFFSET[2],
        )
        target: tuple[float, float, float] = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
        # 每帧都设置：相机要跟着载具连续运动，不能只设一次。
        self._set_camera_view(eye, target)

    def _apply_onboard_once(self) -> None:
        if self._applied_mode == ONBOARD_TRACKER:
            return
        if not self._onboard_camera_path:
            if not self._warned_missing_camera:
                print("[view] 未找到跟随机机载相机，退回手动视角", flush=True)
                self._warned_missing_camera = True
            # 退回手动而不是跟随：免得用户以为相机坏了，却无法自己调整。
            self._mode = FREE
            return
        try:
            from isaacsim.core.utils.viewports import set_active_viewport_camera

            set_active_viewport_camera(self._onboard_camera_path)
            print(f"[view] 已切换到跟随机机载相机 {self._onboard_camera_path}", flush=True)
        except Exception as error:  # 视角切换失败不应中断仿真
            print(f"[view] 切换机载相机失败：{error}，退回手动视角", flush=True)
            self._mode = FREE
            return
        self._applied_mode = self._mode

    def _restore_default_camera(self) -> None:
        try:
            from isaacsim.core.utils.viewports import set_active_viewport_camera

            set_active_viewport_camera(DEFAULT_CAMERA_PATH)
        except Exception as error:
            print(f"[view] 恢复默认相机失败：{error}", flush=True)

    @staticmethod
    def _set_camera_view(eye: tuple[float, float, float], target: tuple[float, float, float]) -> None:
        try:
            from isaacsim.core.utils.viewports import set_camera_view

            set_camera_view(eye, target)
        except Exception as error:
            print(f"[view] 设置视角失败：{error}", flush=True)

    # ------------------------------------------------------------------ 输入

    def poll_stdin(self) -> list[str]:
        """非阻塞读取操作员命令，返回需要打印的提示行。

        使用 ``select`` 零超时轮询而不是 ``input()``：后者会阻塞仿真主循环，
        在流式模式下表现为画面卡死。
        """

        messages: list[str] = []
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return messages
        line = sys.stdin.readline()
        if not line:
            return messages
        text = line.strip().lower()
        if text in ("v", "status", "s"):
            messages.append(self.status_line())
            return messages
        mode = parse_view_command(text)
        if mode is None:
            messages.append(f"[view] 未识别命令 {text!r}；可用：" + " / ".join(ALL_MODES))
            return messages
        self.set_mode(mode)
        messages.append(f"[view] 视角 → {self.describe()}")
        return messages

    def status_line(self) -> str:
        """当前视角与两机世界位置的单行摘要，便于直接在终端核对。"""

        parts = [f"view={self.mode}"]
        for role, entry in self._vehicles.items():
            position = entry["vehicle"].state.position
            parts.append(
                f"{role}=({position[0]:+.2f},{position[1]:+.2f},{position[2]:+.2f})"
            )
        return " ".join(parts)
