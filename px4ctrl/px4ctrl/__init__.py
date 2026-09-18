"""px4ctrl：SimForDrone 的统一飞控控制环。

本包把 PX4 命令执行集中在一处，避免控制代码分散到多个脚本：

- :mod:`px4ctrl.params`    —— 参数与校验（含仿真/真机差异点 ``link``）
- :mod:`px4ctrl.vehicle`   —— 角色 → 端点映射（哪台无人机、哪个端口、哪个 system id）
- :mod:`px4ctrl.frames`    —— ENU/FLU ↔ NED/FRD 转换（含可执行的解析自检）
- :mod:`px4ctrl.inputs`    —— 输入状态与新鲜度判定
- :mod:`px4ctrl.controller`—— 线性几何控制器（推力 + 姿态）
- :mod:`px4ctrl.link`      —— MAVLink 连接层，唯一接触飞控协议的模块
- :mod:`px4ctrl.fsm`       —— 控制状态机（手动/悬停/指令跟踪/自动起降）
- :mod:`px4ctrl.cli`       —— 唯一命令入口

仿真与真机共用同一套控制器与状态机；差异只在参数文件的 ``link`` 段。
"""

from px4ctrl.controller import ControllerOutput, DesiredState, LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.link import LinkError, MavlinkLink
from px4ctrl.params import ParamError, Params, load_params
from px4ctrl.vehicle import VehicleRole, available_roles, resolve_role

__all__ = [
    "ControllerOutput",
    "DesiredState",
    "LinearControl",
    "LinkError",
    "MavlinkLink",
    "PX4CtrlFSM",
    "ParamError",
    "Params",
    "State",
    "VehicleRole",
    "available_roles",
    "load_params",
    "resolve_role",
]
