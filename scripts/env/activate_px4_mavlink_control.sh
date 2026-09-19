#!/usr/bin/env bash
# 仅供 P2.0 控制脚本 source：使用 PX4 工程已创建的 Python 3.12 venv 中的 pymavlink。
# 此环境不加载 Isaac Sim、Conda 或 ROS Python 库；MAVLink 控制和 ROS 验收保持两个进程。

if [[ -z "${SIMFORDRONE_ROOT:-}" ]]; then
    echo "ERROR: source 前必须设置 SIMFORDRONE_ROOT" >&2
    return 1
fi

SIMFORDRONE_PX4_PYTHON="${SIMFORDRONE_ROOT}/PX4-Autopilot/.venv/bin/python"
if [[ ! -x "${SIMFORDRONE_PX4_PYTHON}" ]]; then
    echo "ERROR: PX4 Python venv 或 pymavlink 不存在: ${SIMFORDRONE_PX4_PYTHON}" >&2
    return 1
fi

# 避免 Conda 或用户 PYTHONPATH 向控制进程注入非 PX4 venv 的包。
unset PYTHONPATH PYTHONHOME VIRTUAL_ENV LD_LIBRARY_PATH LD_PRELOAD
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_EXE CONDA_PYTHON_EXE
export PYTHONNOUSERSITE=1
export SIMFORDRONE_PX4_PYTHON

