#!/usr/bin/env bash
# 仅供 start_dual_px4_scene.sh 在子进程中 source。
# 目的：让 Isaac Sim 5.1 的嵌入式 Python 3.11 只加载自身携带的 Jazzy 库，
# 不接触 Ubuntu 24.04 系统 ROS Jazzy 的 Python 3.12 二进制扩展。

if [[ -z "${ISAACSIM_ROOT:-}" ]]; then
    echo "ERROR: source 前必须设置 ISAACSIM_ROOT" >&2
    return 1
fi

ISAAC_ROS_LIB="${ISAACSIM_ROOT}/exts/isaacsim.ros2.bridge/jazzy/lib"
if [[ ! -d "${ISAAC_ROS_LIB}" ]]; then
    echo "ERROR: Isaac Sim 内置 Jazzy 库不存在: ${ISAAC_ROS_LIB}" >&2
    return 1
fi

# 清除会把 Conda、系统 ROS 或其它 Python 运行库带进 Isaac 进程的变量。
unset PYTHONPATH PYTHONHOME VIRTUAL_ENV LD_LIBRARY_PATH LD_PRELOAD
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_EXE CONDA_PYTHON_EXE CUDA_HOME DISPLAY XAUTHORITY WAYLAND_DISPLAY

# 固定可执行文件搜索路径，避免 Conda 的 python、ldconfig 工具等抢占运行时。
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ISAACSIM_PATH="${ISAACSIM_ROOT}"
export PYTHONNOUSERSITE=1

# 这里只设置 Isaac 内部 Jazzy 的动态库目录，绝不拼接 /opt/ros/jazzy/lib。
# ROS_DISTRO/RMW 仅选择 Isaac 内置 bridge 的 Jazzy + Fast DDS 实现。
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${SIMFORDRONE_ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${ISAAC_ROS_LIB}"

