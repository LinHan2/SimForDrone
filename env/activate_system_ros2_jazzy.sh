#!/usr/bin/env bash
# 仅供独立的验收/算法终端 source。
# 此进程使用 Ubuntu 系统 ROS Jazzy（Python 3.12），绝不能被 Isaac Sim 进程 source。

# 先清理可能来自 Conda、先前工作空间或 Isaac Sim 的 ROS 变量，保证 ros2 CLI
# 观察的是系统 Jazzy 域，而不是残留的自定义 overlay。
unset PYTHONPATH PYTHONHOME VIRTUAL_ENV LD_LIBRARY_PATH LD_PRELOAD
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI

# Jazzy 的生成脚本会直接读取未设置的 AMENT_TRACE_SETUP_FILES。兼容启用 nounset
# 的终端：source 时临时关闭，随后恢复原来的 shell 选项。
_simfordrone_had_nounset=0
case $- in
    *u*) _simfordrone_had_nounset=1; set +u ;;
esac
source /opt/ros/jazzy/setup.bash
if (( _simfordrone_had_nounset )); then
    set -u
fi
unset _simfordrone_had_nounset

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${SIMFORDRONE_ROS_DOMAIN_ID:-0}"

