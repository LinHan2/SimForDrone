"""双无人机观测阶段的 Pegasus 场景。

本阶段只建立 PX4 与 ROS 2 观测接口，不包含跟踪控制器、视觉位姿估计器
或 EKF 量测更新；它们必须在接口验收通过后再加入。
"""

from __future__ import annotations

import os
import time

import carb
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS

from simfordrone.config import load_dual_uav_config
from simfordrone.industrial_hangar import IndustrialHangar
from simfordrone.pegasus_compat import enable_rgbd_ros2_depth_marker


# PX4 的 HIL_ACTUATOR_CONTROLS 是归一化控制量。Pegasus 将其转换为转子角速度时，
# 原始默认增益 1000 只能产生约 8 N 总推力，低于 Iris 的离地需求。
# 该值可在启动时用 SIMFORDRONE_PX4_INPUT_SCALING 覆盖，便于单变量推力标定。
class DualUavObservationApp:
    """创建目标机与带前视相机的跟随机，二者使用独立 PX4 实例。"""

    def __init__(self, simulation_app):
        self.simulation_app = simulation_app
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.config = load_dual_uav_config()
        configured_scaling = float(self.config["px4"]["rotor_input_scaling"])
        # 环境变量只用于一次性标定复测，不修改可复现实验的 YAML 基线。
        self.rotor_input_scaling = float(os.environ.get("SIMFORDRONE_PX4_INPUT_SCALING", configured_scaling))
        if self.rotor_input_scaling <= 0.0:
            raise ValueError("PX4 rotor_input_scaling 必须为正数")
        # 使用官方 Warehouse USD，而非由基础几何体拼出的伪机库背景。
        IndustrialHangar(self.world.stage, self.world, self.config["environment"]).build()

        enable_rgbd_ros2_depth_marker()
        # 角色 → 载具句柄注册表：GUI 监视窗与终端状态输出都按角色读取世界位姿，
        # 避免在别处重复硬编码 "target"/"tracker" 字符串及其序号。
        self.vehicles: dict[str, dict] = {}
        self._create_vehicle("target", self.config["vehicles"]["target"], attach_camera=False)
        self._create_vehicle("tracker", self.config["vehicles"]["tracker"], attach_camera=True)

        self.world.reset()
        # WebRTC 全局观察相机，仅供人眼查看；它不等于跟随机机载传感器。
        self.pg.set_viewport_camera(self.config["viewport"]["position"], self.config["viewport"]["target"])
        # GUI 监视窗只在本地桌面模式创建：headless/WebRTC 下 omni.ui 会初始化失败，
        # 因此该分支延迟导入，保证无窗口服务器仍能正常启动场景。
        self.vehicle_monitor = None
        if os.environ.get("SIMFORDRONE_ISAAC_GUI", "0") == "1":
            from simfordrone.vehicle_monitor import VehicleMonitorWindow

            self.vehicle_monitor = VehicleMonitorWindow(self.vehicles)
        # headless 模式没有可交互面板，改用终端周期输出同样的两机状态，
        # 使 WebRTC 用户也能核对物体位置而不仅看到画面。
        self._next_console_status = time.monotonic()
        # 监视窗单独节流到 10 Hz：物理步进远高于此，逐帧更新 UI 模型只是浪费渲染时间。
        self._next_monitor_update = time.monotonic()
        self.stop_sim = False

    def _create_vehicle(
        self,
        role: str,
        vehicle: dict,
        *,
        attach_camera: bool,
    ) -> None:
        vehicle_id = int(vehicle["vehicle_id"])
        stage_path = str(vehicle["stage_path"])
        initial_position = list(vehicle["initial_position"])
        ros_namespace = str(vehicle["ros_namespace"])
        config = MultirotorConfig()
        mavlink = PX4MavlinkBackendConfig(
            {
                "vehicle_id": vehicle_id,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe,
                # 只调整 PX4 输出至 Pegasus 物理推力模型的映射，不改变飞控参数。
                "input_scaling": [self.rotor_input_scaling] * 4,
            }
        )
        ros2 = {
            "namespace": ros_namespace,
            "pub_state": True,
            "pub_sensors": False,
            "pub_graphical_sensors": attach_camera,
            # 暂不发布 TF：Isaac 内部 rclpy 与系统 tf2 Python 不能混用。
            # 后续由项目适配层统一发布机体/相机变换，保证坐标约定可审计。
            "pub_tf": False,
            "sub_control": False,
        }
        config.backends = [
            PX4MavlinkBackend(mavlink),
            ROS2Backend(vehicle_id=vehicle_id, config=ros2),
        ]

        if attach_camera:
            config.graphical_sensors = [
                MonocularCamera(self.config["observer_camera"]["name"], config=self._camera_config())
            ]

        multirotor = Multirotor(
            stage_path,
            ROBOTS["Iris"],
            vehicle_id,
            initial_position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )
        # 登记静态身份信息（stage 路径、序号、PX4 端点）。system_id 与 MAVLink 端口
        # 由 PX4 SITL 固定规则推出：system_id = instance+1、offboard 端口 = 14540+instance，
        # 与 px4ctrl/vehicle.py 的 VehicleRole 必须保持一致，否则会控制错机。
        self.vehicles[role] = {
            "vehicle": multirotor,
            "stage_path": stage_path,
            "vehicle_id": vehicle_id,
            "system_id": vehicle_id + 1,
            "mavlink_port": 14540 + vehicle_id,
        }

    def _camera_config(self) -> dict:
        """把 YAML 的人类可读字段转换为 Pegasus 传感器字段。"""

        camera = self.config["observer_camera"]
        return {
            "depth": bool(camera["depth"]),
            "position": list(camera["position"]),
            "orientation": list(camera["orientation_deg"]),
            "resolution": tuple(camera["resolution"]),
            "frequency": float(camera["frequency_hz"]),
            "intrinsics": camera["intrinsics"],
            "diagonal_fov": float(camera["diagonal_fov_deg"]),
        }

    def run(self) -> None:
        self.timeline.play()
        while self.simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
            # 监视窗读取的是 Pegasus 自己维护的 vehicle.state（Isaac 世界系 ENU），
            # 不引入第二套位姿来源，避免与飞控 EKF 估计混淆。
            now = time.monotonic()
            if self.vehicle_monitor is not None and now >= self._next_monitor_update:
                self.vehicle_monitor.update()
                self._next_monitor_update = now + 0.1
            # 终端状态按 1 Hz 节流输出：物理步进频率远高于此，逐帧打印会淹没日志。
            if now >= self._next_console_status:
                fields = []
                for role, entry in self.vehicles.items():
                    state = entry["vehicle"].state
                    fields.append(
                        f"{role}: p=({state.position[0]:+.2f},{state.position[1]:+.2f},"
                        f"{state.position[2]:+.2f}) v=({state.linear_velocity[0]:+.2f},"
                        f"{state.linear_velocity[1]:+.2f},{state.linear_velocity[2]:+.2f})"
                    )
                print("[vehicle-state] " + " | ".join(fields), flush=True)
                self._next_console_status = now + 1.0

        carb.log_warn("DualUavObservationApp 正在关闭。")
        self.timeline.stop()
        self.simulation_app.close()
