"""双无人机观测阶段的 Pegasus 场景。

本阶段只建立 PX4 与 ROS 2 观测接口，不包含跟踪控制器、视觉位姿估计器
或 EKF 量测更新；它们必须在接口验收通过后再加入。
"""

from __future__ import annotations

import os

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
        self._create_vehicle(self.config["vehicles"]["target"], attach_camera=False)
        self._create_vehicle(self.config["vehicles"]["tracker"], attach_camera=True)

        self.world.reset()
        # WebRTC 全局观察相机，仅供人眼查看；它不等于跟随机机载传感器。
        self.pg.set_viewport_camera(self.config["viewport"]["position"], self.config["viewport"]["target"])
        self.stop_sim = False

    def _create_vehicle(
        self,
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

        Multirotor(
            stage_path,
            ROBOTS["Iris"],
            vehicle_id,
            initial_position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

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

        carb.log_warn("DualUavObservationApp 正在关闭。")
        self.timeline.stop()
        self.simulation_app.close()
