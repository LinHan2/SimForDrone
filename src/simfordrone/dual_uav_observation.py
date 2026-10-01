"""双无人机观测阶段的 Pegasus 场景。

本阶段只建立 PX4 与 ROS 2 观测接口，不包含跟踪控制器、视觉位姿估计器
或 EKF 量测更新；它们必须在接口验收通过后再加入。
"""

from __future__ import annotations

import os
import math
import time
import json
from pathlib import Path

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
from simfordrone.lighting import LightRig
from simfordrone.pegasus_compat import enable_rgbd_ros2_depth_marker
from simfordrone.view_control import FREE, ONBOARD_TRACKER


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
        # 环境与光照分开构建：前者决定“场景里有什么”，后者决定“怎么照亮它们”。
        # 空网格场景既没有物体、光照也不足，无法支撑后续图像闭环。
        environment = IndustrialHangar(
            self.world.stage, self.world, self.config["environment"]
        )
        environment_label = environment.build()
        light_paths = LightRig(self.world.stage, self.config["environment"]["lighting"]).build()
        # 把“环境真的载入了多少物体、装了几盏灯”打进启动日志：
        # 这是判断场景是否为空的最直接证据，无需依赖肉眼看视频流。
        print(
            f"[scene] 环境 {environment_label}: 载入 {environment.loaded_prim_count} 个 prim；"
            f"光源 {len(light_paths)} 个 -> {', '.join(light_paths)}",
            flush=True,
        )

        enable_rgbd_ros2_depth_marker()
        # 角色 → 载具句柄注册表：GUI 监视窗与终端状态输出都按角色读取世界位姿，
        # 避免在别处重复硬编码 "target"/"tracker" 字符串及其序号。
        self.vehicles: dict[str, dict] = {}
        self._create_vehicle("target", self.config["vehicles"]["target"], attach_camera=False)
        self._create_vehicle("tracker", self.config["vehicles"]["tracker"], attach_camera=True)

        self.world.reset()
        # WebRTC 全局观察相机，仅供人眼查看；它不等于跟随机机载传感器。
        self.pg.set_viewport_camera(self.config["viewport"]["position"], self.config["viewport"]["target"])
        # UI 是否绘制：本地桌面（--gui）与“带 UI 的 WebRTC 流”都需要它；
        # 纯画面流不能绘制 omni.ui，因此这里按开关决定是否导入，
        # 避免在无窗口且无 UI 合成的环境下初始化 UI 扩展而报错。
        self.ui_enabled = os.environ.get("SIMFORDRONE_ISAAC_UI", "0") == "1"
        # UI 已启用时默认不再往终端刷状态（信息已在画面上）；若观看端无法播放
        # 视频流，可用 SIMFORDRONE_ISAAC_CONSOLE_STATE=1 强制保留终端输出。
        self.console_state = not self.ui_enabled or (
            os.environ.get("SIMFORDRONE_ISAAC_CONSOLE_STATE", "0") == "1"
        )
        self.vehicle_monitor = None
        self.view_control = None
        self.camera_overlay = None
        if self.ui_enabled:
            # 相机扫描只为机载视角切换服务，非 UI 模式下不做：
            # 否则会打印"target 未找到机载相机"，而 target 本来就没挂相机，产生误导。
            self._register_camera_paths()
            try:
                from simfordrone.vehicle_monitor import VehicleHudWindow
                from simfordrone.view_control import FREE, HELP_LINES, ONBOARD_TRACKER, ViewControl

                # 机载视角只对 tracker 存在（target 未挂相机），缺失时 ViewControl 会退回跟随视角。
                tracker_camera = self.vehicles["tracker"].get("camera_prim_path")
                self._onboard_requested = os.environ.get("SIMFORDRONE_INITIAL_VIEW") == ONBOARD_TRACKER
                self.view_control = ViewControl(self.vehicles, onboard_camera_path=tracker_camera)
                if self._onboard_requested and tracker_camera:
                    self.view_control.set_mode(ONBOARD_TRACKER)
                self.vehicle_monitor = VehicleHudWindow(self.vehicles, self.view_control)
                
                if not self._onboard_requested:
                    try:
                        from simfordrone.camera_overlay import CameraOverlayWindow

                        cam_cfg = self.config["observer_camera"]
                        self.camera_overlay = CameraOverlayWindow(
                            image_width=cam_cfg["resolution"][0],
                            image_height=cam_cfg["resolution"][1],
                            fov_deg=cam_cfg["diagonal_fov_deg"],
                        )
                    except Exception as error:
                        print(f"[ui] 模拟相机窗口不可用，保留双机 HUD：{error}", flush=True)
                print("[ui] HUD 已叠加到视频流；视角控制可用：", flush=True)
                for line in HELP_LINES:
                    print("  " + line, flush=True)
            except Exception as error:
                # UI 出错必须降级而不是让场景崩溃：曾因 HUD 里的 AttributeError
                # 逃逸出 __init__，导致 Isaac 在退出阶段段错误、整个场景不可用。
                # 这里关掉 UI 开关，主循环会自动回到终端 1 Hz 状态输出。
                print(f"[ui] HUD/视角控制创建失败，降级为终端输出：{error}", flush=True)
                self.ui_enabled = False
                self.view_control = None
                self.vehicle_monitor = None
                self.camera_overlay = None
        # headless 模式没有可交互面板，改用终端周期输出同样的两机状态，
        # 使 WebRTC 用户也能核对物体位置而不仅看到画面。
        self._next_console_status = time.monotonic()
        # 监视窗单独节流到 10 Hz：物理步进远高于此，逐帧更新 UI 模型只是浪费渲染时间。
        self._next_monitor_update = time.monotonic()
        # 操作员命令轮询节流（同 10 Hz）。
        self._next_command_poll = time.monotonic()
        # 相机 prim 可能晚于场景构建才出现，因此保留一个低频重扫节流点。
        self._next_camera_scan = time.monotonic()
        self.stop_sim = False
        self.restart_dir = Path(__file__).resolve().parents[2] / "logs" / "px4_restart" / str(os.getpid())
        self.restart_dir.mkdir(parents=True, exist_ok=True)
        print(f"[px4-restart] 请求目录：{self.restart_dir}", flush=True)

    def _handle_px4_restart(self) -> None:
        request_file = self.restart_dir / "request.json"
        if not request_file.exists():
            return
        response_file = self.restart_dir / "response.json"
        request = None
        try:
            request = json.loads(request_file.read_text(encoding="utf-8"))
            nonce = request["nonce"]
            if not isinstance(nonce, str) or not nonce:
                raise ValueError("无效请求标识")
            backends = []
            for role in ("target", "tracker"):
                vehicle = self.vehicles[role]["vehicle"]
                state = vehicle.state
                home_z = self.config["vehicles"][role]["initial_position"][2]
                if abs(float(state.position[2]) - float(home_z)) > 0.25 or math.sqrt(
                    sum(float(value) ** 2 for value in state.linear_velocity)
                ) > 0.2:
                    raise RuntimeError(f"{role} 尚未静止在初始地面位置，拒绝重启")
                backends.append(next(backend for backend in vehicle._backends if isinstance(backend, PX4MavlinkBackend)))
            processes = [backend.px4_tool.px4_process for backend in backends]
            for backend in backends:
                backend.stop()
            for process in processes:
                if process is not None:
                    process.wait(timeout=5)
            for backend in backends:
                backend.start()
            result = {"nonce": nonce, "ok": True, "message": "双 PX4 backend 已重新启动；等待健康检查恢复"}
        except Exception as error:
            result = {"nonce": request.get("nonce") if isinstance(request, dict) else None,
                      "ok": False, "message": str(error)}
        response_file.write_text(json.dumps(result, ensure_ascii=False), encoding="utf-8")
        request_file.unlink(missing_ok=True)
        print(f"[px4-restart] {result['message']}", flush=True)

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
            # PoseStamped 提供 ENU 位姿/姿态，state/accel 提供世界 ENU 加速度；IMU
            # 保留 Pegasus 的原始 NED/FRD 比力，供 ROS 感知进程完成独立的时间对齐与转换。
            "pub_pose": True,
            "pub_accel": True,
            "pub_sensors": True,
            "pub_imu": True,
            # Pegasus 的 ROS2Backend 会向后端分发每一种仿真传感器数据；当前上游实现
            # 对单项发布器关闭没有保护，因此传感器总开关打开时必须同时创建 IMU、磁力计
            # 和 GPS 发布器，避免每帧访问缺失属性而中断图形相机更新。
            "pub_mag": True,
            "pub_gps": True,
            "pub_gps_vel": True,
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
        self._color_vehicle_body(stage_path, role)
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

    def _color_vehicle_body(self, stage_path: str, role: str) -> None:
        from pxr import Gf, Sdf, UsdShade

        color = {"target": (1.0, 0.35, 0.04), "tracker": (0.02, 0.8, 0.72)}[role]
        body = self.world.stage.GetPrimAtPath(f"{stage_path}/body")
        if not body.IsValid():
            raise RuntimeError(f"Iris 机身节点不存在: {stage_path}/body")
        material = UsdShade.Material.Define(self.world.stage, f"{stage_path}/Looks/RolePaint")
        shader = UsdShade.Shader.Define(self.world.stage, f"{stage_path}/Looks/RolePaint/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(body).Bind(
            material, UsdShade.Tokens.strongerThanDescendants
        )

    def _register_camera_paths(self) -> None:
        """在 stage 中查找各载具挂载的相机 prim，供机载视角切换使用。

        Pegasus 用 ``get_stage_next_free_path`` 生成相机路径，同名时可能追加后缀，
        因此不能写死 ``<vehicle>/body/<camera>``，必须实际遍历确认。
        """

        from pxr import UsdGeom

        stage = self.world.stage
        for role, entry in self.vehicles.items():
            prefix = str(entry["stage_path"]) + "/"
            entry["camera_prim_path"] = None
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith(prefix):
                    continue
                if prim.IsA(UsdGeom.Camera):
                    entry["camera_prim_path"] = path
                    break
            if entry["camera_prim_path"] is None:
                print(f"[ui] {role} 未找到机载相机 prim", flush=True)

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
            # 视角必须在每个仿真步重新应用：跟随模式要求相机连续跟随载具运动。
            now = time.monotonic()
            self._handle_px4_restart()
            if self.view_control is not None:
                self.view_control.apply()
                # 命令轮询单独节流到 10 Hz：stdin 读取无需跟随帧率。
                if now >= self._next_command_poll:
                    for message in self.view_control.poll_stdin():
                        print(message, flush=True)
                    self._next_command_poll = now + 0.1
                # 机载相机 prim 可能晚于场景构建才由传感器 start() 创建，
                # 因此只要 tracker 还没拿到路径就低频重扫一次。
                if self.vehicles["tracker"].get("camera_prim_path") is None and now >= self._next_camera_scan:
                    self._register_camera_paths()
                    self.view_control.set_onboard_camera_path(
                        self.vehicles["tracker"].get("camera_prim_path")
                    )
                    if (self._onboard_requested and self.view_control.mode == FREE
                            and self.vehicles["tracker"].get("camera_prim_path")):
                        self.view_control.set_mode(ONBOARD_TRACKER)
                        self._onboard_requested = False
                    self._next_camera_scan = now + 2.0
            # HUD 读取的是 Pegasus 自己维护的 vehicle.state（Isaac 世界系 ENU），
            # 不引入第二套位姿来源，避免与飞控 EKF 估计混淆。
            if self.vehicle_monitor is not None and now >= self._next_monitor_update:
                self.vehicle_monitor.update()
                self._next_monitor_update = now + 0.1
            
            # 更新相机叠加层：显示目标在图像中的位置
            if self.camera_overlay is not None:
                target_state = self.vehicles["target"]["vehicle"].state
                tracker_state = self.vehicles["tracker"]["vehicle"].state
                
                # 计算相对位置（世界系 ENU）
                rel_pos = (
                    float(target_state.position[0] - tracker_state.position[0]),
                    float(target_state.position[1] - tracker_state.position[1]),
                    float(target_state.position[2] - tracker_state.position[2]),
                )
                
                # 获取 tracker 偏航角
                # Pegasus state.attitude 是四元数 [w, x, y, z]
                q = tracker_state.attitude
                tracker_yaw = math.atan2(
                    2.0 * (float(q[0]) * float(q[3]) + float(q[1]) * float(q[2])),
                    1.0 - 2.0 * (float(q[2])**2 + float(q[3])**2)
                )
                
                # 更新相机视角投影
                self.camera_overlay.update_from_relative_state(rel_pos, tracker_yaw)
                
                # TODO: 从控制器获取实际的 Barrier Lyapunov 控制状态
                # 目前先用占位值，后续需要从 ROS2 话题订阅或控制器直接回传
                self.camera_overlay.update_control_status(
                    z1_normalized=0.0,  # 占位
                    yaw_rate_cmd=0.0,   # 占位
                    in_deadzone=True,   # 占位
                )
                self.camera_overlay.render()
            
            # 终端状态按 1 Hz 节流输出：物理步进频率远高于此，逐帧打印会淹没日志。
            if self.console_state and now >= self._next_console_status:
                # 除两机位置外还给出速度模长与两机间距：跟踪实验首先关心的就是
                # 相对几何是否维持，间距比六个坐标分量更直接。
                states = {role: entry["vehicle"].state for role, entry in self.vehicles.items()}
                fields = []
                for role, state in states.items():
                    position = state.position
                    velocity = state.linear_velocity
                    speed = math.sqrt(sum(float(component) ** 2 for component in velocity))
                    fields.append(
                        f"{role}: p=({position[0]:+.2f},{position[1]:+.2f},{position[2]:+.2f}) "
                        f"|v|={speed:.2f}"
                    )
                if len(states) == 2:
                    first, second = states.values()
                    fields.append(f"gap={math.dist(first.position, second.position):.2f}m")
                print("[vehicle-state] " + " | ".join(fields), flush=True)
                self._next_console_status = now + 1.0

        carb.log_warn("DualUavObservationApp 正在关闭。")
        self.timeline.stop()
        self.simulation_app.close()
