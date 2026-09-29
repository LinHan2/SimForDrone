"""机载相机视角叠加层：显示目标位置、偏航控制状态。

在 Isaac Sim UI 中创建一个模拟相机画面的窗口，实时显示：
- 目标在图像中的像素坐标（从相对位置投影）
- 图像中心十字参考线
- Barrier Lyapunov 偏航控制状态（像素偏差、归一化误差、控制量）
- 视野边界警告（当目标接近图像边缘时）
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import omni.ui as ui


class CameraOverlayWindow:
    """机载相机视角可视化窗口。"""

    def __init__(
        self,
        image_width: int = 1280,
        image_height: int = 720,
        fov_deg: float = 100.0,
    ) -> None:
        self.image_width = image_width
        self.image_height = image_height
        self.fov_deg = fov_deg
        self.center_x = image_width / 2
        self.center_y = image_height / 2
        
        # 水平 FOV（假设对角线 FOV）
        aspect = image_width / image_height
        self.fov_h_rad = 2 * math.atan(math.tan(math.radians(fov_deg / 2)) * aspect / math.sqrt(1 + aspect**2))
        
        # 状态变量
        self.target_pixel_x: float | None = None
        self.target_pixel_y: float | None = None
        self.pixel_offset_x: float = 0.0
        self.z1_normalized: float = 0.0
        self.yaw_rate_cmd: float = 0.0
        self.in_deadzone: bool = False
        self.target_visible: bool = False
        
        # UI 窗口
        self.window = ui.Window("Camera View (IBVS)", width=640, height=400, visible=True)
        self.window.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Tracker Onboard Camera View", height=24)
                
                # 相机画面模拟区域（简化显示）
                with ui.ZStack(height=280):
                    # 背景
                    ui.Rectangle(style={"background_color": 0xFF1A1A1A})
                    
                    # 这里可以用 Canvas 绘制十字线和目标标记
                    self._canvas = ui.Canvas(width=640, height=280)
                
                # 控制状态信息
                ui.Label("Barrier Lyapunov Yaw Control Status:", height=20)
                self._status_model = ui.SimpleStringModel("Waiting for target...")
                ui.StringField(model=self._status_model, read_only=True, multiline=True, height=80)

    def update_from_relative_state(
        self,
        relative_position: tuple[float, float, float],
        tracker_yaw: float,
    ) -> None:
        """从相对位置计算目标在图像中的投影。
        
        Args:
            relative_position: tracker 到 target 的相对位置向量 (ENU)
            tracker_yaw: tracker 偏航角 (rad, ENU)
        """
        # 转换到 tracker body frame (FLU)
        dx, dy, dz = relative_position
        cos_yaw = math.cos(-tracker_yaw)  # 负号：从世界系到机体系
        sin_yaw = math.sin(-tracker_yaw)
        
        # Body FLU: x=forward, y=left, z=up
        x_body = dx * cos_yaw - dy * sin_yaw
        y_body = dx * sin_yaw + dy * cos_yaw
        z_body = dz
        
        # 相机坐标系（相机朝前，光轴 = body +X）
        # 简化：假设相机与机体对齐
        x_cam = x_body  # 深度（前向）
        y_cam = y_body  # 横向（左为正）
        z_cam = z_body  # 纵向（上为正）
        
        # 目标是否在相机前方
        if x_cam <= 0.1:  # 太近或在后方
            self.target_visible = False
            self.target_pixel_x = None
            self.target_pixel_y = None
            return
        
        # 针孔投影
        # 水平 FOV -> 焦距
        f_pixel = self.center_x / math.tan(self.fov_h_rad / 2)
        
        # 像素坐标（图像中心为原点，右正下正）
        u = -y_cam * f_pixel / x_cam + self.center_x  # 注意：y_cam 左正，像素右正
        v = -z_cam * f_pixel / x_cam + self.center_y  # z_cam 上正，像素下正
        
        # 判断是否在视野内
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            self.target_visible = True
            self.target_pixel_x = u
            self.target_pixel_y = v
            self.pixel_offset_x = u - self.center_x
        else:
            self.target_visible = False
            self.target_pixel_x = None
            self.target_pixel_y = None

    def update_control_status(
        self,
        z1_normalized: float,
        yaw_rate_cmd: float,
        in_deadzone: bool,
    ) -> None:
        """更新 Barrier Lyapunov 控制状态。"""
        self.z1_normalized = z1_normalized
        self.yaw_rate_cmd = yaw_rate_cmd
        self.in_deadzone = in_deadzone

    def render(self) -> None:
        """刷新 UI 显示。"""
        # 清空画布
        self._canvas.clear()
        
        with self._canvas:
            # 绘制图像边界
            ui.Line(
                ui.Point(10, 10),
                ui.Point(630, 10),
                ui.Point(630, 270),
                ui.Point(10, 270),
                ui.Point(10, 10),
                style={"color": 0xFF404040, "border_width": 2},
            )
            
            # 绘制中心十字线
            cx = 320  # 画布中心
            cy = 140
            ui.Line(
                ui.Point(cx - 20, cy),
                ui.Point(cx + 20, cy),
                style={"color": 0xFF00FF00, "border_width": 1},
            )
            ui.Line(
                ui.Point(cx, cy - 20),
                ui.Point(cx, cy + 20),
                style={"color": 0xFF00FF00, "border_width": 1},
            )
            
            # 绘制目标标记
            if self.target_visible and self.target_pixel_x is not None:
                # 将图像坐标映射到画布坐标
                # 图像: 1280x720 -> 画布: 620x260 (留 10px 边距)
                scale_x = 620 / self.image_width
                scale_y = 260 / self.image_height
                
                canvas_x = self.target_pixel_x * scale_x + 10
                canvas_y = self.target_pixel_y * scale_y + 10
                
                # 目标圆圈
                color = 0xFFFF0000 if abs(self.z1_normalized) > 0.6 else 0xFF00FFFF
                ui.Circle(
                    ui.Point(canvas_x, canvas_y),
                    radius=8,
                    style={"color": color, "border_width": 2},
                )
                
                # 从中心到目标的连线
                ui.Line(
                    ui.Point(cx, cy),
                    ui.Point(canvas_x, canvas_y),
                    style={"color": 0xFF888888, "border_width": 1},
                )
        
        # 更新状态文本
        if not self.target_visible:
            status_text = "⚠️  Target NOT in camera view!\n"
        elif self.in_deadzone:
            status_text = "✓  Target centered (deadzone)\n"
        else:
            status_text = f"⚡ Active control\n"
        
        status_text += (
            f"Pixel offset X: {self.pixel_offset_x:+.1f} px\n"
            f"Normalized z1: {self.z1_normalized:+.4f} (barrier @ ±0.80)\n"
            f"Yaw rate cmd: {math.degrees(self.yaw_rate_cmd):+.2f} deg/s\n"
        )
        
        if abs(self.z1_normalized) > 0.7:
            status_text += "⚠️  WARNING: Approaching FOV boundary!"
        
        self._status_model.set_value(status_text)
