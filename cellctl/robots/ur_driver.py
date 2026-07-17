"""优傲 UR 驱动（RTDE，经 ur_rtde 库）。

依赖: pip install ur_rtde
文档: https://sdurobotics.gitlab.io/ur_rtde/
适配型号: UR e 系列（含 UR7e）。

单位约定: UR 用 **米 + 旋转矢量(弧度)**。本项目 Pose 是 mm + 度 RPY，
故 move_to 内做 mm→m、欧拉角→旋转矢量换算（见 geometry.py）。

配置字段（line.yaml 的 robot 段）:
  host, home(关节角 deg 6轴), speed_mm_s, blend_mm,
  accel_mm_s2(默认 500), grip_do(标准数字输出口，默认 0), grip_close_high(默认 true)
"""
from __future__ import annotations

import logging

from . import geometry as geo
from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class URDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self.accel = float(cfg.get("accel_mm_s2", 500.0)) / 1000.0  # m/s^2
        self.grip_do = int(cfg.get("grip_do", 0))
        self.grip_close_high = bool(cfg.get("grip_close_high", True))
        self._ctrl = None  # rtde_control.RTDEControlInterface
        self._recv = None  # rtde_receive.RTDEReceiveInterface

    def connect(self) -> None:
        import rtde_control
        import rtde_receive
        self._ctrl = rtde_control.RTDEControlInterface(self.host)
        self._recv = rtde_receive.RTDEReceiveInterface(self.host)
        self._at_home = False
        log.info("[%s] UR connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._ctrl:
            self._ctrl.stopScript()
            self._ctrl.disconnect()
        if self._recv:
            self._recv.disconnect()

    def _pose_to_ur(self, pose: Pose) -> list[float]:
        x, y, z = pose.x / 1000.0, pose.y / 1000.0, pose.z / 1000.0
        rx, ry, rz = geo.euler_deg_to_rotvec(pose.rx, pose.ry, pose.rz)
        return [x, y, z, rx, ry, rz]

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        p = self._pose_to_ur(pose)
        v = self.speed_mm_s / 1000.0
        b = (blend_mm if blend_mm is not None else self.blend_mm) / 1000.0
        if b > 0:
            # path 形式带转弯区：每个路点 = 位姿 + [speed, accel, blend]
            self._ctrl.moveL([p + [v, self.accel, b]])
        else:
            self._ctrl.moveL(p, v, self.accel)

    def go_home(self) -> None:
        import math
        home_deg = self.cfg["home"]
        q = [math.radians(a) for a in home_deg]
        self._ctrl.moveJ(q, 1.05, 1.4)
        self._at_home = True

    def grip(self, close: bool) -> None:
        level = close == self.grip_close_high
        self._ctrl.setStandardDigitalOut(self.grip_do, level)

    def read_pose(self) -> list[float]:
        """读当前 TCP 位姿（米 + 旋转矢量），首触/标定用。"""
        return self._recv.getActualTCPPose()
