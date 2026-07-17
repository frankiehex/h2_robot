"""节卡 JAKA 驱动（官方 jkrc Python SDK）。

依赖: 节卡官网下载 jkrc（Python 版）。
适配型号: Zu / A 系列（含 A5）。

单位约定: JAKA 笛卡尔用 **mm + 弧度 RPY**，故姿态三分量做 度→弧度（见 geometry.py）；
关节用 **弧度**。move_mode: 0=绝对(ABS)。

配置字段（line.yaml 的 robot 段）:
  host, home(关节角 deg 6轴), speed_mm_s, blend_mm,
  accel_mm_s2(默认 500), joint_speed_deg_s(默认 30),
  grip_iotype(默认 0=控制器DO), grip_do(索引,默认 0), grip_close_high(默认 true)
"""
from __future__ import annotations

import logging
import math

from . import geometry as geo
from .base import Pose, RobotArm

log = logging.getLogger(__name__)

_ABS = 0  # jkrc 绝对运动模式


class JakaDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self.accel = float(cfg.get("accel_mm_s2", 500.0))
        self.joint_speed = math.radians(float(cfg.get("joint_speed_deg_s", 30.0)))
        self.grip_iotype = int(cfg.get("grip_iotype", 0))
        self.grip_do = int(cfg.get("grip_do", 0))
        self.grip_close_high = bool(cfg.get("grip_close_high", True))
        self._rc = None  # jkrc.RC 实例

    def connect(self) -> None:
        import jkrc
        self._rc = jkrc.RC(self.host)
        self._rc.login()
        self._rc.power_on()
        self._rc.enable_robot()
        self._at_home = False
        log.info("[%s] JAKA connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._rc:
            self._rc.logout()

    def _pose_to_jaka(self, pose: Pose) -> list[float]:
        rx, ry, rz = geo.euler_deg_to_rad(pose.rx, pose.ry, pose.rz)
        return [pose.x, pose.y, pose.z, rx, ry, rz]  # mm + rad

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        end = self._pose_to_jaka(pose)
        b = blend_mm if blend_mm is not None else self.blend_mm
        if b > 0 and hasattr(self._rc, "linear_move_extend"):
            # 带转弯区（tol = 混合半径 mm）
            self._rc.linear_move_extend(end, _ABS, True, self.speed_mm_s, self.accel, b)
        else:
            self._rc.linear_move(end, _ABS, True, self.speed_mm_s)

    def go_home(self) -> None:
        q = [math.radians(a) for a in self.cfg["home"]]  # rad
        self._rc.joint_move(q, _ABS, True, self.joint_speed)
        self._at_home = True

    def grip(self, close: bool) -> None:
        value = 1 if (close == self.grip_close_high) else 0
        self._rc.set_digital_output(self.grip_iotype, self.grip_do, value)

    def read_pose(self):
        """读当前 TCP 位姿（mm + 弧度），首触/标定用。"""
        return self._rc.get_tcp_position()

    def read_joints(self):
        """六轴关节角（度）—— JAKA get_joint_position 返回弧度，转度。"""
        rc = self._rc.get_joint_position()
        data = rc[1] if isinstance(rc, (tuple, list)) and len(rc) == 2 else rc
        return [math.degrees(a) for a in data]
