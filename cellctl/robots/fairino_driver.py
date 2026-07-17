"""法奥 Fairino 驱动（官方 fairino Python SDK）。

依赖: 官网下载 fairino Python SDK（import fairino）。
适配型号: FR 系列（含 FR5 / “F5”）。

单位约定: 法奥用 **mm + 度 RPY**，与本项目 Pose 一致，直接透传（最省事）。
注意: 法奥 MoveL 的 vel 是**速度百分比(0-100)**，不是 mm/s；
      故用配置 vel_pct，而非 speed_mm_s。blendR 单位 mm，= blend_mm。

配置字段（line.yaml 的 robot 段）:
  host, home(关节角 deg 6轴), vel_pct(默认 30), blend_mm,
  tool(工具坐标号,默认 0), user(用户坐标号,默认 0),
  grip_do(数字输出口,默认 0), grip_close_high(默认 true)
"""
from __future__ import annotations

import logging

from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class FairinoDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self.vel_pct = float(cfg.get("vel_pct", 30.0))
        self.tool = int(cfg.get("tool", 0))
        self.user = int(cfg.get("user", 0))
        self.grip_do = int(cfg.get("grip_do", 0))
        self.grip_close_high = bool(cfg.get("grip_close_high", True))
        self._robot = None  # fairino.Robot.RPC 实例

    def connect(self) -> None:
        from fairino import Robot
        self._robot = Robot.RPC(self.host)
        self._at_home = False
        log.info("[%s] Fairino connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._robot:
            self._robot.CloseRPC()

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        b = blend_mm if blend_mm is not None else self.blend_mm
        desc = [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]  # mm + deg，透传
        rc = self._robot.MoveL(desc, self.tool, self.user,
                               vel=self.vel_pct, blendR=(b if b > 0 else -1.0))
        self._check(rc, "MoveL")

    def go_home(self) -> None:
        joints = list(self.cfg["home"])  # deg，透传
        rc = self._robot.MoveJ(joints, self.tool, self.user, vel=self.vel_pct)
        self._check(rc, "MoveJ")
        self._at_home = True

    def grip(self, close: bool) -> None:
        level = 1 if (close == self.grip_close_high) else 0
        rc = self._robot.SetDO(self.grip_do, level)
        self._check(rc, "SetDO")

    @staticmethod
    def _check(rc, op: str) -> None:
        # 法奥 SDK 多数指令返回错误码（0 = 成功）；有的返回 (码, 数据)
        code = rc[0] if isinstance(rc, (tuple, list)) else rc
        if code not in (0, None):
            raise RuntimeError(f"法奥 {op} 失败，错误码 {code}")
