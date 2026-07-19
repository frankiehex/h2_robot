"""仿真驱动：无硬件跑通全流程（状态机、互锁、回 HOME 检查）。

用真实感的时间模型（距离/速度）模拟运动耗时，
这样在仿真里就能看出节拍预算够不够、锁等待有多长。
"""
from __future__ import annotations

import logging
import math
import time

from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class SimDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self._pos = Pose(0.0, 0.0, 300.0)
        self._gripping = False
        # 仿真时间倍率：1.0=真实节奏；调大跑得快，只验证逻辑
        self.time_scale: float = float(cfg.get("sim_time_scale", 20.0))

    def connect(self) -> None:
        log.info("[%s] sim connected", self.name)
        self._at_home = True

    def disconnect(self) -> None:
        log.info("[%s] sim disconnected", self.name)

    def _travel(self, target: Pose) -> None:
        dist = math.dist(
            (self._pos.x, self._pos.y, self._pos.z), (target.x, target.y, target.z)
        )
        t = dist / self.speed_mm_s / self.time_scale
        time.sleep(t)
        self._pos = target

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        self._travel(pose)
        log.debug("[%s] at (%.0f, %.0f, %.0f)", self.name, pose.x, pose.y, pose.z)

    def go_home(self) -> None:
        self._travel(Pose(0.0, 0.0, 300.0))
        self._at_home = True
        log.info("[%s] HOME", self.name)

    def grip(self, close: bool) -> None:
        time.sleep(0.3 / self.time_scale)
        self._gripping = close
