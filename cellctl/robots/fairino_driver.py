"""法奥 Fairino 驱动骨架（官方 fairino Python SDK）。

依赖: pip install fairino（或官网 SDK 包）。
接真机时补齐 TODO；接口语义以 base.RobotArm 为准。
"""
from __future__ import annotations

import logging

from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class FairinoDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self._robot = None  # fairino.Robot.RPC 实例

    def connect(self) -> None:
        from fairino import Robot

        self._robot = Robot.RPC(self.host)
        log.info("[%s] Fairino connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._robot:
            self._robot.CloseRPC()

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        # 法奥 MoveL: 位姿 mm + deg（与本项目 Pose 同单位，最省事）。
        # blendR 参数即转弯区半径。
        raise NotImplementedError(
            "接真机时实现: self._robot.MoveL(desc_pos=[x,y,z,rx,ry,rz], tool, user, "
            "vel, blendR=blend_mm)"
        )

    def go_home(self) -> None:
        # MoveJ 到 cfg['home']
        raise NotImplementedError
        self._at_home = True

    def grip(self, close: bool) -> None:
        # self._robot.SetDO(...)
        raise NotImplementedError
