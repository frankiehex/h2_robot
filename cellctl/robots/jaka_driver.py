"""节卡 JAKA 驱动骨架（官方 Python SDK，TCP 10001 端口）。

依赖: 节卡官网下载 jkrc SDK（Python 版）。
接真机时补齐 TODO；接口语义以 base.RobotArm 为准。
"""
from __future__ import annotations

import logging

from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class JakaDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self._rc = None  # jkrc.RC 实例

    def connect(self) -> None:
        import jkrc

        self._rc = jkrc.RC(self.host)
        self._rc.login()
        self._rc.power_on()
        self._rc.enable_robot()
        log.info("[%s] JAKA connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._rc:
            self._rc.logout()

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        # JAKA linear_move: 位姿单位 mm + rad。TODO: deg→rad；
        # blend 对应 SDK 的运动混合参数（不同固件版本字段名有差异，需实测）。
        raise NotImplementedError(
            "接真机时实现: self._rc.linear_move([x,y,z,rx,ry,rz], ABS, True, speed)"
        )

    def go_home(self) -> None:
        # joint_move 到 cfg['home']
        raise NotImplementedError
        self._at_home = True

    def grip(self, close: bool) -> None:
        # self._rc.set_digital_output(...)
        raise NotImplementedError
