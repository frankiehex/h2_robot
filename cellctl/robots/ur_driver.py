"""优傲 UR 驱动骨架（RTDE，经 ur_rtde 库）。

依赖: pip install ur_rtde
文档: https://sdurobotics.gitlab.io/ur_rtde/
接真机时补齐 TODO；接口语义以 base.RobotArm 为准。
"""
from __future__ import annotations

import logging

from .base import Pose, RobotArm

log = logging.getLogger(__name__)


class URDriver(RobotArm):
    def __init__(self, name: str, cfg: dict):
        super().__init__(name, cfg)
        self.host: str = cfg["host"]
        self._ctrl = None  # rtde_control.RTDEControlInterface
        self._recv = None  # rtde_receive.RTDEReceiveInterface

    def connect(self) -> None:
        import rtde_control
        import rtde_receive

        self._ctrl = rtde_control.RTDEControlInterface(self.host)
        self._recv = rtde_receive.RTDEReceiveInterface(self.host)
        log.info("[%s] UR connected @ %s", self.name, self.host)

    def disconnect(self) -> None:
        if self._ctrl:
            self._ctrl.disconnect()

    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        # UR 位姿单位: m + 旋转矢量(rad)。TODO: deg 欧拉角 → 旋转矢量转换。
        blend_m = (blend_mm if blend_mm is not None else self.blend_mm) / 1000.0
        raise NotImplementedError(
            "接真机时实现: self._ctrl.moveL([x,y,z,rx,ry,rz], speed, accel, "
            f"blend={blend_m}) — 注意单位换算与欧拉角→旋转矢量"
        )

    def go_home(self) -> None:
        # moveJ 到 cfg['home'] 关节角（deg → rad）
        raise NotImplementedError
        self._at_home = True

    def grip(self, close: bool) -> None:
        # 真空发生器/夹爪走 UR 控制柜数字 IO: self._ctrl.setStandardDigitalOut(...)
        raise NotImplementedError
