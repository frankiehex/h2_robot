"""驱动注册表：按配置里的 driver 字段实例化对应品牌驱动。"""
from __future__ import annotations

from .base import Pose, RobotArm, Waypoint


def create_arm(name: str, cfg: dict) -> RobotArm:
    driver = cfg.get("driver", "sim")
    if driver == "sim":
        from .sim_driver import SimDriver
        return SimDriver(name, cfg)
    if driver == "ur":
        from .ur_driver import URDriver
        return URDriver(name, cfg)
    if driver == "jaka":
        from .jaka_driver import JakaDriver
        return JakaDriver(name, cfg)
    if driver == "fairino":
        from .fairino_driver import FairinoDriver
        return FairinoDriver(name, cfg)
    raise ValueError(f"未知驱动: {driver}")


__all__ = ["Pose", "RobotArm", "Waypoint", "create_arm"]
