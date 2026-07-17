"""三品牌统一的机器人抽象接口。

上层（调度器）只依赖本模块；UR/节卡/法奥的差异封装在各自驱动里。
手臂端保持"傻"：只有 移动/取/放/回HOME 四类原语，
所有智能（视觉、互锁、异常策略）都在上位机。
"""
from __future__ import annotations

import abc
from dataclasses import dataclass, field


@dataclass
class Pose:
    """工具位姿，基座坐标系。单位 mm / deg。"""
    x: float
    y: float
    z: float
    rx: float = 180.0
    ry: float = 0.0
    rz: float = 0.0

    def offset(self, dx: float = 0.0, dy: float = 0.0, drz: float = 0.0) -> "Pose":
        """叠加视觉偏差（只用于取料点，放置点不加）。"""
        return Pose(self.x + dx, self.y + dy, self.z, self.rx, self.ry, self.rz + drz)

    @classmethod
    def from_list(cls, v: list[float]) -> "Pose":
        return cls(*v)


@dataclass
class Waypoint:
    """一个动作点：位姿 + 接近/退出 + 该段轨迹占用的 zone。"""
    pose: Pose
    approach: Pose
    retreat: Pose
    zones: list[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: dict) -> "Waypoint":
        return cls(
            pose=Pose.from_list(d["pose"]),
            approach=Pose.from_list(d["approach"]),
            retreat=Pose.from_list(d["retreat"]),
            zones=list(d.get("zones", [])),
        )


class RobotArm(abc.ABC):
    """统一手臂接口。实现类：SimDriver / URDriver / JakaDriver / FairinoDriver。

    所有运动调用为阻塞式（返回即到位）；节拍内的并行由调度器
    给每臂开线程实现，驱动内部不搞并发。
    """

    def __init__(self, name: str, cfg: dict):
        self.name = name
        self.cfg = cfg
        self.blend_mm: float = float(cfg.get("blend_mm", 10.0))
        self.speed_mm_s: float = float(cfg.get("speed_mm_s", 250.0))
        self._at_home = False

    # -- 连接管理 -----------------------------------------------------------
    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    # -- 运动原语 -----------------------------------------------------------
    @abc.abstractmethod
    def move_to(self, pose: Pose, blend_mm: float | None = None) -> None:
        """直线运动到位姿，blend>0 时与下一段圆滑过渡。"""

    @abc.abstractmethod
    def go_home(self) -> None:
        """回 HOME 安全岛。线体步进前调度器强制检查 at_home。"""

    @abc.abstractmethod
    def grip(self, close: bool) -> None:
        """夹爪/吸盘开合。真空吸盘: close=True 抽真空。"""

    # -- 状态 ----------------------------------------------------------------
    @property
    def at_home(self) -> bool:
        return self._at_home

    def read_pose(self):
        """读当前 TCP 位姿（首触/标定用）。真机驱动可覆盖；默认返回 None。"""
        return None

    def read_joints(self):
        """读当前六轴关节角（**度**），供看板 3D 用实际关节角还原姿态。
        真机驱动覆盖；默认返回 None（看板退回两连杆估计）。"""
        return None

    # -- 组合动作（品牌无关，直接用原语拼） -----------------------------------
    def pick(self, wp: Waypoint, dx: float = 0.0, dy: float = 0.0, drz: float = 0.0) -> None:
        """取料：approach → 下探(含视觉偏差) → 吸/夹 → retreat。"""
        self._at_home = False
        self.move_to(wp.approach.offset(dx, dy, drz), blend_mm=self.blend_mm)
        self.move_to(wp.pose.offset(dx, dy, drz), blend_mm=0.0)
        self.grip(True)
        self.move_to(wp.retreat.offset(dx, dy, drz), blend_mm=self.blend_mm)

    def place(self, wp: Waypoint) -> None:
        """放置：载具即基准，示教点直达，不加视觉偏差。"""
        self._at_home = False
        self.move_to(wp.approach, blend_mm=self.blend_mm)
        self.move_to(wp.pose, blend_mm=0.0)
        self.grip(False)
        self.move_to(wp.retreat, blend_mm=self.blend_mm)
