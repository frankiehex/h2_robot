"""任务生成与派发：网格 → 本节拍任务表 → 各臂并行执行。

一个任务 = 一次"取料(含视觉偏差) + 放到第(r,c)格"。
扫描顺序（蛇形/按列）在配置里定，减少手臂大幅重新定向。
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

from ..robots import RobotArm, Waypoint
from ..vision.locator import VisionResult
from .zone_lock import ZoneLockManager

log = logging.getLogger(__name__)


@dataclass
class PickPlaceTask:
    cell: str                    # "R1C2"
    pick: Waypoint
    place: Waypoint
    vision: VisionResult         # 偏差只加在取料上


@dataclass
class StationRuntime:
    """一个工位的运行时：手臂 + 位姿表 + 定位器。"""
    name: str
    arm: RobotArm
    pick_wp: Waypoint
    grid_wps: dict[str, Waypoint]
    locator: object              # ContourLocator / FakeLocator
    min_score: float = 0.5
    last_report: list[str] = field(default_factory=list)


def scan_cells(rows: int, cols: int, order: str, active_row: int = 1) -> list[str]:
    """生成本节拍要作业的格位序列。基础节拍只做面前一行（active_row）。"""
    cells = []
    rng = range(1, cols + 1)
    if order == "serpentine" and active_row % 2 == 0:
        rng = reversed(list(rng))
    for c in rng:
        cells.append(f"R{active_row}C{c}")
    return cells


def build_tasks(st: StationRuntime, cells: list[str], frame=None) -> list[PickPlaceTask]:
    """每格取一次料：每次取料前重新定位（料源每次都变）。"""
    tasks = []
    for cell in cells:
        wp = st.grid_wps.get(cell)
        if wp is None:
            log.warning("[%s] 格 %s 未示教，跳过", st.name, cell)
            continue
        vr = st.locator.locate(frame)
        if not vr.found:
            log.warning("[%s] 格 %s 来料未找到/异常(score=%.2f)，跳过并记录",
                        st.name, cell, vr.score)
            continue
        tasks.append(PickPlaceTask(cell=cell, pick=st.pick_wp, place=wp, vision=vr))
    return tasks


def run_station(st: StationRuntime, tasks: list[PickPlaceTask],
                locks: ZoneLockManager) -> None:
    """单工位串行执行任务表；zone 锁按轨迹段申请（取料段/放置段分开）。"""
    st.last_report = []
    for t in tasks:
        vr = t.vision
        with locks.hold(st.name, t.pick.zones):
            st.arm.pick(t.pick, dx=vr.x_mm, dy=vr.y_mm, drz=vr.theta_deg)
        with locks.hold(st.name, t.place.zones):
            st.arm.place(t.place)
        st.last_report.append(t.cell)
        log.info("[%s] 完成 %s (视觉偏差 dx=%.1f dy=%.1f dθ=%.1f)",
                 st.name, t.cell, vr.x_mm, vr.y_mm, vr.theta_deg)
    st.arm.go_home()


def run_all_stations(stations: list[StationRuntime],
                     tasks_by_station: dict[str, list[PickPlaceTask]],
                     locks: ZoneLockManager) -> None:
    """各工位并行（每臂一线程），全部完成才返回——这就是节拍的 EXECUTING 段。"""
    errors: dict[str, Exception] = {}

    def _worker(st: StationRuntime) -> None:
        try:
            run_station(st, tasks_by_station.get(st.name, []), locks)
        except Exception as e:  # noqa: BLE001 - 集中上报给状态机转 FAULT
            errors[st.name] = e
            log.exception("[%s] 执行异常", st.name)

    threads = [threading.Thread(target=_worker, args=(st,), name=st.name)
               for st in stations]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    if errors:
        raise RuntimeError(f"工位执行异常: {list(errors)}")
