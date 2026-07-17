"""主状态机：每个步进节拍一圈。

WAIT_LINE_LOCKED → CAPTURE → DISPATCH → EXECUTING → ADVANCE → (回到开头)
任一环节异常 → FAULT（本骨架里 = 停机并抛出，接真机后接人工复位/自动重试策略）。

线体 IO 抽象成 LineIO：仿真用 SimLineIO；
接真机换成 PLC 实现（Modbus TCP / OPC UA 读到位信号、发步进指令）。
"""
from __future__ import annotations

import enum
import logging
import time

from .dispatcher import StationRuntime, build_tasks, run_all_stations, scan_cells
from .zone_lock import ZoneLockManager

log = logging.getLogger(__name__)


class State(enum.Enum):
    WAIT_LINE_LOCKED = "wait_line_locked"
    CAPTURE = "capture"
    DISPATCH = "dispatch"
    EXECUTING = "executing"
    ADVANCE = "advance"
    FAULT = "fault"


class LineIO:
    """线体接口。接真机时实现 PLC 版本。"""

    def wait_locked(self, timeout_s: float) -> None:
        """阻塞直到线体停稳+到位锁定。"""
        raise NotImplementedError

    def advance(self, timeout_s: float) -> None:
        """发步进指令并等完成。"""
        raise NotImplementedError


class SimLineIO(LineIO):
    def __init__(self, step_time_s: float = 0.1):
        self.step_time_s = step_time_s

    def wait_locked(self, timeout_s: float) -> None:
        time.sleep(self.step_time_s)
        log.info("[line] 到位锁定")

    def advance(self, timeout_s: float) -> None:
        time.sleep(self.step_time_s)
        log.info("[line] 步进完成")


class CellController:
    def __init__(self, stations: list[StationRuntime], locks: ZoneLockManager,
                 line: LineIO, grid_cfg: dict, line_cfg: dict, store=None):
        self.stations = stations
        self.locks = locks
        self.line = line
        self.grid_cfg = grid_cfg
        self.line_cfg = line_cfg
        self.state = State.WAIT_LINE_LOCKED
        self.cycle_count = 0
        self.store = store  # 可选 StateStore：接看板时发布快照

    def _set(self, s: State) -> None:
        log.info("== 状态: %s → %s", self.state.value, s.value)
        self.state = s
        self._publish()

    def _publish(self, cycle_time: float | None = None) -> None:
        """把当前状态发布给看板（若挂了 StateStore）。"""
        if self.store is None:
            return
        stations = []
        for st in self.stations:
            done = list(getattr(st, "last_report", []) or [])
            stations.append({
                "name": st.name,
                "label": getattr(st, "label", st.name),
                "brand": getattr(st, "brand", ""),
                "status": "home" if st.arm.at_home else "working",
                "target": None,
                "vision": {"dx": 0.0, "dy": 0.0, "dth": 0.0},
                "done": done,
                "at_home": st.arm.at_home,
            })
        # 全量 zone 表（未持有 = None），供看板显示 FREE/持有者
        all_zones = getattr(self.locks, "_locks", {})
        owners = getattr(self.locks, "_owner", {})
        zones = {z: owners.get(z) for z in all_zones}
        fields = {
            "state": self.state.value,
            "cycle": self.cycle_count,
            "stations": stations,
            "zones": zones,
        }
        if cycle_time is not None:
            cols = self.grid_cfg.get("cols", 4)
            fields["cycle_time_s"] = round(cycle_time, 1)
            fields["throughput_pph"] = round(cols * 3600 / max(cycle_time, 1e-6))
        self.store.publish(**fields)

    def run_cycle(self) -> None:
        """跑一个完整节拍。"""
        t0 = time.monotonic()

        self._set(State.WAIT_LINE_LOCKED)
        self.line.wait_locked(self.line_cfg.get("lock_signal_timeout_s", 10.0))

        self._set(State.CAPTURE)
        # 骨架里定位在 build_tasks 内逐格做；接真机后此处触发全部相机
        # 预拍一帧存档（QC/溯源用），定位仍在取料前逐次进行。

        self._set(State.DISPATCH)
        cells = scan_cells(self.grid_cfg["rows"], self.grid_cfg["cols"],
                           self.grid_cfg.get("scan_order", "serpentine"))
        tasks = {st.name: build_tasks(st, cells) for st in self.stations}
        total = sum(len(v) for v in tasks.values())
        log.info("本节拍任务: %d 个 (格位: %s)", total, cells)

        self._set(State.EXECUTING)
        try:
            run_all_stations(self.stations, tasks, self.locks)
        except Exception:
            self._set(State.FAULT)
            raise
        self._publish()  # 执行完刷新各工位 done / HOME 状态

        # 步进前强制检查：所有臂必须在 HOME 安全岛
        not_home = [st.name for st in self.stations if not st.arm.at_home]
        if not_home:
            self._set(State.FAULT)
            raise RuntimeError(f"步进前有臂不在 HOME: {not_home}——禁止步进")

        self._set(State.ADVANCE)
        self.line.advance(self.line_cfg.get("step_timeout_s", 5.0))

        self.cycle_count += 1
        cycle_time = time.monotonic() - t0
        log.info("== 节拍 #%d 完成，用时 %.2fs（仿真时间已加速）",
                 self.cycle_count, cycle_time)
        self._publish(cycle_time=cycle_time)
