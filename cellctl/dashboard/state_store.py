"""线程安全的状态快照持有者。

控制器在主线程更新快照；HTTP 服务在后台线程读取。
快照是普通 dict（可直接 json 序列化），面板轮询 /state.json 取用。
"""
from __future__ import annotations

import threading
from collections import deque


class StateStore:
    def __init__(self, grid_cfg: dict):
        self._lock = threading.Lock()
        self._grid = {"rows": grid_cfg.get("rows", 3), "cols": grid_cfg.get("cols", 4)}
        self._log: deque[dict] = deque(maxlen=30)
        self._snap: dict = {
            "state": "wait_line_locked",
            "cycle": 0,
            "cycle_time_s": 0.0,
            "throughput_pph": 0,
            "grid": self._grid,
            "stations": [],
            "zones": {},
            "active": None,
            "skips": [],
            "log": [],
        }

    def publish(self, **fields) -> None:
        """合并更新若干顶层字段。"""
        with self._lock:
            self._snap.update(fields)

    def log(self, msg: str, t: str = "", fault: bool = False) -> None:
        with self._lock:
            self._log.appendleft({"t": t, "msg": msg, "fault": fault})
            self._snap["log"] = list(self._log)

    def snapshot(self) -> dict:
        with self._lock:
            # 浅拷贝够用：字段整体替换，不做原地深改
            return dict(self._snap)
