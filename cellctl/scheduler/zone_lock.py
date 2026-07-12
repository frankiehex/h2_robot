"""区域互锁（Zone Lock）：多臂共享空间的互斥管理。

原则（见 docs/cell-control-design.md §3）：
  - 能靠布局分开的不进这里；这里只登记确实共享的空间
  - 一段轨迹要进哪些 zone 由 Waypoint.zones 声明，先拿全再动
  - 按 zone 名排序申请 → 天然无死锁（资源有序分配）
  - 申请/释放全部写日志，出问题可回放
"""
from __future__ import annotations

import logging
import threading
from contextlib import contextmanager

log = logging.getLogger(__name__)


class ZoneLockManager:
    def __init__(self, zone_names: list[str], acquire_timeout_s: float = 30.0):
        self._locks: dict[str, threading.Lock] = {z: threading.Lock() for z in zone_names}
        self._owner: dict[str, str] = {}
        self.acquire_timeout_s = acquire_timeout_s

    @contextmanager
    def hold(self, owner: str, zones: list[str]):
        """拿到 zones 里全部锁才返回；with 退出自动全部释放。

        排序申请是防死锁的关键：两臂同时要 {A,B} 时都先抢 A，
        不会出现"你拿A等B、我拿B等A"。
        """
        ordered = sorted(set(zones))
        acquired: list[str] = []
        try:
            for z in ordered:
                lock = self._locks.get(z)
                if lock is None:
                    raise KeyError(f"未定义的 zone: {z}（检查 line.yaml 的 zones）")
                holder = self._owner.get(z)
                if holder and holder != owner:
                    log.info("[lock] %s 等待 %s（当前持有: %s）", owner, z, holder)
                if not lock.acquire(timeout=self.acquire_timeout_s):
                    raise TimeoutError(
                        f"{owner} 等 {z} 超时 {self.acquire_timeout_s}s"
                        f"（持有者: {self._owner.get(z)}）——检查是否有臂卡死或 zone 粒度过粗"
                    )
                self._owner[z] = owner
                acquired.append(z)
                log.info("[lock] %s 获得 %s", owner, z)
            yield
        finally:
            for z in reversed(acquired):
                self._owner.pop(z, None)
                self._locks[z].release()
                log.info("[lock] %s 释放 %s", owner, z)
