"""区域互锁核心性质验证。直接运行: python3 tests/test_zone_lock.py"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from cellctl.scheduler.dispatcher import scan_cells
from cellctl.scheduler.zone_lock import ZoneLockManager


def test_shared_zone_mutual_exclusion():
    """两臂抢同一 zone：任何时刻至多一个在区内。"""
    mgr = ZoneLockManager(["SHARED"], acquire_timeout_s=5.0)
    inside = []
    max_inside = []

    def worker(name: str):
        for _ in range(20):
            with mgr.hold(name, ["SHARED"]):
                inside.append(name)
                max_inside.append(len(inside))
                time.sleep(0.001)
                inside.remove(name)

    threads = [threading.Thread(target=worker, args=(f"arm{i}",)) for i in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert max(max_inside) == 1, f"共享区同时出现 {max(max_inside)} 个臂"


def test_multi_zone_no_deadlock():
    """两臂以相反顺序声明 {A,B}：排序申请应保证不死锁。"""
    mgr = ZoneLockManager(["A", "B"], acquire_timeout_s=5.0)
    done = []

    def worker(name: str, zones: list[str]):
        for _ in range(50):
            with mgr.hold(name, zones):
                time.sleep(0.0005)
        done.append(name)

    t1 = threading.Thread(target=worker, args=("arm1", ["A", "B"]))
    t2 = threading.Thread(target=worker, args=("arm2", ["B", "A"]))
    t1.start(); t2.start()
    t1.join(timeout=10); t2.join(timeout=10)
    assert done == ["arm1", "arm2"] or done == ["arm2", "arm1"], f"疑似死锁: 完成={done}"


def test_unknown_zone_raises():
    mgr = ZoneLockManager(["A"])
    try:
        with mgr.hold("arm1", ["NOT_EXIST"]):
            pass
    except KeyError:
        return
    raise AssertionError("未定义 zone 应报 KeyError")


def test_scan_order():
    assert scan_cells(3, 4, "serpentine", active_row=1) == ["R1C1", "R1C2", "R1C3", "R1C4"]
    assert scan_cells(3, 4, "serpentine", active_row=2) == ["R2C4", "R2C3", "R2C2", "R2C1"]
    assert scan_cells(3, 3, "by_column", active_row=1) == ["R1C1", "R1C2", "R1C3"]


if __name__ == "__main__":
    for fn in [test_shared_zone_mutual_exclusion, test_multi_zone_no_deadlock,
               test_unknown_zone_raises, test_scan_order]:
        fn()
        print(f"PASS {fn.__name__}")
    print("全部通过")
