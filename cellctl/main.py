"""入口：加载配置 → 组装工位 → 仿真跑 N 个节拍。

无硬件验证:  python -m cellctl.main --cycles 3
带看板:      python -m cellctl.main --dashboard --cycles 50
             然后浏览器打开 http://127.0.0.1:8700 看实时状态。
重点看: 状态流转、互锁 [lock] 日志、步进前的回 HOME 检查。
"""
from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import yaml

from .dashboard import DashboardServer, StateStore
from .robots import Waypoint, create_arm
from .scheduler import CellController, SimLineIO, StationRuntime, ZoneLockManager
from .vision.locator import FakeLocator

PKG_DIR = Path(__file__).parent


def load_stations(line_cfg: dict, poses_cfg: dict) -> list[StationRuntime]:
    stations = []
    for s in line_cfg["stations"]:
        name = s["name"]
        arm = create_arm(name, s["robot"])
        arm.connect()

        poses = poses_cfg.get(name)
        if poses is None:
            raise KeyError(f"grid_poses.yaml 缺少工位 {name} 的位姿表")
        pick_wp = Waypoint.from_dict(poses["pick"])
        grid_wps = {cell: Waypoint.from_dict(d) for cell, d in poses["grid"].items()}

        vis_cfg = s.get("vision", {})
        if vis_cfg.get("enabled") and s["robot"]["driver"] != "sim":
            # 接真机时: PlanarCalibration.load(...) + ContourLocator(...)
            raise NotImplementedError("真机视觉接入见 docs/cell-control-design.md §5")
        locator = FakeLocator()

        st = StationRuntime(
            name=name, arm=arm, pick_wp=pick_wp, grid_wps=grid_wps,
            locator=locator, min_score=vis_cfg.get("min_score", 0.5),
        )
        st.label = s.get("label", name)      # 看板显示名
        st.brand = s.get("brand", "")        # 看板品牌标签
        st.kin = s["robot"].get("kin")       # 可选连杆长度(3D FK 逐型号更准)
        stations.append(st)
    return stations


def main() -> None:
    ap = argparse.ArgumentParser(description="工位单元控制器（仿真跑通版）")
    ap.add_argument("--cycles", type=int, default=3, help="跑几个步进节拍")
    ap.add_argument("--config", default=str(PKG_DIR / "config" / "line.yaml"))
    ap.add_argument("--poses", default=str(PKG_DIR / "config" / "grid_poses.yaml"))
    ap.add_argument("--dashboard", action="store_true", help="起操作看板 HTTP 服务")
    ap.add_argument("--port", type=int, default=8700, help="看板端口")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S",
    )

    line_cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    poses_cfg = yaml.safe_load(Path(args.poses).read_text(encoding="utf-8"))

    stations = load_stations(line_cfg, poses_cfg)
    locks = ZoneLockManager([z["name"] for z in line_cfg["zones"]])

    store = server = None
    if args.dashboard:
        store = StateStore(line_cfg["grid"])
        server = DashboardServer(store, port=args.port)
        server.start()

    ctrl = CellController(
        stations=stations, locks=locks, line=SimLineIO(),
        grid_cfg=line_cfg["grid"], line_cfg=line_cfg["line"], store=store,
    )

    for _ in range(args.cycles):
        ctrl.run_cycle()
        if args.dashboard:
            time.sleep(1.0)  # 放慢节拍便于在看板上观察

    for st in stations:
        st.arm.disconnect()
    logging.info("仿真结束: %d 个节拍全部完成", ctrl.cycle_count)
    if server:
        logging.info("看板仍在运行，Ctrl-C 退出。")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            server.stop()


if __name__ == "__main__":
    main()
