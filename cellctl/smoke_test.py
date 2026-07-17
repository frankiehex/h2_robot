"""单臂安全首触（Phase 1 第一步）—— 接真机后先跑这个，不碰产线。

按配置连接**指定一台**臂，做最小、最慢、最安全的验证：
  1. 连接 → 读当前 TCP 位姿（确认通信）
  2. （可选，--home）低速回 HOME 安全岛
  3. （可选，--grip）开合一次夹爪
不做任何取放/轨迹动作。跑通说明驱动层与该品牌真机打通。

用法:
  python -m cellctl.smoke_test --station S3_ham --config cellctl/config/fleet.example.yaml
  python -m cellctl.smoke_test --station S3_ham --config ... --home --grip

⚠ 现场务必：手持急停、周围无人、臂周围无障碍，再执行 --home。
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml

from .robots import create_arm

log = logging.getLogger("smoke")


def main() -> int:
    ap = argparse.ArgumentParser(description="单臂安全首触")
    ap.add_argument("--station", required=True, help="line.yaml 里的工位名")
    ap.add_argument("--config", required=True, help="配置文件路径")
    ap.add_argument("--home", action="store_true", help="低速回 HOME（现场确认安全后再用）")
    ap.add_argument("--grip", action="store_true", help="开合一次夹爪")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")

    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    st = next((s for s in cfg["stations"] if s["name"] == args.station), None)
    if st is None:
        log.error("配置里找不到工位 %s；可选: %s",
                  args.station, [s["name"] for s in cfg["stations"]])
        return 2

    arm = create_arm(args.station, st["robot"])
    log.info("准备连接 %s（%s @ %s）…", args.station,
             st["robot"]["driver"], st["robot"].get("host", "?"))
    try:
        arm.connect()
    except Exception as e:  # noqa: BLE001
        log.error("连接失败: %s", e)
        log.error("排查: 网线/网段、IP、机器人已上电使能、SDK 已安装")
        return 1

    try:
        pose = arm.read_pose()
        log.info("当前 TCP 位姿: %s", pose if pose is not None else "(该驱动未实现 read_pose)")

        if args.grip:
            log.info("夹爪开合测试…")
            arm.grip(True)
            arm.grip(False)
            log.info("夹爪 OK")

        if args.home:
            log.warning("低速回 HOME —— 确认周围安全，3 秒后执行（Ctrl-C 取消）")
            import time
            for i in (3, 2, 1):
                log.warning("  %d…", i)
                time.sleep(1)
            arm.go_home()
            log.info("已到 HOME，at_home=%s", arm.at_home)

        log.info("✅ 首触通过：%s 驱动层与真机打通", args.station)
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
