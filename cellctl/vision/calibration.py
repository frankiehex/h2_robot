"""2D 视觉标定：像素 → 机器人基座坐标。

工作面是固定高度的平面，所以两步就够，不做完整 3D 内外参：
  1. 平面单应 H：像素 → 工作平面 mm（棋盘格/9 点标定）
  2. 2D 刚体变换 T：工作平面 mm → 机器人基座 mm
     （机器人夹标定针走 3-4 个点，同时记像素与基座坐标）

结果存 yaml，换相机/撞了支架才需要重标。

独立运行采集标定数据:
  python -m cellctl.vision.calibration --station S3_ham --camera 0
"""
from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np

log = logging.getLogger(__name__)


@dataclass
class PlanarCalibration:
    """像素 → 基座系的复合映射。theta 偏差直接可用（刚体变换保角）。

    numpy 在方法内部导入：仿真跑通（FakeLocator）不需要视觉依赖。
    """
    homography: "np.ndarray"    # 3x3, 像素 → 平面 mm
    rigid: "np.ndarray"         # 2x3, 平面 mm → 基座 mm ([R|t])
    theta_offset_deg: float     # 相机 θ=0 方向与基座 X 轴的夹角

    def pixel_to_base(self, u: float, v: float) -> tuple[float, float]:
        import numpy as np
        p = self.homography @ np.array([u, v, 1.0])
        plane = p[:2] / p[2]
        base = self.rigid @ np.array([plane[0], plane[1], 1.0])
        return float(base[0]), float(base[1])

    def theta_to_base(self, theta_img_deg: float) -> float:
        return theta_img_deg + self.theta_offset_deg

    def save(self, path: str | Path) -> None:
        import yaml
        data = {
            "homography": self.homography.tolist(),
            "rigid": self.rigid.tolist(),
            "theta_offset_deg": float(self.theta_offset_deg),
        }
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(yaml.safe_dump(data), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "PlanarCalibration":
        import numpy as np
        import yaml
        d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(
            homography=np.array(d["homography"]),
            rigid=np.array(d["rigid"]),
            theta_offset_deg=float(d["theta_offset_deg"]),
        )

    @classmethod
    def identity(cls) -> "PlanarCalibration":
        """未标定占位（仿真用）：像素直接当 mm。"""
        import numpy as np
        return cls(np.eye(3), np.array([[1.0, 0, 0], [0, 1.0, 0]]), 0.0)


def solve_homography(pixel_pts: "np.ndarray", plane_pts_mm: "np.ndarray") -> "np.ndarray":
    """≥4 组对应点求单应。棋盘格标定时点数多，最小二乘更稳。"""
    import cv2
    import numpy as np
    H, _ = cv2.findHomography(pixel_pts.astype(np.float64), plane_pts_mm.astype(np.float64))
    if H is None:
        raise RuntimeError("单应求解失败：检查对应点")
    return H


def solve_rigid_2d(plane_pts_mm: "np.ndarray", base_pts_mm: "np.ndarray") -> "np.ndarray":
    """≥2 组对应点求 2D 刚体变换（SVD, Umeyama 不含缩放）。

    采集方法：机器人夹标定针依次压在 3-4 个标记点上，
    记录基座坐标；同一批点在图像里取像素坐标过单应得平面坐标。
    """
    import numpy as np
    src = plane_pts_mm.mean(axis=0)
    dst = base_pts_mm.mean(axis=0)
    src_c = plane_pts_mm - src
    dst_c = base_pts_mm - dst
    U, _, Vt = np.linalg.svd(src_c.T @ dst_c)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = dst - R @ src
    return np.hstack([R, t.reshape(2, 1)])


def _main() -> None:
    ap = argparse.ArgumentParser(description="交互式采集标定数据（需要相机与真机）")
    ap.add_argument("--station", required=True)
    ap.add_argument("--camera", type=int, default=0)
    ap.parse_args()
    raise SystemExit(
        "接真机时实现交互流程：拍棋盘格 → solve_homography → "
        "机器人走 4 点 → solve_rigid_2d → PlanarCalibration.save()"
    )


if __name__ == "__main__":
    _main()
