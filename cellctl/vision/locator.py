"""来料定位：一帧图 → (x_mm, y_mm, θ_deg) 基座系坐标。

算法梯度（够用就停）：
  1. 轮廓 + 最小外接矩形（本文件实现）—— 火腿等外形规整切片
  2. 旋转模板匹配 —— 轮廓不稳时
  3. 分割网络出掩膜 —— 蛋皮等边缘发虚的，二期

score 低于阈值 → found=False，按"空格/来料异常"跳过并记录，不猜。
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from .calibration import PlanarCalibration

log = logging.getLogger(__name__)


@dataclass
class VisionResult:
    found: bool
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_deg: float = 0.0
    score: float = 0.0


class ContourLocator:
    """HSV 分割 → 最大轮廓 → minAreaRect。打光罩 + 固定曝光是前提。"""

    def __init__(self, calib: PlanarCalibration, min_score: float = 0.6,
                 min_area_px: int = 5000):
        self.calib = calib
        self.min_score = min_score
        self.min_area_px = min_area_px
        # HSV 阈值按现场打光调（火腿偏粉红的默认起点）
        self.hsv_lo = (0, 40, 60)
        self.hsv_hi = (20, 255, 255)

    def locate(self, frame) -> VisionResult:
        import cv2
        import numpy as np

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.hsv_lo), np.array(self.hsv_hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return VisionResult(found=False)

        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < self.min_area_px:
            return VisionResult(found=False, score=0.0)

        (cu, cv_), (w, h), theta = cv2.minAreaRect(biggest)
        # 矩形填充度作为置信分：轮廓破碎/粘连时下降
        score = float(area / max(w * h, 1.0))
        if score < self.min_score:
            log.warning("定位置信度低 score=%.2f，按来料异常跳过", score)
            return VisionResult(found=False, score=score)

        x_mm, y_mm = self.calib.pixel_to_base(cu, cv_)
        theta_base = self.calib.theta_to_base(theta)
        return VisionResult(True, x_mm, y_mm, theta_base, score)


class FakeLocator:
    """仿真用：返回固定小偏差，验证'视觉偏差叠加到取料点'的通路。"""

    def __init__(self, dx: float = 3.5, dy: float = -2.0, dtheta: float = 4.0):
        self.result = VisionResult(True, dx, dy, dtheta, score=0.99)

    def locate(self, frame=None) -> VisionResult:
        return self.result
