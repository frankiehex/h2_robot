"""位姿约定换算 —— 各品牌姿态表示不同，换算集中在这里。

本项目 Pose 约定：位置 mm，姿态为 **RPY 欧拉角（度）**，
旋转合成顺序 R = Rz(rz) · Ry(ry) · Rx(rx)（即先绕 X、再 Y、后 Z 的内旋 RPY）。

各品牌下发时的换算：
  - 法奥 Fairino：mm + 度 RPY，**与本约定一致，直接透传**。
  - 节卡 JAKA：mm + **弧度** RPY，只需 度→弧度。
  - 优傲 UR：米 + **旋转矢量(轴角)**，需 欧拉角→旋转矢量。

纯标准库实现（math），不依赖 numpy —— 控制路径不引入第三方依赖。
"""
from __future__ import annotations

import math


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def euler_deg_to_matrix(rx: float, ry: float, rz: float) -> list[list[float]]:
    """RPY 欧拉角(度) → 3×3 旋转矩阵，R = Rz·Ry·Rx。"""
    x, y, z = map(math.radians, (rx, ry, rz))
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return _matmul(Rz, _matmul(Ry, Rx))


def matrix_to_rotvec(R) -> tuple[float, float, float]:
    """旋转矩阵 → 旋转矢量(轴角，弧度)。UR 用。"""
    # 夹角 θ = acos((trace-1)/2)，带边界保护
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return (0.0, 0.0, 0.0)  # 近似无旋转
    if abs(math.pi - theta) < 1e-6:
        # θ≈π：从对角提取轴（对称矩阵，符号需单独处理）
        def axis_component(i):
            return math.sqrt(max(0.0, (R[i][i] + 1.0) / 2.0))
        ax, ay, az = axis_component(0), axis_component(1), axis_component(2)
        # 用非对角元定号
        if R[2][1] - R[1][2] < 0:
            ax = -ax
        if R[0][2] - R[2][0] < 0:
            ay = -ay
        if R[1][0] - R[0][1] < 0:
            az = -az
        n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        return (theta * ax / n, theta * ay / n, theta * az / n)
    k = theta / (2.0 * math.sin(theta))
    return ((R[2][1] - R[1][2]) * k,
            (R[0][2] - R[2][0]) * k,
            (R[1][0] - R[0][1]) * k)


def euler_deg_to_rotvec(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    """RPY 欧拉角(度) → 旋转矢量(弧度)。UR moveL 用。"""
    return matrix_to_rotvec(euler_deg_to_matrix(rx, ry, rz))


def euler_deg_to_rad(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    """度 → 弧度（姿态三分量）。JAKA 用。"""
    return (math.radians(rx), math.radians(ry), math.radians(rz))


def _selftest() -> None:
    """round-trip 与已知值自测。"""
    # 绕 Z 90°：旋转矢量应为 (0,0,π/2)
    v = euler_deg_to_rotvec(0, 0, 90)
    assert abs(v[0]) < 1e-9 and abs(v[1]) < 1e-9 and abs(v[2] - math.pi / 2) < 1e-9, v
    # 绕 X 180°：应为 (π,0,0)
    v = euler_deg_to_rotvec(180, 0, 0)
    assert abs(v[0] - math.pi) < 1e-6 and abs(v[1]) < 1e-6 and abs(v[2]) < 1e-6, v
    # 零旋转
    assert euler_deg_to_rotvec(0, 0, 0) == (0.0, 0.0, 0.0)
    # 度→弧度
    r = euler_deg_to_rad(90, -45, 0)
    assert abs(r[0] - math.pi / 2) < 1e-9 and abs(r[1] + math.pi / 4) < 1e-9
    # 复合角 round-trip：euler→R→rotvec→R 应与原 R 一致
    R1 = euler_deg_to_matrix(30, -20, 75)
    vx, vy, vz = matrix_to_rotvec(R1)
    ang = math.sqrt(vx * vx + vy * vy + vz * vz)
    # 由旋转矢量重建 R（Rodrigues），比对
    if ang > 1e-9:
        kx, ky, kz = vx / ang, vy / ang, vz / ang
        c, s = math.cos(ang), math.sin(ang)
        K = [[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]]
        KK = _matmul(K, K)
        R2 = [[(1 if i == j else 0) + s * K[i][j] + (1 - c) * KK[i][j]
               for j in range(3)] for i in range(3)]
        for i in range(3):
            for j in range(3):
                assert abs(R1[i][j] - R2[i][j]) < 1e-9, (i, j, R1[i][j], R2[i][j])
    print("geometry selftest OK")


if __name__ == "__main__":
    _selftest()
