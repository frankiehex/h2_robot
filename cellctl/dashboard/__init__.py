"""操作看板：控制器状态的实时可视化。

- StateStore：线程安全快照
- DashboardServer：标准库 HTTP 服务，供 panel.html 轮询
- panel.html：同一文件双模式 —— 被控制器托管时显示 LIVE 实时状态；
  单独打开时在网页内跑仿真（DEMO），无需硬件即可查看。
"""
from .server import DashboardServer
from .state_store import StateStore

__all__ = ["DashboardServer", "StateStore"]
