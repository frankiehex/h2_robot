"""操作看板 HTTP 服务（Python 标准库，无第三方依赖）。

后台线程起一个 http.server：
  GET /            → panel.html（面板）
  GET /state.json  → 当前状态快照（面板每 ~0.5s 轮询）
控制器主循环照常跑，只通过 StateStore 发布快照，二者解耦。

用法见 main.py 的 --dashboard 开关。
"""
from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .state_store import StateStore

log = logging.getLogger(__name__)
PANEL = Path(__file__).parent / "panel.html"


def make_handler(store: StateStore):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # 静音默认访问日志
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html", "/panel.html"):
                try:
                    self._send(200, PANEL.read_bytes(), "text/html; charset=utf-8")
                except OSError:
                    self._send(500, b"panel.html missing", "text/plain")
            elif path == "/state.json":
                body = json.dumps(store.snapshot(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


class DashboardServer:
    def __init__(self, store: StateStore, host: str = "127.0.0.1", port: int = 8700):
        self.store = store
        self.host, self.port = host, port
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._httpd = ThreadingHTTPServer((self.host, self.port), make_handler(self.store))
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="dashboard", daemon=True)
        self._thread.start()
        log.info("看板已启动 → http://%s:%d", self.host, self.port)

    def stop(self) -> None:
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
