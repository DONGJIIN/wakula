#!/usr/bin/env python3
"""Wakula SLAM/Nav2/感知链路的只读浏览器调试台。

工具故意作为独立 ROS 2 进程存在：它不启动或关闭 Gazebo、SLAM、Nav2、自主任务，也不
发布速度和 Action Goal。浏览器只集中显示算法已经发布的标准话题，因此同一套调试台既能
连接仓库仿真，也能连接未来真机。除 OpenCV JPEG 编码外不引入前端框架，适合 RK3588。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from math import degrees
from pathlib import Path
import threading
import time
from typing import Any, Dict, Optional
from urllib.parse import urlparse
import webbrowser

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String


MODE_NAMES = {0: "UNKNOWN", 1: "WALK", 2: "STEP", 3: "CLIMB", 4: "STOP"}
OBSTACLE_NAMES = {
    0: "UNKNOWN", 1: "CLEAR", 2: "STEP", 3: "PIT",
    4: "WALL", 5: "BAR", 6: "POLE",
}
PHASE_NAMES = {
    0: "INVALID", 1: "CLEAR", 2: "APPROACH", 3: "ALIGN", 4: "READY",
}


@dataclass
class StreamMetric:
    """一条话题的轻量接收统计；使用单调时钟，不受仿真时间跳变影响。"""

    count: int = 0
    last_received: float = 0.0
    intervals: deque = field(default_factory=lambda: deque(maxlen=30))

    def update(self, now: Optional[float] = None) -> None:
        current = time.monotonic() if now is None else float(now)
        if self.last_received > 0.0 and current >= self.last_received:
            self.intervals.append(current - self.last_received)
        self.last_received = current
        self.count += 1

    def public(self, now: Optional[float] = None) -> Dict[str, Any]:
        current = time.monotonic() if now is None else float(now)
        age = current - self.last_received if self.last_received else None
        positive = [value for value in self.intervals if value > 1e-6]
        hz = len(positive) / sum(positive) if positive else 0.0
        return {
            "count": self.count,
            "age_seconds": round(age, 3) if age is not None else None,
            "rate_hz": round(hz, 2),
            "healthy": bool(age is not None and age <= 2.0),
        }


def camera_info_summary(msg: CameraInfo) -> Dict[str, Any]:
    """提取相机标定是否有效及主要内参，供网页和单元测试共用。"""
    calibrated = (
        int(msg.width) > 0
        and int(msg.height) > 0
        and len(msg.k) == 9
        and float(msg.k[0]) > 0.0
        and float(msg.k[4]) > 0.0
    )
    return {
        "calibrated": calibrated,
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": str(msg.distortion_model),
        "fx": round(float(msg.k[0]), 3) if len(msg.k) == 9 else 0.0,
        "fy": round(float(msg.k[4]), 3) if len(msg.k) == 9 else 0.0,
        "cx": round(float(msg.k[2]), 3) if len(msg.k) == 9 else 0.0,
        "cy": round(float(msg.k[5]), 3) if len(msg.k) == 9 else 0.0,
        "distortion": [round(float(value), 6) for value in msg.d],
    }


class AlgorithmDebugDashboard(Node):
    """缓存算法只读状态并通过本机 HTTP 页面展示。"""

    def __init__(self) -> None:
        super().__init__("algorithm_debug_dashboard")
        self.declare_parameter("bind_address", "127.0.0.1")
        self.declare_parameter("port", 8088)
        self.declare_parameter("open_browser", True)
        self.declare_parameter("image_topic", "/vision/annotated_image")
        self.declare_parameter("report_directory", "reports/debug_dashboard")

        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._latest_jpeg: Optional[bytes] = None
        self._latest_map_jpeg: Optional[bytes] = None
        self._latest = {
            "front_obstacle_name": "等待感知数据",
            "autonomy_state": "NOT_RUNNING",
            "autonomy_progress": "自主任务未启动或尚未发布",
            "completed_obstacles": {},
            "pending_obstacles": {},
            "navigation_healthy": None,
            "safety": {},
            "guidance": {},
            "map": {},
            "odom": {},
            "plan": {},
            "camera_info": {"calibrated": False},
        }
        self._streams: Dict[str, StreamMetric] = {}
        self._started = time.monotonic()

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, image_topic, self._image_callback, reliable)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._camera_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", lambda msg: self._stream_only("/scan"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, latched)
        self.create_subscription(NavPath, "/plan", self._plan_callback, reliable)
        self.create_subscription(NavigationSafety, "/terrain/navigation_safety", self._safety_callback, reliable)
        self.create_subscription(TraversalGuidance, "/traversal/guidance", self._guidance_callback, reliable)
        self.create_subscription(String, "/perception/front_obstacle_name", self._front_name_callback, reliable)
        self.create_subscription(Bool, "/navigation/healthy", self._health_callback, reliable)
        self.create_subscription(String, "/autonomy/state", self._autonomy_state_callback, reliable)
        self.create_subscription(String, "/autonomy/progress", self._autonomy_progress_callback, reliable)
        self.create_subscription(String, "/autonomy/completed_obstacles", lambda msg: self._inventory_callback("completed_obstacles", msg), latched)
        self.create_subscription(String, "/autonomy/pending_obstacles", lambda msg: self._inventory_callback("pending_obstacles", msg), latched)

        address = str(self.get_parameter("bind_address").value)
        port = int(self.get_parameter("port").value)
        handler = self._make_handler()
        self._http = ThreadingHTTPServer((address, port), handler)
        self._http.daemon_threads = True
        self._http_thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._http_thread.start()
        url = f"http://{address}:{port}"
        self.get_logger().info(f"Wakula algorithm debug dashboard: {url}")
        if bool(self.get_parameter("open_browser").value):
            threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    def _mark(self, topic: str) -> None:
        self._streams.setdefault(topic, StreamMetric()).update()

    def _stream_only(self, topic: str) -> None:
        with self._lock:
            self._mark(topic)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        except (CvBridgeError, cv2.error) as exc:
            self.get_logger().warning(f"Cannot encode debug image: {exc}")
            return
        if ok:
            with self._lock:
                self._latest_jpeg = encoded.tobytes()
                self._mark(str(self.get_parameter("image_topic").value))

    def _camera_callback(self, msg: CameraInfo) -> None:
        with self._lock:
            self._latest["camera_info"] = camera_info_summary(msg)
            self._mark("/camera/camera_info")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        known = sum(1 for value in msg.data if int(value) >= 0)
        total = len(msg.data)
        width, height = int(msg.info.width), int(msg.info.height)
        encoded_map = None
        if width > 0 and height > 0 and total == width * height:
            values = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
            # RViz 习惯：未知灰、自由白、占用黑。OccupancyGrid 原点在左下，因此网页显示
            # 前先上下翻转；最近邻缩放保持每个栅格边界清楚，不伪造地图细节。
            image = np.full((height, width), 128, dtype=np.uint8)
            image[(values >= 0) & (values < 50)] = 245
            image[values >= 50] = 15
            image = np.flipud(image)
            scale = max(1, min(8, int(640 / max(width, height))))
            if scale > 1:
                image = cv2.resize(
                    image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST
                )
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                encoded_map = encoded.tobytes()
        with self._lock:
            self._latest["map"] = {
                "width": width,
                "height": height,
                "resolution": round(float(msg.info.resolution), 4),
                "known_ratio": round(known / total, 3) if total else 0.0,
            }
            if encoded_map is not None:
                self._latest_map_jpeg = encoded_map
            self._mark("/map")

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        twist = msg.twist.twist
        with self._lock:
            self._latest["odom"] = {
                "x": round(float(pose.x), 3), "y": round(float(pose.y), 3),
                "linear_x": round(float(twist.linear.x), 3),
                "angular_z": round(float(twist.angular.z), 3),
            }
            self._mark("/odom")

    def _plan_callback(self, msg: NavPath) -> None:
        with self._lock:
            self._latest["plan"] = {"poses": len(msg.poses), "frame_id": msg.header.frame_id}
            self._mark("/plan")

    def _safety_callback(self, msg: NavigationSafety) -> None:
        with self._lock:
            self._latest["safety"] = {
                "valid": bool(msg.perception_valid),
                "mode": MODE_NAMES.get(int(msg.mode), str(msg.mode)),
                "obstacle": OBSTACLE_NAMES.get(int(msg.obstacle_type), str(msg.obstacle_type)),
                "confidence": round(float(msg.confidence), 3),
                "distance_m": round(float(msg.distance), 3),
                "height_m": round(float(msg.obstacle_height), 3),
                "pit_depth_m": round(float(msg.pit_depth), 3),
                "slope_pitch_deg": round(degrees(float(msg.slope_pitch)), 2),
                "lateral_m": round(float(msg.lateral_offset), 3),
                "speed_limit": round(float(msg.speed_limit), 3),
                "visual_assist": bool(msg.visual_assist_active),
            }
            self._mark("/terrain/navigation_safety")

    def _guidance_callback(self, msg: TraversalGuidance) -> None:
        with self._lock:
            self._latest["guidance"] = {
                "valid": bool(msg.perception_valid),
                "phase": PHASE_NAMES.get(int(msg.phase), str(msg.phase)),
                "ready": bool(msg.ready_for_handoff),
                "traversal_required": bool(msg.traversal_required),
                "distance_m": round(float(msg.distance), 3),
                "lateral_m": round(float(msg.lateral_offset), 3),
                "heading_error_deg": round(degrees(float(msg.heading_error)), 2),
            }
            self._mark("/traversal/guidance")

    def _front_name_callback(self, msg: String) -> None:
        with self._lock:
            self._latest["front_obstacle_name"] = msg.data or "无障碍名称"
            self._mark("/perception/front_obstacle_name")

    def _health_callback(self, msg: Bool) -> None:
        with self._lock:
            self._latest["navigation_healthy"] = bool(msg.data)
            self._mark("/navigation/healthy")

    def _autonomy_state_callback(self, msg: String) -> None:
        with self._lock:
            self._latest["autonomy_state"] = msg.data or "UNKNOWN"
            self._mark("/autonomy/state")

    def _autonomy_progress_callback(self, msg: String) -> None:
        with self._lock:
            self._latest["autonomy_progress"] = msg.data
            self._mark("/autonomy/progress")

    def _inventory_callback(self, key: str, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"raw": msg.data}
        with self._lock:
            self._latest[key] = value
            self._mark(f"/autonomy/{'completed' if key.startswith('completed') else 'pending'}_obstacles")

    def status_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "generated_at_unix": round(time.time(), 3),
                "uptime_seconds": round(time.monotonic() - self._started, 1),
                **json.loads(json.dumps(self._latest, ensure_ascii=False)),
                "streams": {name: metric.public() for name, metric in sorted(self._streams.items())},
            }

    def export_report(self, include_image: bool = False) -> Dict[str, str]:
        """保存原子 JSON 状态；可选同时保存当前标注图，便于队员复盘。"""
        directory = Path(str(self.get_parameter("report_directory").value)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("wakula_debug_%Y%m%d_%H%M%S")
        report_path = directory / f"{stem}.json"
        report_path.write_text(
            json.dumps(self.status_snapshot(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        result = {"report": str(report_path.resolve())}
        if include_image:
            with self._lock:
                jpeg = self._latest_jpeg
            if jpeg:
                image_path = directory / f"{stem}.jpg"
                image_path.write_bytes(jpeg)
                result["image"] = str(image_path.resolve())
        return result

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def _send(self, data: bytes, content_type: str, status=HTTPStatus.OK):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/":
                    self._send(DASHBOARD_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/api/status":
                    payload = json.dumps(node.status_snapshot(), ensure_ascii=False).encode("utf-8")
                    self._send(payload, "application/json; charset=utf-8")
                elif path == "/image.jpg":
                    with node._lock:
                        image = node._latest_jpeg
                    if image:
                        self._send(image, "image/jpeg")
                    else:
                        self._send(b"image not received", "text/plain", HTTPStatus.SERVICE_UNAVAILABLE)
                elif path == "/map.jpg":
                    with node._lock:
                        image = node._latest_map_jpeg
                    if image:
                        self._send(image, "image/jpeg")
                    else:
                        self._send(b"map not received", "text/plain", HTTPStatus.SERVICE_UNAVAILABLE)
                else:
                    self._send(b"not found", "text/plain", HTTPStatus.NOT_FOUND)

            def do_POST(self):
                path = urlparse(self.path).path
                if path not in ("/api/report", "/api/snapshot"):
                    self._send(b"not found", "text/plain", HTTPStatus.NOT_FOUND)
                    return
                result = node.export_report(include_image=path.endswith("snapshot"))
                payload = json.dumps(result, ensure_ascii=False).encode("utf-8")
                self._send(payload, "application/json; charset=utf-8")

        return Handler

    def destroy_node(self):
        self._http.shutdown()
        self._http.server_close()
        return super().destroy_node()


DASHBOARD_HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Wakula 算法调试台</title>
<style>
:root{color-scheme:dark;--bg:#0b1220;--card:#111c2f;--line:#263752;--fg:#e6edf7;--muted:#95a7bf;--ok:#46d39a;--bad:#ff6b6b;--accent:#62a8ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}.wrap{max-width:1500px;margin:auto;padding:18px}h1{font-size:22px;margin:0 0 4px}.sub{color:var(--muted);margin-bottom:16px}.grid{display:grid;grid-template-columns:minmax(480px,1.6fr) minmax(360px,1fr);gap:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:14px}h2{font-size:15px;margin:0 0 10px;color:#bdd6f7}.hero{font-size:25px;font-weight:700;color:var(--accent)}img{width:100%;min-height:280px;object-fit:contain;background:#05080e;border-radius:8px}table{width:100%;border-collapse:collapse}td,th{padding:6px;border-bottom:1px solid var(--line);text-align:left}.ok{color:var(--ok)}.bad{color:var(--bad)}.pill{display:inline-block;padding:3px 8px;border-radius:20px;background:#1b2b45;margin:2px}button{background:#245f9e;color:white;border:0;border-radius:7px;padding:8px 12px;margin-right:8px;cursor:pointer}pre{white-space:pre-wrap;color:#c5d2e4}@media(max-width:900px){.grid{grid-template-columns:1fr}.wrap{padding:10px}}
</style></head><body><div class="wrap">
<h1>Wakula SLAM · Nav2 · OpenCV 调试台</h1><div class="sub">只读监控，不发布速度、不调用越障 Action、不改变三个主进程。</div>
<div class="grid"><main>
<section class="card"><h2>相机标注画面</h2><img id="camera" alt="等待 /vision/annotated_image"></section>
<section class="card"><h2>SLAM 占据地图</h2><img id="map" alt="等待 /map"></section>
<section class="card"><h2>话题健康与频率</h2><table><thead><tr><th>话题</th><th>频率</th><th>延迟</th><th>状态</th></tr></thead><tbody id="streams"></tbody></table></section>
</main><aside>
<section class="card"><h2>当前正前方障碍</h2><div class="hero" id="obstacle">等待数据</div><div id="safety"></div></section>
<section class="card"><h2>越障与任务状态</h2><div id="guidance"></div><pre id="autonomy"></pre><div id="inventory"></div></section>
<section class="card"><h2>SLAM / Nav2</h2><div id="navigation"></div></section>
<section class="card"><h2>相机标定检查</h2><div id="calibration"></div></section>
<section class="card"><h2>复盘导出</h2><button onclick="save('/api/report')">导出诊断 JSON</button><button onclick="save('/api/snapshot')">保存状态 + 截图</button><pre id="saved"></pre></section>
</aside></div></div>
<script>
const esc=x=>String(x??'—').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const kv=o=>Object.entries(o||{}).map(([k,v])=>`<span class="pill">${esc(k)}: ${esc(v)}</span>`).join('');
async function tick(){try{let r=await fetch('/api/status');let d=await r.json();
document.getElementById('camera').src='/image.jpg?t='+Date.now();
document.getElementById('map').src='/map.jpg?t='+Date.now();
document.getElementById('obstacle').textContent=d.front_obstacle_name;
document.getElementById('safety').innerHTML=kv(d.safety);
document.getElementById('guidance').innerHTML=kv(d.guidance);
document.getElementById('autonomy').textContent=`状态: ${d.autonomy_state}\n${d.autonomy_progress}`;
document.getElementById('inventory').innerHTML='<b>已完成</b> '+kv(d.completed_obstacles)+'<br><b>待完成</b> '+kv(d.pending_obstacles);
document.getElementById('navigation').innerHTML=`导航健康: <b class="${d.navigation_healthy?'ok':'bad'}">${esc(d.navigation_healthy)}</b><br>`+kv(d.map)+kv(d.odom)+kv(d.plan);
let c=d.camera_info||{};document.getElementById('calibration').innerHTML=`<b class="${c.calibrated?'ok':'bad'}">${c.calibrated?'内参有效':'未收到有效内参'}</b><br>`+kv(c);
document.getElementById('streams').innerHTML=Object.entries(d.streams).map(([n,s])=>`<tr><td>${esc(n)}</td><td>${s.rate_hz} Hz</td><td>${s.age_seconds??'—'} s</td><td class="${s.healthy?'ok':'bad'}">${s.healthy?'正常':'断流/等待'}</td></tr>`).join('');
}catch(e){document.getElementById('saved').textContent='调试台连接失败: '+e}setTimeout(tick,700)}
async function save(path){let r=await fetch(path,{method:'POST'});document.getElementById('saved').textContent=JSON.stringify(await r.json(),null,2)}tick();
</script></body></html>"""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlgorithmDebugDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
