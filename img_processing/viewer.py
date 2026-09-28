"""Web-based viewer for testing color & shape detection with manual gimbal control.

Features:
- Live camera stream (RoboMaster EP or USB Webcam).
- Manual Gimbal handling: 'Gimbal Suspend/Free' mode allows physically turning
  the gimbal by hand without motor resistance.
- Toggle between 'Robust Mode' (perspective & close-range tolerance) and 'Classic Mode'.
- Real-time Lens Undistortion toggle.
- Live inspection of Color Masks and Shape Geometry metrics.
- NO auto-tracking, NO auto-aiming, NO auto-firing.
"""

from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from queue import Empty
import threading
import time
from urllib.parse import parse_qs, urlsplit

from typing import Dict, List, Optional, Tuple, Union
import cv2
import numpy as np

from pathlib import Path
import sys

# Support running directly or as a module
_pkg_dir = Path(__file__).resolve().parent
_root_dir = _pkg_dir.parent
for _p in (str(_pkg_dir), str(_root_dir)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from .config import COLORS, SHAPES, DEFAULT_DETECTION_CONFIG
    from .detector import ColorShapeDetector, Detection, COLORS as DET_COLORS
except (ImportError, ValueError):
    from config import COLORS, SHAPES, DEFAULT_DETECTION_CONFIG
    from detector import ColorShapeDetector, Detection, COLORS as DET_COLORS


PAGE_HTML = """<!doctype html>
<html lang="th">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Color & Shape Diagnostic Viewer (Manual Gimbal)</title>
  <style>
    :root {
      --bg: #12181f;
      --card-bg: #1a232c;
      --card-border: #2a3845;
      --accent: #2e86de;
      --accent-hover: #1e6ec2;
      --text: #e6eff5;
      --text-muted: #8b9eb0;
      --success: #10ac84;
      --warning: #ee5253;
      --tag-bg: #222f3e;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Noto Sans Thai", sans-serif;
      background: var(--bg);
      color: var(--text);
      line-height: 1.5;
      padding: 16px 20px;
    }
    header {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 16px;
      padding-bottom: 12px;
      border-bottom: 1px solid var(--card-border);
    }
    h1 { font-size: 1.4rem; font-weight: 600; }
    .badge {
      display: inline-block;
      font-size: 0.75rem;
      padding: 3px 8px;
      border-radius: 4px;
      background: var(--tag-bg);
      color: var(--text-muted);
      margin-left: 8px;
    }
    .toolbar {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 12px 16px;
      margin-bottom: 16px;
    }
    .control-group {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 0.9rem;
    }
    select, button, input {
      font: inherit;
      background: #253340;
      color: var(--text);
      border: 1px solid var(--card-border);
      padding: 6px 12px;
      border-radius: 6px;
      outline: none;
    }
    select:focus, button:focus { border-color: var(--accent); }
    button {
      cursor: pointer;
      font-weight: 500;
      transition: background 0.15s;
    }
    button:hover:not(:disabled) { background: #2f4052; }
    button.primary {
      background: var(--accent);
      border-color: var(--accent);
      color: #fff;
    }
    button.primary:hover:not(:disabled) { background: var(--accent-hover); }
    button.danger {
      background: #c0392b;
      border-color: #c0392b;
      color: #fff;
    }
    button.active {
      background: var(--success);
      border-color: var(--success);
      color: #fff;
    }
    button:disabled { opacity: 0.5; cursor: not-allowed; }

    .views-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      margin-bottom: 16px;
    }
    @media (max-width: 900px) {
      .views-grid { grid-template-columns: 1fr; }
    }
    .panel {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 14px;
      display: flex;
      flex-direction: column;
    }
    .panel-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 10px;
      font-weight: 500;
      font-size: 0.95rem;
    }
    .img-container {
      width: 100%;
      height: 380px;
      background: #0d1217;
      border-radius: 6px;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    img {
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
      display: block;
    }
    .results-table-card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 14px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.88rem;
      margin-top: 8px;
    }
    th, td {
      padding: 8px 10px;
      text-align: left;
      border-bottom: 1px solid var(--card-border);
    }
    th {
      color: var(--text-muted);
      font-weight: 600;
      background: #151d24;
    }
    .color-pill {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      margin-right: 6px;
      vertical-align: middle;
    }
    #status-bar {
      font-size: 0.88rem;
      color: var(--text-muted);
      margin-top: 8px;
    }
    .mode-tag {
      padding: 2px 6px;
      border-radius: 4px;
      font-size: 0.75rem;
      font-weight: bold;
    }
    .mode-robust { background: #1b4d3e; color: #55efc4; }
    .mode-classic { background: #4a3c1c; color: #ffeaa7; }
  </style>
</head>
<body>

  <header>
    <div>
      <h1>Color & Shape Testing <span class="badge">Manual Gimbal Mode</span></h1>
      <div id="status-bar">พร้อมทำงาน — กรุณาเลือกแหล่งภาพแล้วกด "เชื่อมต่อกล้อง"</div>
    </div>
    <div style="font-size: 0.85rem; color: var(--text-muted);">
      หมุนกิมบอลด้วยมือเองได้ / ปรับโหมดแก้ระยะชิดได้ทันที
    </div>
  </header>

  <!-- Controls Toolbar -->
  <div class="toolbar">
    <div class="control-group">
      <label for="source">แหล่งภาพ:</label>
      <select id="source">
        <option value="ep_ap">RoboMaster EP (Wi-Fi AP)</option>
        <option value="ep_rndis">RoboMaster EP (USB RNDIS)</option>
        <option value="webcam0">Webcam 0 (โน้ตบุ๊ก/USB)</option>
        <option value="webcam1">Webcam 1</option>
      </select>
      <button id="btn-start" class="primary" onclick="startCamera()">เริ่มทำงาน</button>
      <button id="btn-stop" class="danger" onclick="stopCamera()" disabled>หยุด</button>
    </div>

    <div class="control-group" style="border-left: 1px solid var(--card-border); padding-left: 12px;">
      <label for="det-mode">อัลกอริทึม:</label>
      <button id="btn-mode" onclick="toggleMode()">
        โหมด: <span id="mode-label" class="mode-tag mode-robust">Robust (แก้ระยะชิด)</span>
      </button>
      <button id="btn-undistort" onclick="toggleUndistort()">
        แก้เลนส์โค้ง: <span id="undistort-label" style="color:#55efc4;">เปิด</span>
      </button>
    </div>

    <div class="control-group" style="border-left: 1px solid var(--card-border); padding-left: 12px;">
      <label>กิมบอล:</label>
      <button id="btn-suspend" onclick="toggleGimbalFree()" title="ปลดมอเตอร์เพื่อให้สามารถใช้มือจับกิมบอลหมุนได้อิสระ">
        ปลดกิมบอลหมุนด้วยมือ: <span id="gimbal-state" style="color:#ffeaa7;">พร้อม</span>
      </button>
      <button onclick="recenterGimbal()" title="สั่งกิมบอลกลับสู่จุดกึ่งกลาง">จัดกึ่งกลาง</button>
    </div>
  </div>

  <!-- Video and Mask Split View -->
  <div class="views-grid">
    <div class="panel">
      <div class="panel-header">
        <span>ภาพกล้อง + ผลตรวจจับรูปทรงและสี</span>
        <span id="fps-label" style="font-size: 0.8rem; color: var(--text-muted);">0 FPS</span>
      </div>
      <div class="img-container">
        <img id="frame-img" alt="รอภาพกล้อง...">
      </div>
    </div>

    <div class="panel">
      <div class="panel-header">
        <span>พิกเซลที่ผ่านการกรองสี (Mask)</span>
        <div class="control-group">
          <label for="mask-color" style="font-size:0.85rem">สี:</label>
          <select id="mask-color" onchange="changeMaskColor()">
            <option value="all">ทุกสี (รวมกัน)</option>
            <option value="red">แดง (Red)</option>
            <option value="green">เขียว (Green)</option>
            <option value="yellow">เหลือง (Yellow)</option>
            <option value="blue">น้ำเงิน (Blue)</option>
          </select>
        </div>
      </div>
      <div class="img-container">
        <img id="mask-img" alt="รอ Mask...">
      </div>
    </div>
  </div>

  <!-- Results Table -->
  <div class="results-table-card">
    <div style="display:flex; justify-content:space-between; align-items:center;">
      <h3 style="font-size: 1.05rem;">เป้าหมายที่ตรวจพบในเฟรมล่าสุด</h3>
      <span id="summary-count" style="font-size: 0.85rem; color: var(--text-muted);">พบ 0 ชิ้น</span>
    </div>
    <table>
      <thead>
        <tr>
          <th>สี</th>
          <th>รูปทรง</th>
          <th>ขนาด (px²)</th>
          <th>จุดศูนย์กลาง (X, Y)</th>
          <th>สัดส่วนด้าน (Ratio)</th>
          <th>Ellipse Fit (สำหรับวงกลม)</th>
          <th>จำนวนมุม (Corners)</th>
          <th>สถานะ / เหตุผล</th>
        </tr>
      </thead>
      <tbody id="results-body">
        <tr><td colspan="8" style="text-align:center; color:var(--text-muted); padding: 18px;">ยังไม่มีการตรวจจับ</td></tr>
      </tbody>
    </table>
  </div>

  <script>
    let currentMode = "robust";
    let enableUndistort = true;
    let gimbalFree = false;
    let activeMask = "all";

    const colorHex = {
      red: "#ff4d4d",
      green: "#2ecc71",
      yellow: "#f1c40f",
      blue: "#3498db"
    };

    const colorTh = { red: "แดง", green: "เขียว", yellow: "เหลือง", blue: "น้ำเงิน" };
    const shapeTh = { circle: "วงกลม", square: "จัตุรัส", horizontal: "ผืนผ้าแนวนอน", vertical: "ผืนผ้าแนวตั้ง", unknown: "ไม่ระบุ" };

    async function startCamera() {
      const src = document.getElementById("source").value;
      document.getElementById("status-bar").textContent = "กำลังเชื่อมต่อ...";
      const res = await fetch("/api/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ source: src, mode: currentMode, undistort: enableUndistort })
      });
      const data = await res.json();
      document.getElementById("status-bar").textContent = data.status;
    }

    async function stopCamera() {
      document.getElementById("status-bar").textContent = "กำลังหยุด...";
      const res = await fetch("/api/stop", { method: "POST" });
      const data = await res.json();
      document.getElementById("status-bar").textContent = data.status;
    }

    async function toggleMode() {
      currentMode = (currentMode === "robust") ? "classic" : "robust";
      const lbl = document.getElementById("mode-label");
      if (currentMode === "robust") {
        lbl.className = "mode-tag mode-robust";
        lbl.textContent = "Robust (แก้ระยะชิด)";
      } else {
        lbl.className = "mode-tag mode-classic";
        lbl.textContent = "Classic (ดั้งเดิม)";
      }
      await fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: currentMode })
      });
    }

    async function toggleUndistort() {
      enableUndistort = !enableUndistort;
      const lbl = document.getElementById("undistort-label");
      lbl.textContent = enableUndistort ? "เปิด" : "ปิด";
      lbl.style.color = enableUndistort ? "#55efc4" : "#ee5253";
      await fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ undistort: enableUndistort })
      });
    }

    async function toggleGimbalFree() {
      gimbalFree = !gimbalFree;
      const res = await fetch("/api/gimbal", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: gimbalFree ? "suspend" : "resume" })
      });
      const data = await res.json();
      const stateLbl = document.getElementById("gimbal-state");
      stateLbl.textContent = gimbalFree ? "ปล่อยอิสระ (จับหมุนได้)" : "ล็อกตำแหน่ง (มอเตอร์ทำงาน)";
      stateLbl.style.color = gimbalFree ? "#55efc4" : "#ffeaa7";
    }

    async function recenterGimbal() {
      await fetch("/api/gimbal", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "recenter" })
      });
    }

    function changeMaskColor() {
      activeMask = document.getElementById("mask-color").value;
    }

    async function refreshLoop() {
      try {
        const stateRes = await fetch("/api/state", { cache: "no-store" });
        if (stateRes.ok) {
          const state = await stateRes.json();
          document.getElementById("status-bar").textContent = state.status;
          document.getElementById("btn-start").disabled = state.running;
          document.getElementById("btn-stop").disabled = !state.running;
          document.getElementById("fps-label").textContent = `${state.fps.toFixed(1)} FPS`;

          if (state.has_frame) {
            const t = Date.now();
            document.getElementById("frame-img").src = `/frame.jpg?t=${t}`;
            document.getElementById("mask-img").src = `/mask.jpg?color=${activeMask}&t=${t}`;
          }

          // Update detections table
          const tbody = document.getElementById("results-body");
          tbody.replaceChildren();
          document.getElementById("summary-count").textContent = `พบ ${state.detections.length} ชิ้น`;

          if (state.detections.length === 0) {
            const row = document.createElement("tr");
            row.innerHTML = `<td colspan="8" style="text-align:center; color:var(--text-muted); padding: 18px;">ไม่พบวัตถุที่ผ่านเกณฑ์</td>`;
            tbody.appendChild(row);
          } else {
            for (const d of state.detections) {
              const row = document.createElement("tr");
              const m = d.metrics || {};
              const ratioStr = m.ratio ? m.ratio.toFixed(2) : "-";
              const ellipseStr = m.ellipse_fit_ratio ? `${m.ellipse_fit_ratio.toFixed(2)} (แกน ${m.ellipse_axis_ratio ? m.ellipse_axis_ratio.toFixed(2) : '-'})` : "-";
              const cornersStr = m.corners !== undefined ? m.corners : (m.angles ? m.angles.length : "-");
              const reasonStr = d.reasons && d.reasons.length > 0 ? d.reasons.join(", ") : "<span style='color:#10ac84'>ผ่าน (PASS)</span>";

              row.innerHTML = `
                <td><span class="color-pill" style="background:${colorHex[d.color]||'#fff'}"></span>${colorTh[d.color] || d.color}</td>
                <td><b>${shapeTh[d.shape] || d.shape}</b></td>
                <td>${Math.round(d.area)}</td>
                <td>(${d.center[0]}, ${d.center[1]})</td>
                <td>${ratioStr}</td>
                <td>${ellipseStr}</td>
                <td>${cornersStr}</td>
                <td>${reasonStr}</td>
              `;
              tbody.appendChild(row);
            }
          }
        }
      } catch (err) {
        // Ignored connection hiccups
      }
      setTimeout(refreshLoop, 250);
    }

    refreshLoop();
  </script>
</body>
</html>
"""


class LiveCameraManager:
    """Manages robot or webcam streaming, detection thread, and gimbal interaction."""

    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.status = "พร้อมทำงาน — เลือกแหล่งภาพแล้วกด เริ่มทำงาน"
        self.running = False
        self.frame_jpeg = None
        self.masks = {}
        self.detections = []
        self.fps = 0.0
        self.mode = "robust"
        self.undistort = True
        self.robot_ep = None
        self.gimbal_suspended = False
        self.detector = ColorShapeDetector(mode="robust", enable_undistort=True)

    def start(self, source: str) -> Tuple[bool, str]:
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False, "กล้องกำลังทำงานอยู่แล้ว"
            self.stop_event = threading.Event()
            self.running = True
            self.status = "กำลังเชื่อมต่อ..."
            self.thread = threading.Thread(target=self._run_capture, args=(source,), daemon=True)
            self.thread.start()
            return True, self.status

    def stop(self) -> str:
        with self.lock:
            if self.thread and self.thread.is_alive():
                self.status = "กำลังตัดการเชื่อมต่อ..."
                self.stop_event.set()
            return self.status

    def set_config(self, mode: Optional[str] = None, undistort: Optional[bool] = None):
        with self.lock:
            if mode in ("robust", "classic"):
                self.mode = mode
                self.detector.mode = mode
            if undistort is not None:
                self.undistort = bool(undistort)
                self.detector.enable_undistort = self.undistort

    def control_gimbal(self, action: str) -> Tuple[bool, str]:
        with self.lock:
            if self.robot_ep is None:
                return False, "หุ่นยนต์ RoboMaster EP ยังไม่ได้เชื่อมต่อ"
            try:
                if action == "suspend":
                    self.robot_ep.gimbal.suspend()
                    self.gimbal_suspended = True
                    return True, "ปลดกิมบอลแล้ว (สามารถจับหมุนด้วยมือได้)"
                elif action == "resume":
                    self.robot_ep.gimbal.resume()
                    self.gimbal_suspended = False
                    return True, "เปิดมอเตอร์กิมบอลแล้ว"
                elif action == "recenter":
                    self.robot_ep.gimbal.resume()
                    self.robot_ep.gimbal.recenter()
                    self.gimbal_suspended = False
                    return True, "จัดกิมบอลกึ่งกลางสำเร็จ"
            except Exception as e:
                return False, f"คำสั่งกิมบอลล้มเหลว: {e}"
            return False, "ไม่รู้จักคำสั่ง"

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "status": self.status,
                "running": self.running,
                "has_frame": self.frame_jpeg is not None,
                "fps": self.fps,
                "mode": self.mode,
                "undistort": self.undistort,
                "gimbal_suspended": self.gimbal_suspended,
                "detections": [
                    {
                        "color": d.color,
                        "shape": d.shape,
                        "area": d.area,
                        "center": d.center,
                        "metrics": d.metrics,
                        "reasons": d.reasons,
                        "mode": d.mode,
                    }
                    for d in self.detections
                ],
            }

    def get_image(self, color: str = "frame") -> Optional[bytes]:
        with self.lock:
            if color == "frame":
                return self.frame_jpeg
            if not self.masks:
                return None
            if color == "all":
                combined = np.zeros_like(next(iter(self.masks.values())))
                for m in self.masks.values():
                    combined |= m
                mask_to_encode = combined
            elif color in self.masks:
                mask_to_encode = self.masks[color]
            else:
                return None
            success, encoded = cv2.imencode(".jpg", mask_to_encode)
            return encoded.tobytes() if success else None

    def _run_capture(self, source: str):
        camera = None
        stream_started = False
        is_ep = source.startswith("ep_")

        try:
            if is_ep:
                from robomaster import robot, conn
                conn_type = conn.CONNECTION_USB_RNDIS if source == "ep_rndis" else conn.CONNECTION_WIFI_AP
                self.robot_ep = robot.Robot()
                self.robot_ep.initialize(conn_type=conn_type)
                # By default, put gimbal in suspend so user can manually rotate it by hand
                self.robot_ep.gimbal.suspend()
                self.gimbal_suspended = True
                camera = self.robot_ep.camera
                stream_started = camera.start_video_stream(display=False)
                if not stream_started:
                    raise RuntimeError("ไม่สามารถเริ่มวิดีโอสตรีมจาก RoboMaster EP ได้")
            else:
                cam_id = int(source[-1])
                camera = cv2.VideoCapture(cam_id)
                if not camera.isOpened():
                    raise RuntimeError(f"ไม่สามารถเปิด Webcam {cam_id} ได้")

            with self.lock:
                self.status = f"กำลังสตรีมจาก {source}"

            frame_times = []
            while not self.stop_event.is_set():
                t0 = time.time()
                if is_ep:
                    try:
                        frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
                    except Empty:
                        continue
                else:
                    ret, frame = camera.read()
                    if not ret:
                        continue

                if frame is None:
                    continue

                # Run detection with current mode & undistort settings
                detections, masks, proc_frame = self.detector.detect(frame, debug=True)

                # Annotate frame
                annotated = proc_frame.copy()
                for d in detections:
                    if d.shape == "unknown":
                        continue
                    bgr = COLORS[d.color]["bgr"]
                    cv2.drawContours(annotated, [d.contour], -1, bgr, 2)
                    cv2.circle(annotated, d.center, 5, bgr, -1)
                    x, y, w, h = cv2.boundingRect(d.contour)
                    text = f"{d.color} {d.shape}"
                    cv2.putText(annotated, text, (x, max(20, y - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

                success, enc = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if success:
                    t1 = time.time()
                    frame_times.append(t1 - t0)
                    if len(frame_times) > 15:
                        frame_times.pop(0)
                    fps_val = len(frame_times) / sum(frame_times) if sum(frame_times) > 0 else 0.0

                    with self.lock:
                        self.frame_jpeg = enc.tobytes()
                        self.masks = masks
                        self.detections = [d for d in detections if d.shape != "unknown"]
                        self.fps = fps_val

        except Exception as exc:
            with self.lock:
                self.status = f"เกิดข้อผิดพลาด: {exc}"
        finally:
            with self.lock:
                self.running = False
                self.status = "หยุดการทำงานแล้ว"
            if camera is not None:
                try:
                    if is_ep and stream_started:
                        camera.stop_video_stream()
                    elif not is_ep:
                        camera.release()
                except Exception:
                    pass
            if self.robot_ep is not None:
                try:
                    self.robot_ep.close()
                except Exception:
                    pass
                self.robot_ep = None


camera_manager = LiveCameraManager()


class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/":
            self._send(PAGE_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif url.path == "/api/state":
            data = json.dumps(camera_manager.snapshot()).encode("utf-8")
            self._send(data, "application/json; charset=utf-8")
        elif url.path in ("/frame.jpg", "/mask.jpg"):
            color = "frame" if url.path == "/frame.jpg" else parse_qs(url.query).get("color", ["all"])[0]
            img_bytes = camera_manager.get_image(color)
            if img_bytes is None:
                self.send_error(503, "Waiting for image")
                return
            self._send(img_bytes, "image/jpeg")
        else:
            self.send_error(404)

    def do_POST(self):
        url = urlsplit(self.path)
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length > 0 else {}
        except Exception:
            body = {}

        if url.path == "/api/start":
            src = body.get("source", "ep_ap")
            ok, msg = camera_manager.start(src)
            self._send(json.dumps({"ok": ok, "status": msg}).encode("utf-8"), "application/json")
        elif url.path == "/api/stop":
            msg = camera_manager.stop()
            self._send(json.dumps({"ok": True, "status": msg}).encode("utf-8"), "application/json")
        elif url.path == "/api/config":
            camera_manager.set_config(mode=body.get("mode"), undistort=body.get("undistort"))
            self._send(json.dumps({"ok": True}).encode("utf-8"), "application/json")
        elif url.path == "/api/gimbal":
            ok, msg = camera_manager.control_gimbal(body.get("action", ""))
            self._send(json.dumps({"ok": ok, "message": msg}).encode("utf-8"), "application/json")
        else:
            self.send_error(404)

    def _send(self, data: bytes, content_type: str, status: int = 200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_args):
        pass


def run_viewer(host: str = "127.0.0.1", port: int = 8080):
    server = ThreadingHTTPServer((host, port), RequestHandler)
    actual_port = server.server_port
    print(f"\n=======================================================")
    print(f"  Color & Shape Testing Viewer พร้อมใช้งานแล้ว!")
    print(f"  เปิดหน้าเว็บที่: http://{host}:{actual_port}/")
    print(f"  (กด Ctrl+C ในหน้าต่างนี้เพื่อปิดโปรแกรม)")
    print(f"=======================================================\n", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nกำลังปิดโปรแกรม...")
    finally:
        camera_manager.stop()
        server.server_close()
        print("ปิดเรียบร้อยแล้ว")


if __name__ == "__main__":
    run_viewer()
