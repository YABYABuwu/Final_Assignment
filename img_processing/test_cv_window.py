"""Standalone OpenCV desktop viewer for color & shape detection with manual gimbal testing.

Usage:
  # Test with default webcam:
  python test_cv_window.py --source webcam0

  # Test with RoboMaster EP (AP mode):
  py -3.8 test_cv_window.py --source ep_ap

  # Test with RoboMaster EP (RNDIS USB):
  py -3.8 test_cv_window.py --source ep_rndis

Keyboard Shortcuts:
  'q' / ESC : ออกจากโปรแกรม
  'm'       : สลับโหมด Robust <-> Classic
  'u'       : สลับเปิด/ปิด แก้ความบิดเบี้ยวเลนส์ (Undistort)
  'c'       : เปลี่ยนสี Mask (All -> Red -> Green -> Yellow -> Blue)
  'g'       : ปลด/ล็อกกิมบอล (Gimbal Suspend/Resume) เพื่อใช้มือจับหมุน
  's'       : บันทึกภาพ Snapshot ไว้ในโฟลเดอร์ snapshots/
"""

import argparse
from datetime import datetime
from pathlib import Path
from queue import Empty
import sys
import time

import cv2
import numpy as np

# Support running directly or as a module
_pkg_dir = Path(__file__).resolve().parent
_root_dir = _pkg_dir.parent
for _p in (str(_pkg_dir), str(_root_dir)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from .config import COLORS, SHAPES, DEFAULT_DETECTION_CONFIG
    from .detector import ColorShapeDetector
except (ImportError, ValueError):
    from config import COLORS, SHAPES, DEFAULT_DETECTION_CONFIG
    from detector import ColorShapeDetector


def main():
    parser = argparse.ArgumentParser(description="OpenCV Color and Shape Diagnostic Window")
    parser.add_argument("--source", default="webcam0", choices=["webcam0", "webcam1", "ep_ap", "ep_rndis"],
                        help="Video source: webcam0, webcam1, ep_ap, ep_rndis")
    parser.add_argument("--mode", default="robust", choices=["robust", "classic"],
                        help="Shape classification algorithm")
    parser.add_argument("--undistort", action="store_true", default=True,
                        help="Enable camera lens undistortion")
    args = parser.parse_args()

    detector = ColorShapeDetector(mode=args.mode, enable_undistort=args.undistort)
    camera = None
    ep_robot = None
    stream_started = False
    gimbal_suspended = False
    active_color_idx = 0
    color_keys = ["all"] + list(COLORS.keys())

    snapshot_dir = Path(__file__).resolve().parent / "snapshots"
    snapshot_dir.mkdir(exist_ok=True)

    print(f"\n=======================================================")
    print(f"  OpenCV Diagnostic Window Starting...")
    print(f"  Source: {args.source} | Mode: {args.mode} | Undistort: {args.undistort}")
    print(f"  คีย์ลัด:")
    print(f"    [m] : สลับโหมด Robust / Classic")
    print(f"    [u] : สลับเปิด/ปิด Undistortion")
    print(f"    [c] : เปลี่ยนสี Mask ที่แสดง")
    print(f"    [g] : ปลดมอเตอร์กิมบอลให้จับหมุนด้วยมือได้ (เฉพาะต่อหุ่นยนต์)")
    print(f"    [s] : บันทึกภาพ Snapshot")
    print(f"    [q] / ESC : ออกจากโปรแกรม")
    print(f"=======================================================\n")

    try:
        if args.source.startswith("ep_"):
            from robomaster import robot, conn
            conn_type = conn.CONNECTION_USB_RNDIS if args.source == "ep_rndis" else conn.CONNECTION_WIFI_AP
            ep_robot = robot.Robot()
            ep_robot.initialize(conn_type=conn_type)
            # Release gimbal so user can freely rotate it by hand
            ep_robot.gimbal.suspend()
            gimbal_suspended = True
            print(">> กิมบอลอยู่ในสถานะ Suspend (สามารถใช้มือจับหมุนได้อย่างอิสระ)")
            camera = ep_robot.camera
            stream_started = camera.start_video_stream(display=False)
            if not stream_started:
                raise RuntimeError("ไม่สามารถเริ่มวิดีโอสตรีมจาก RoboMaster EP ได้")
        else:
            cam_idx = int(args.source[-1])
            camera = cv2.VideoCapture(cam_idx)
            if not camera.isOpened():
                raise RuntimeError(f"ไม่สามารถเปิด Webcam {cam_idx} ได้")

        window_name = "Target Detection [Manual Gimbal]"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 500)

        fps_times = []
        while True:
            t0 = time.time()
            if args.source.startswith("ep_"):
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

            # Detect targets
            detections, masks, proc_frame = detector.detect(frame, debug=True)

            # Draw annotations
            annotated = proc_frame.copy()
            for d in detections:
                if d.shape == "unknown":
                    continue
                bgr = COLORS[d.color]["bgr"]
                cv2.drawContours(annotated, [d.contour], -1, bgr, 2)
                cv2.circle(annotated, d.center, 5, bgr, -1)
                x, y, w, h = cv2.boundingRect(d.contour)
                lbl = f"{d.color} {d.shape} ({int(d.area)}px)"
                cv2.putText(annotated, lbl, (x, max(22, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)
                m = d.metrics
                sub_lbl = f"R={m.get('ratio', 0):.2f}"
                if "ellipse_fit_ratio" in m:
                    sub_lbl += f" E={m['ellipse_fit_ratio']:.2f}"
                cv2.putText(annotated, sub_lbl, (x, y + h + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            # Mask to show
            cur_color = color_keys[active_color_idx]
            if cur_color == "all":
                combined = np.zeros_like(next(iter(masks.values())))
                for m in masks.values():
                    combined |= m
                mask_display = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
            else:
                mask_display = cv2.cvtColor(masks[cur_color], cv2.COLOR_GRAY2BGR)

            # Overlay status text
            t1 = time.time()
            fps_times.append(t1 - t0)
            if len(fps_times) > 15:
                fps_times.pop(0)
            fps = len(fps_times) / sum(fps_times) if sum(fps_times) > 0 else 0

            info_text = f"Mode: {detector.mode.upper()} | Undistort: {'ON' if detector.enable_undistort else 'OFF'} | Mask: {cur_color} | {fps:.1f} FPS"
            if ep_robot:
                info_text += f" | Gimbal: {'FREE (Hand-turn)' if gimbal_suspended else 'LOCKED'}"
            cv2.putText(annotated, info_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            # Concatenate left (annotated) + right (mask)
            h, w = annotated.shape[:2]
            mask_resized = cv2.resize(mask_display, (w, h))
            display_img = cv2.hconcat([annotated, mask_resized])

            cv2.imshow(window_name, display_img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('m'):
                detector.mode = "classic" if detector.mode == "robust" else "robust"
                print(f">> เปลี่ยนโหมดเป็น: {detector.mode.upper()}")
            elif key == ord('u'):
                detector.enable_undistort = not detector.enable_undistort
                print(f">> แก้เลนส์โค้ง (Undistort): {'เปิด' if detector.enable_undistort else 'ปิด'}")
            elif key == ord('c'):
                active_color_idx = (active_color_idx + 1) % len(color_keys)
                print(f">> สลับ Mask สี: {color_keys[active_color_idx]}")
            elif key == ord('g') and ep_robot:
                if gimbal_suspended:
                    ep_robot.gimbal.resume()
                    gimbal_suspended = False
                    print(">> ล็อกมอเตอร์กิมบอลแล้ว")
                else:
                    ep_robot.gimbal.suspend()
                    gimbal_suspended = True
                    print(">> ปลดมอเตอร์กิมบอลแล้ว (สามารถจับหมุนด้วยมือได้)")
            elif key == ord('s'):
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = snapshot_dir / f"snapshot_{stamp}.png"
                cv2.imwrite(str(save_path), display_img)
                print(f">> บันทึกภาพ Snapshot ที่: {save_path}")

    except Exception as e:
        print(f"\nเกิดข้อผิดพลาด: {e}")
    finally:
        cv2.destroyAllWindows()
        if camera is not None:
            try:
                if stream_started:
                    camera.stop_video_stream()
                else:
                    camera.release()
            except Exception:
                pass
        if ep_robot is not None:
            try:
                ep_robot.close()
            except Exception:
                pass
        print("ปิดโปรแกรมเรียบร้อย")


if __name__ == "__main__":
    main()
