"""Camera calibration helper script for RoboMaster EP wide-angle lens.

Computes exact intrinsic matrix K and distortion coefficients D using a chessboard.

Usage:
  # Capture 10-15 chessboard images with camera:
  py -3.8 calibrate_camera.py --capture --source ep_ap

  # Or run calibration on saved calibration images:
  python calibrate_camera.py --images "calib_images/*.jpg" --pattern 9x6 --square-size 25
"""

import argparse
import glob
from pathlib import Path
import time

import cv2
import numpy as np


def calibrate(images, pattern_size=(9, 6), square_size_mm=25.0):
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size_mm

    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    img_shape = None

    print(f"กำลังประมวลผล {len(images)} ภาพเพื่อหาตารางหมากรุก...")
    found_count = 0
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners2)
            found_count += 1
            print(f"  [OK] ตรวจพบหมากรุกใน {fname}")
        else:
            print(f"  [SKIP] ไม่พบหมากรุกใน {fname}")

    if found_count < 5:
        print(f"ข้อผิดพลาด: ตรวจพบหมากรุกเพียง {found_count} ภาพ (ต้องการอย่างน้อย 5 ภาพขึ้นไป)")
        return None, None

    print("กำลังคำนวณ Camera Matrix และ Distortion Coefficients...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    print("\n=== ผลการ Calibrate กล้อง ===")
    print("Camera Matrix K:")
    print(mtx)
    print("\nDistortion Coefficients D (k1, k2, p1, p2, k3):")
    print(dist.ravel())
    print(f"Reprojection Error (RMS): {ret:.4f} px\n")
    return mtx, dist


def main():
    parser = argparse.ArgumentParser(description="Camera Calibration Tool")
    parser.add_argument("--images", default="calib_images/*.jpg", help="Path glob for chessboard images")
    parser.add_argument("--pattern", default="9x6", help="Number of inner corners, e.g. '9x6'")
    parser.add_argument("--square-size", type=float, default=25.0, help="Square side length in mm")
    args = parser.parse_args()

    parts = [int(p) for p in args.pattern.split("x")]
    pattern_size = (parts[0], parts[1])
    images = glob.glob(args.images)
    if not images:
        print(f"ไม่พบไฟล์ภาพใน: {args.images}")
        print("คำแนะนำ: บันทึกภาพแผ่นตารางหมากรุกจากกล้องหุ่นยนต์ประมาณ 10-15 ภาพในมุมต่างๆ แล้วนำมา Calibrate")
        return

    calibrate(images, pattern_size, args.square_size)


if __name__ == "__main__":
    main()
