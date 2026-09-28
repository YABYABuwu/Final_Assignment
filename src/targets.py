"""Color and shape detection adapted from Color_Shape_Viewer.py.

Contour area is a fraction of the current camera frame, so the threshold
continues to mean the same thing at 360p, 540p and 720p.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np


COLORS = {
    "red": (((0, 120, 70), (10, 255, 255)),
            ((170, 120, 70), (179, 255, 255))),
    "green": (((36, 80, 20), (95, 255, 255)),),
    "yellow": (((20, 100, 70), (35, 255, 255)),),
    "blue": (((96, 80, 40), (135, 255, 255)),),
}
SHAPES = ("circle", "square", "horizontal", "vertical")


@dataclass
class Detection:
    color: str
    shape: str
    area: float
    center: tuple
    contour: np.ndarray


def _shape(contour, min_area):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if area < min_area or perimeter <= 0:
        return None
    corners = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
    side_a, side_b = cv2.minAreaRect(contour)[1]
    if side_a <= 0 or side_b <= 0:
        return None
    ratio = min(side_a, side_b) / max(side_a, side_b)
    circularity = 4 * math.pi * area / (perimeter * perimeter)
    if circularity >= .70 and ratio >= .75 and len(corners) >= 7:
        return "circle"
    if len(corners) != 4 or not cv2.isContourConvex(corners):
        return None
    if area / (side_a * side_b) < .78:
        return None
    if ratio >= .85:
        return "square"
    box = cv2.boxPoints(cv2.minAreaRect(contour))
    edges = np.roll(box, -1, axis=0) - box
    long_edge = max(edges, key=lambda edge: np.dot(edge, edge))
    return "horizontal" if abs(long_edge[0]) >= abs(long_edge[1]) else "vertical"


def detect(frame, min_area_fraction=.02, selected=None):
    """Return accepted targets, largest first; selected is a set of pairs or None."""
    height, width = frame.shape[:2]
    min_area = height * width * min_area_fraction
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), dtype=np.uint8)
    found = []
    for color, ranges in COLORS.items():
        mask = np.zeros((height, width), dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            shape = _shape(contour, min_area)
            if shape is None or (selected is not None and (color, shape) not in selected):
                continue
            moments = cv2.moments(contour)
            if not moments["m00"]:
                continue
            center = (round(moments["m10"] / moments["m00"]),
                      round(moments["m01"] / moments["m00"]))
            found.append(Detection(color, shape, cv2.contourArea(contour),
                                   center, contour))
    return sorted(found, key=lambda item: item.area, reverse=True)


@dataclass
class Track:
    id: int
    color: str
    shape: str
    center: tuple
    area: float
    consecutive: int = 1
    misses: int = 0
    attempted: bool = False


class TargetTracker:
    """Match contours through adjacent frames and bounded gimbal movements."""

    def __init__(self, min_area_fraction, selected=None):
        self.min_area_fraction = min_area_fraction
        self.selected = selected
        self.tracks = {}
        self.next_id = 1

    def update(self, frame):
        height, width = frame.shape[:2]
        diagonal = math.hypot(width, height)
        detections = detect(frame, self.min_area_fraction, self.selected)
        pairs = []
        for track_id, track in self.tracks.items():
            for index, item in enumerate(detections):
                if (item.color, item.shape) != (track.color, track.shape):
                    continue
                ratio = item.area / track.area
                distance = math.hypot(item.center[0] - track.center[0],
                                      item.center[1] - track.center[1]) / diagonal
                if .4 <= ratio <= 2.5 and distance <= .18:
                    pairs.append((distance + .05 * abs(math.log(ratio)), track_id, index))
        pairs.sort()
        matched_tracks = set()
        matched_items = set()
        visible = {}
        for _, track_id, index in pairs:
            if track_id in matched_tracks or index in matched_items:
                continue
            matched_tracks.add(track_id)
            matched_items.add(index)
            item = detections[index]
            track = self.tracks[track_id]
            track.consecutive = track.consecutive + 1 if track.misses == 0 else 1
            track.center, track.area, track.misses = item.center, item.area, 0
            visible[track_id] = item
        for track_id, track in list(self.tracks.items()):
            if track_id not in matched_tracks:
                track.misses += 1
                track.consecutive = 0
                if track.misses > 8:
                    del self.tracks[track_id]
        for index, item in enumerate(detections):
            if index in matched_items:
                continue
            track_id = self.next_id
            self.next_id += 1
            self.tracks[track_id] = Track(track_id, item.color, item.shape,
                                          item.center, item.area)
            visible[track_id] = item
        return visible
