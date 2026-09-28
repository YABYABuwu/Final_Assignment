"""Helpers for confirming, firing at, and saving detected targets for Round 2."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile


TARGET_FILE_FORMAT = "robomaster-round2-targets"
TARGET_FILE_VERSION = 1


def parse_cell(value):
    """Parse a grid cell written as 'x,y'."""
    try:
        parts = [int(part.strip()) for part in value.split(",")]
    except (AttributeError, TypeError, ValueError):
        raise ValueError("target cell must use the form x,y, for example 3,2")
    if len(parts) != 2:
        raise ValueError("target cell must contain exactly two integers")
    return tuple(parts)


def select_confirmed_detection(status, color=None, shape=None,
                               max_center_offset=0.22, max_age_ms=500):
    """Choose the largest fresh, confirmed detection close to the crosshair."""
    if not isinstance(status, dict) or not status.get("enabled"):
        return None, "target detector is disabled"
    if status.get("error"):
        return None, f"target detector error: {status['error']}"
    age_ms = status.get("age_ms")
    if not isinstance(age_ms, (int, float)) or age_ms > max_age_ms:
        return None, "target detection result is missing or stale"

    candidates = []
    for detection in status.get("detections", []):
        if not detection.get("confirmed"):
            continue
        if color is not None and detection.get("color") != color:
            continue
        if shape is not None and detection.get("shape") != shape:
            continue
        offset = detection.get("center_offset_norm")
        if (not isinstance(offset, (list, tuple)) or len(offset) != 2 or
                not all(isinstance(value, (int, float)) for value in offset)):
            continue
        center_distance = math.hypot(offset[0], offset[1])
        if center_distance > max_center_offset:
            continue
        candidates.append((center_distance, -float(detection.get("area_px2", 0)), detection))

    if not candidates:
        return None, "no confirmed target matching the filters is near the crosshair"
    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2], None


def fire_infrared(blaster_module, blaster_device, times=1):
    """Fire only the SDK infrared mode; never rely on the water-fire default."""
    if type(times) is not int or not 1 <= times <= 5:
        raise ValueError("infrared fire times must be an integer from 1 to 5")
    return blaster_device.fire(fire_type=blaster_module.INFRARED_FIRE, times=times)


def build_target_record(target_id, target_cell, detection, pose, gimbal,
                        side=None, fired=False, fire_times=0):
    """Build a Round 2 compatible target with diagnostic observation metadata."""
    record = {
        "id": target_id,
        "pos": [int(target_cell[0]), int(target_cell[1])],
        "color": detection["color"],
        "shape": detection["shape"],
        "type": "target",
        "should_shoot": True,
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "observation": {
            "pose": [round(float(value), 5) for value in pose],
            "gimbal_yaw_deg": round(float(gimbal[1]), 3),
            "gimbal_pitch_deg": round(float(gimbal[0]), 3),
            "center_px": list(detection["center_px"]),
            "center_offset_norm": list(detection["center_offset_norm"]),
            "area_px2": float(detection["area_px2"]),
            "stability_hits": int(detection["stability_hits"]),
            "stability_required": int(detection["stability_required"]),
        },
        "infrared_test": {
            "fired": bool(fired),
            "times": int(fire_times) if fired else 0,
        },
    }
    if side is not None:
        record["side"] = side
    return record


def init_target_document(path, backup=True):
    """Reset target document to an empty targets list, optionally archiving previous file."""
    path = Path(path)
    if path.exists() and backup:
        try:
            document = load_target_document(path)
            if document.get("targets"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_dir = path.parent / "targets_archive"
                backup_dir.mkdir(parents=True, exist_ok=True)
                backup_file = backup_dir / f"targets_{timestamp}.json"
                import shutil
                shutil.copy2(path, backup_file)
                print(f"[targets] Archived previous {len(document['targets'])} targets to {backup_file.name}")
        except Exception as e:
            print(f"[targets] Warning: failed to backup {path}: {e}")

    empty_document = {
        "format": TARGET_FILE_FORMAT,
        "version": TARGET_FILE_VERSION,
        "targets": [],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
                "w", encoding="utf-8", dir=str(path.parent),
                prefix=path.name + ".", suffix=".tmp", delete=False) as temp_file:
            json.dump(empty_document, temp_file, ensure_ascii=False, indent=2)
            temp_file.write("\n")
            temp_name = temp_file.name
        Path(temp_name).replace(path)
    finally:
        if temp_name is not None:
            temp_path = Path(temp_name)
            if temp_path.exists():
                temp_path.unlink()
    return empty_document


def load_target_document(path):
    """Load a target file while accepting the legacy top-level list."""
    path = Path(path)
    if not path.exists():
        return {
            "format": TARGET_FILE_FORMAT,
            "version": TARGET_FILE_VERSION,
            "targets": [],
        }
    with path.open(encoding="utf-8") as file:
        document = json.load(file)
    if isinstance(document, list):
        document = {
            "format": TARGET_FILE_FORMAT,
            "version": TARGET_FILE_VERSION,
            "targets": document,
        }
    if not isinstance(document, dict) or not isinstance(document.get("targets"), list):
        raise ValueError("target file must be a list or an object containing a targets list")
    return document


def next_target_id(targets):
    """Return the first unused T-number."""
    used = {item.get("id") for item in targets if isinstance(item, dict)}
    index = 1
    while f"T{index}" in used:
        index += 1
    return f"T{index}"


def save_target_record(path, record, replace=False):
    """Atomically append a target, or replace the same ID when explicitly requested."""
    path = Path(path)
    document = load_target_document(path)
    targets = document["targets"]
    matches = [index for index, item in enumerate(targets)
               if isinstance(item, dict) and item.get("id") == record.get("id")]
    if matches and not replace:
        raise ValueError(f"target id {record.get('id')} already exists; use --replace to update it")
    if matches:
        targets[matches[0]] = record
    else:
        targets.append(record)
    document["format"] = TARGET_FILE_FORMAT
    document["version"] = TARGET_FILE_VERSION

    path.parent.mkdir(parents=True, exist_ok=True)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
                "w", encoding="utf-8", dir=str(path.parent),
                prefix=path.name + ".", suffix=".tmp", delete=False) as temp_file:
            json.dump(document, temp_file, ensure_ascii=False, indent=2)
            temp_file.write("\n")
            temp_name = temp_file.name
        Path(temp_name).replace(path)
    finally:
        if temp_name is not None:
            temp_path = Path(temp_name)
            if temp_path.exists():
                temp_path.unlink()
    return document
