"""Local IR lane geometry shared by exploration and Round 2 navigation."""

import math

from src.mission_stop import MissionStop


def prepare_lane(source, destination, targets, yaw_deg, saved=None):
    edge = tuple(sorted((tuple(source), tuple(destination))))
    du, dv = edge[1][0] - edge[0][0], edge[1][1] - edge[0][1]
    if abs(du) + abs(dv) != 1:
        raise ValueError("IR lanes require adjacent cells")
    angle = math.radians(yaw_deg)
    normal = (-du * math.sin(angle) - dv * math.cos(angle),
              du * math.cos(angle) - dv * math.sin(angle))
    index = edge.index(tuple(destination))
    reference = saved["anchors_m"][index] if saved else targets[tuple(destination)]
    # Preserve this visit's longitudinal waypoint, including shooting standoff.
    # Both endpoints share one lateral coordinate for forward/reverse travel.
    anchors = []
    for node in edge:
        point = targets[node]
        cross = sum((reference[i] - point[i]) * normal[i] for i in (0, 1))
        anchors.append([point[i] + cross * normal[i] for i in (0, 1)])
    lane = {"cells": [list(node) for node in edge], "anchors_m": anchors,
            "offset_m": list(saved["offset_m"]) if saved else [0.0, 0.0]}
    target = tuple(anchors[index][i] + lane["offset_m"][i] for i in (0, 1))
    return edge, lane, normal, index, target


def retarget_lane(lane, normal, index, before, after, settings, contains_world):
    lateral = sum((after[i] - before[i]) * normal[i] for i in (0, 1))
    if abs(lateral) < settings["min_shift_m"]:
        return None
    baseline = lane["anchors_m"][index]
    offset = sum((after[i] - baseline[i]) * normal[i] for i in (0, 1))
    if not math.isfinite(offset):
        raise MissionStop("IR lane correction exceeds the local offset or map limit: lateral offset is invalid")
    if abs(offset) > settings["max_offset_m"]:
        raise MissionStop(
            "IR lane correction exceeds the local offset or map limit: "
            "needs {:.3f} m lateral offset; allowed {:.3f} m on edge {}".format(
                abs(offset), settings["max_offset_m"], lane["cells"]))
    if not all(contains_world(anchor[0] + offset * normal[0],
                              anchor[1] + offset * normal[1])
               for anchor in lane["anchors_m"]):
        raise MissionStop(
            "IR lane correction exceeds the local offset or map limit: "
            "shifted edge {} would leave the permitted map area".format(lane["cells"]))
    updated = {**lane, "offset_m": [offset * value for value in normal]}
    target = tuple(baseline[i] + updated["offset_m"][i] for i in (0, 1))
    return updated, target
