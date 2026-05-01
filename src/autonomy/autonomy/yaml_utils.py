#!/usr/bin/env python3

from __future__ import annotations

import os
import math
from typing import Any
from urllib.parse import unquote, urlparse

import yaml


def _as_float(value: Any, label: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric (got {value!r})") from exc


def _coerce_pose_entry(entry: Any, idx: int) -> dict[str, float]:
    if not isinstance(entry, dict):
        raise ValueError(f"poses[{idx}] must be a mapping")

    if "x" in entry and "y" in entry:
        x = _as_float(entry["x"], f"poses[{idx}].x")
        y = _as_float(entry["y"], f"poses[{idx}].y")
        qz = _as_float(entry.get("qz", 0.0), f"poses[{idx}].qz")
        qw = _as_float(entry.get("qw", 1.0), f"poses[{idx}].qw")
        return {"x": x, "y": y, "qz": qz, "qw": qw}

    # Also accept nav_msgs/PoseStamped-shaped YAML dumps.
    pose = entry.get("pose")
    if isinstance(pose, dict):
        pos = pose.get("position")
        ori = pose.get("orientation")
        if isinstance(pos, dict) and isinstance(ori, dict):
            x = _as_float(pos.get("x"), f"poses[{idx}].pose.position.x")
            y = _as_float(pos.get("y"), f"poses[{idx}].pose.position.y")
            qz = _as_float(ori.get("z", 0.0), f"poses[{idx}].pose.orientation.z")
            qw = _as_float(ori.get("w", 1.0), f"poses[{idx}].pose.orientation.w")
            return {"x": x, "y": y, "qz": qz, "qw": qw}

    raise ValueError(
        f"poses[{idx}] missing expected fields (need x/y or pose.position + pose.orientation)"
    )


def load_recorded_poses(path: str) -> list[dict[str, float]]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("path YAML root must be a mapping with 'poses'")
    poses_raw = data.get("poses")
    if not isinstance(poses_raw, list):
        raise ValueError("path YAML missing 'poses' list")
    if not poses_raw:
        raise ValueError("path YAML contains an empty 'poses' list")

    return [_coerce_pose_entry(entry, idx) for idx, entry in enumerate(poses_raw)]


def dump_recorded_poses(path: str, poses: list[dict[str, float]]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"poses": poses}, f, sort_keys=False)


def _validate_map_yaml(data: dict[str, Any]) -> None:
    if "image" in data and not isinstance(data.get("image"), str):
        raise ValueError("map YAML 'image' must be a string when present")
    if "resolution" in data:
        resolution = _as_float(data["resolution"], "map YAML resolution")
        if resolution <= 0.0:
            raise ValueError("map YAML resolution must be > 0")
    if "origin" in data:
        origin = data["origin"]
        if not isinstance(origin, (list, tuple)) or len(origin) < 2:
            raise ValueError("map YAML origin must be a list like [x, y, yaw]")
        _as_float(origin[0], "map YAML origin[0]")
        _as_float(origin[1], "map YAML origin[1]")


def load_map_yaml(path: str) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("map YAML root must be a mapping")
    _validate_map_yaml(data)
    return data


def resolve_map_image_path(yaml_path: str, map_base: str | None = None) -> str:
    data = load_map_yaml(yaml_path)
    image = data.get("image")
    if not image:
        if map_base:
            return os.path.normpath(f"{map_base}.pgm")
        raise ValueError("map YAML missing required 'image' field")

    image = os.path.expandvars(os.path.expanduser(image.strip()))
    parsed = urlparse(image)
    if parsed.scheme == "file":
        return os.path.normpath(unquote(parsed.path))
    if os.path.isabs(image):
        return os.path.normpath(image)
    return os.path.normpath(os.path.join(os.path.dirname(yaml_path), image))


def interpolate_and_close_poses(
    poses: list[dict[str, float]],
    min_dist: float = 0.15,
    max_bridge: float = 1.0,
) -> tuple[list[dict[str, float]], int, bool]:
    """Fill short waypoint gaps and close loop, mirroring interpolate_path.py behavior."""
    if len(poses) < 2:
        return poses, 0, False

    filled: list[dict[str, float]] = [poses[0]]
    total_added = 0
    for i in range(1, len(poses)):
        prev = poses[i - 1]
        curr = poses[i]
        gap = math.hypot(curr["x"] - prev["x"], curr["y"] - prev["y"])
        if min_dist < gap <= max_bridge:
            n_steps = max(1, int(gap / min_dist))
            for j in range(1, n_steps):
                t = j / n_steps
                filled.append(
                    {
                        "x": float(prev["x"] + t * (curr["x"] - prev["x"])),
                        "y": float(prev["y"] + t * (curr["y"] - prev["y"])),
                        "qz": float(prev.get("qz", 0.0) + t * (curr.get("qz", 0.0) - prev.get("qz", 0.0))),
                        "qw": float(prev.get("qw", 1.0) + t * (curr.get("qw", 1.0) - prev.get("qw", 1.0))),
                    }
                )
            total_added += n_steps - 1
        filled.append(curr)

    first = filled[0]
    last = filled[-1]
    gap = math.hypot(first["x"] - last["x"], first["y"] - last["y"])
    if gap < min_dist or gap > max_bridge:
        return filled, total_added, False

    n_steps = max(1, int(gap / min_dist))
    closing: list[dict[str, float]] = []
    for i in range(1, n_steps):
        t = i / n_steps
        closing.append(
            {
                "x": float(last["x"] + t * (first["x"] - last["x"])),
                "y": float(last["y"] + t * (first["y"] - last["y"])),
                "qz": float(last.get("qz", 0.0)),
                "qw": float(last.get("qw", 1.0)),
            }
        )
    return filled + closing, total_added + len(closing), True
