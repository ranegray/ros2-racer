#!/usr/bin/env python3
"""
interpolate_path.py

Post-processes an existing recorded_path.yaml to fill gaps and close the loop.
Run this after Lap 1 if the saved path has missing segments.

Usage:
    python3 interpolate_path.py [path_to_yaml]

Default path: ~/.ros/recorded_path.yaml
The original file is backed up as <name>_pre_interp.yaml before overwriting.
"""

import math
import os
import shutil
import sys

try:
    from autonomy.yaml_utils import dump_recorded_poses, load_recorded_poses
except ImportError:
    from yaml_utils import dump_recorded_poses, load_recorded_poses

MIN_DIST   = 0.15   # m — minimum spacing between waypoints
MAX_BRIDGE = 1.0    # m — gaps larger than this are left alone (genuine discontinuity)


def interpolate_gaps(poses: list[dict]) -> list[dict]:
    if len(poses) < 2:
        return poses
    filled: list[dict] = [poses[0]]
    total_added = 0
    for i in range(1, len(poses)):
        prev = poses[i - 1]
        curr = poses[i]
        gap = math.hypot(curr['x'] - prev['x'], curr['y'] - prev['y'])
        if MIN_DIST < gap <= MAX_BRIDGE:
            n_steps = max(1, int(gap / MIN_DIST))
            for j in range(1, n_steps):
                t = j / n_steps
                filled.append({
                    'x':  float(prev['x']  + t * (curr['x']  - prev['x'])),
                    'y':  float(prev['y']  + t * (curr['y']  - prev['y'])),
                    'qz': float(prev['qz'] + t * (curr['qz'] - prev['qz'])),
                    'qw': float(prev['qw'] + t * (curr['qw'] - prev['qw'])),
                })
            total_added += n_steps - 1
        filled.append(curr)
    print(f"  Gap interpolation: added {total_added} waypoints ({len(poses)} → {len(filled)})")
    return filled


def close_loop(poses: list[dict]) -> list[dict]:
    if len(poses) < 2:
        return poses
    last  = poses[-1]
    first = poses[0]
    gap = math.hypot(first['x'] - last['x'], first['y'] - last['y'])
    if gap < MIN_DIST:
        print(f"  Loop already closed (gap={gap:.3f}m < MIN_DIST)")
        return poses
    if gap > MAX_BRIDGE:
        print(f"  Loop gap too large to bridge ({gap:.2f}m > {MAX_BRIDGE}m) — skipping")
        return poses
    n_steps = max(1, int(gap / MIN_DIST))
    closing = []
    for i in range(1, n_steps):
        t = i / n_steps
        closing.append({
            'x':  float(last['x']  + t * (first['x']  - last['x'])),
            'y':  float(last['y']  + t * (first['y']  - last['y'])),
            'qz': float(last['qz']),
            'qw': float(last['qw']),
        })
    print(f"  Loop closure: bridged {gap:.2f}m gap with {len(closing)} waypoints")
    return poses + closing


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/.ros/recorded_path.yaml")
    path = os.path.realpath(path)  # resolve symlink so we edit the archive, not the link

    if not os.path.exists(path):
        print(f"ERROR: file not found: {path}")
        sys.exit(1)

    print(f"Reading: {path}")
    try:
        poses = load_recorded_poses(path)
    except (OSError, ValueError) as e:
        print(f"ERROR: invalid path yaml: {e}")
        sys.exit(1)

    print(f"Loaded {len(poses)} poses")

    poses = interpolate_gaps(poses)
    poses = close_loop(poses)

    backup = path.replace('.yaml', '_pre_interp.yaml')
    shutil.copy2(path, backup)
    print(f"Backup saved to: {backup}")

    dump_recorded_poses(path, poses)

    print(f"Wrote {len(poses)} poses to: {path}")


if __name__ == '__main__':
    main()
