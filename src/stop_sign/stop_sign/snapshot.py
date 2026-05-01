"""
Grab one frame from the Intel RealSense, run stop-sign detection on it, and
save the annotated image to disk. Standalone — no ROS.

CLI:
    python3 -m stop_sign.snapshot \\
        --config ./src/stop_sign/config/stop_sign.yaml \\
        --output ./snapshot.png \\
        [--width 640 --height 480 --fps 30] \\
        [--warmup 15] [--raw raw.png]
"""
import argparse
import os
import sys
import time

import imageio
import numpy as np
import yaml

try:
    from stop_sign.algorithm.detection import detect
    from stop_sign.algorithm.temporal import StopSignFilter
    from stop_sign.visualization import annotate_frame
    from stop_sign.visualizer import _open_realsense
except ImportError:
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(_HERE))
    from stop_sign.algorithm.detection import detect  # noqa: E402
    from stop_sign.algorithm.temporal import StopSignFilter  # noqa: E402
    from stop_sign.visualization import annotate_frame  # noqa: E402
    from stop_sign.visualizer import _open_realsense  # noqa: E402


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Take one annotated stop-sign snapshot from the RealSense.',
    )
    p.add_argument('--config', required=True, help='Path to stop_sign.yaml.')
    p.add_argument('--output', required=True, help='Path to write the annotated PNG/JPG.')
    p.add_argument('--raw', default=None,
                   help='Optional path to also save the raw (un-annotated) frame.')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--warmup', type=int, default=15,
                   help='Discard this many frames before snapping, so auto-exposure settles.')
    return p.parse_args(argv)


def _load_config(path: str) -> dict:
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)
    if isinstance(raw, dict) and 'stop_sign' in raw:
        return raw['stop_sign']
    return raw


def main(argv=None):
    args = _parse_args(argv)
    config = _load_config(args.config)

    iterator, _, focal_px = _open_realsense(args.width, args.height, args.fps)
    sys.stderr.write(f"[snapshot] focal_px={focal_px:.1f}; warming up {args.warmup} frames…\n")

    rgb = None
    try:
        for i, frame, _ in iterator:
            rgb = frame
            if i + 1 >= max(args.warmup, 1):
                break
        if rgb is None:
            raise RuntimeError("Did not receive any frames from RealSense.")
    finally:
        # Iterator's finally clause stops the pipeline on close.
        try:
            iterator.close()
        except Exception:
            pass

    detection, debug_info = detect(rgb, config, focal_px, debug=True)
    stop_filter = StopSignFilter(config)
    filt_out = stop_filter.update(detection, time.time())

    annotated = annotate_frame(
        rgb, debug_info, filt_out,
        frame_index=0, fps=float(args.fps),
    )

    if args.raw:
        imageio.imwrite(args.raw, rgb)
        sys.stderr.write(f"[snapshot] wrote raw frame: {args.raw}\n")

    imageio.imwrite(args.output, annotated)

    if detection is not None:
        sys.stderr.write(
            f"[snapshot] DETECTED bbox={detection.bbox} "
            f"distance={detection.distance_m:.2f}m conf={detection.confidence:.2f} "
            f"-> {args.output}\n"
        )
    else:
        sys.stderr.write(
            f"[snapshot] no stop sign detected "
            f"(candidates={len(debug_info.all_candidates) if debug_info else 0}) "
            f"-> {args.output}\n"
        )


if __name__ == '__main__':
    main()
