"""
Standalone (non-ROS) stop sign visualizer.

Runs the detection algorithm against a video file, an image directory, or a
webcam, with annotated output. Lets you tune detection without booting the
robot stack.

CLI:
    python -m stop_sign.visualizer \\
        --input ./test_video.mp4 \\
        --config ./src/stop_sign/config/stop_sign.yaml \\
        --output ./output_debug.mp4 \\
        [--display] [--no-overlay] [--focal-px 600]
"""
#python3 -m stop_sign.visualizer --input realsense --config config/stop_sign.yaml
import argparse
import glob
import json
import os
import sys
import time
from typing import Iterator, Optional, Tuple

import imageio
import numpy as np
import yaml

# Allow `python -m stop_sign.visualizer` from anywhere; also fall back when run
# as a script directly.
try:
    from stop_sign.algorithm.detection import detect
    from stop_sign.algorithm.temporal import (
        StopSignFilter, STATE_NONE, STATE_DETECTED, STATE_APPROACH,
    )
    from stop_sign.visualization import annotate_frame
except ImportError:
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(_HERE))
    from stop_sign.algorithm.detection import detect  # noqa: E402
    from stop_sign.algorithm.temporal import (  # noqa: E402
        StopSignFilter, STATE_NONE, STATE_DETECTED, STATE_APPROACH,
    )
    from stop_sign.visualization import annotate_frame  # noqa: E402


_STATE_NAMES = {STATE_NONE: 'NONE', STATE_DETECTED: 'DETECTED', STATE_APPROACH: 'APPROACH'}


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Standalone stop sign detector visualizer (non-ROS).',
    )
    p.add_argument('--input', required=True,
                   help='Video file (mp4/avi), directory of images, "webcam:N" for /dev/videoN, '
                        'or "realsense"/"realsense:WxH@FPS" to use the Intel RealSense '
                        'directly via pyrealsense2.')
    p.add_argument('--config', required=True,
                   help='Path to stop_sign.yaml.')
    p.add_argument('--output', default=None,
                   help='Path for annotated output video. If omitted, no output video is written.')
    p.add_argument('--display', dest='display', action='store_true', default=True,
                   help='Show live matplotlib preview window (default).')
    p.add_argument('--no-display', dest='display', action='store_false',
                   help='Disable the live preview window — useful for batch / headless runs.')
    p.add_argument('--no-overlay', action='store_true',
                   help='Skip annotation drawing (speed-only profiling). Still emits per-frame JSON to stdout.')
    p.add_argument('--focal-px', type=float, default=600.0,
                   help='Focal length in pixels for distance estimation (default 600, D435 at 640x480).')
    return p.parse_args(argv)


def _load_config(path: str) -> dict:
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)
    if isinstance(raw, dict) and 'stop_sign' in raw:
        return raw['stop_sign']
    return raw


def _iter_video(path: str) -> Iterator[Tuple[int, np.ndarray, float]]:
    reader = imageio.get_reader(path)
    try:
        meta = reader.get_meta_data()
        fps = float(meta.get('fps', 30.0)) or 30.0
    except Exception:
        fps = 30.0
    for i, frame in enumerate(reader):
        yield i, _to_rgb(frame), fps
    reader.close()


def _iter_image_dir(path: str) -> Iterator[Tuple[int, np.ndarray, float]]:
    patterns = ('*.jpg', '*.jpeg', '*.png', '*.bmp')
    files = []
    for pat in patterns:
        files.extend(glob.glob(os.path.join(path, pat)))
        files.extend(glob.glob(os.path.join(path, pat.upper())))
    files = sorted(set(files))
    if not files:
        raise FileNotFoundError(f"No images found in directory: {path}")
    for i, f in enumerate(files):
        yield i, _to_rgb(imageio.imread(f)), 10.0


def _iter_webcam(device_index: int) -> Iterator[Tuple[int, np.ndarray, float]]:
    reader = imageio.get_reader(f'<video{device_index}>')
    i = 0
    try:
        for frame in reader:
            yield i, _to_rgb(frame), 30.0
            i += 1
    finally:
        reader.close()


def _parse_realsense_spec(spec: str) -> Tuple[int, int, int]:
    """
    Parse 'realsense' or 'realsense:WxH@FPS' (e.g. 'realsense:1280x720@15').
    Returns (width, height, fps).
    """
    rest = spec.split(':', 1)[1] if ':' in spec else ''
    if not rest:
        return 640, 480, 30
    try:
        wh, fps_s = rest.split('@', 1) if '@' in rest else (rest, '30')
        w, h = wh.lower().split('x', 1)
        return int(w), int(h), int(fps_s)
    except Exception:
        raise ValueError(
            f"Bad realsense spec '{spec}'; expected 'realsense' or 'realsense:WxH@FPS'"
        )


def _open_realsense(width: int, height: int, fps: int):
    """
    Start a pyrealsense2 color pipeline. Returns (iterator, fps, focal_px) where
    focal_px comes from the camera's actual color intrinsics (fx).
    """
    try:
        import pyrealsense2 as rs
    except ImportError as e:
        raise RuntimeError(
            "pyrealsense2 not installed; install librealsense2/python-pyrealsense2 "
            "or use --input webcam:N as a fallback."
        ) from e

    # Hardware-reset to release stale USB handles, mirroring rs_stream_node.
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        raise RuntimeError("No RealSense device found on USB")
    for dev in devices:
        sys.stderr.write(
            f"[visualizer] hardware-resetting RealSense: "
            f"{dev.get_info(rs.camera_info.name)}\n"
        )
        dev.hardware_reset()
    time.sleep(3)  # wait for re-enumeration

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

    profile = None
    last_err = None
    for attempt in range(5):
        try:
            profile = pipe.start(cfg)
            break
        except RuntimeError as e:
            last_err = e
            sys.stderr.write(f"[visualizer] pipeline start failed (attempt {attempt + 1}/5): {e}\n")
            time.sleep(2)
    if profile is None:
        raise RuntimeError(f"Failed to start RealSense pipeline: {last_err}")

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    focal_px = float(intr.fx)
    sys.stderr.write(
        f"[visualizer] RealSense streaming {width}x{height}@{fps} "
        f"fx={intr.fx:.1f} fy={intr.fy:.1f}\n"
    )

    def _iter():
        i = 0
        try:
            while True:
                try:
                    frames = pipe.wait_for_frames(timeout_ms=2000)
                except RuntimeError:
                    continue  # transient timeout; keep streaming
                color = frames.get_color_frame()
                if not color:
                    continue
                rgb = np.asanyarray(color.get_data()).copy()
                yield i, rgb, float(fps)
                i += 1
        finally:
            try:
                pipe.stop()
            except Exception:
                pass

    return _iter(), float(fps), focal_px


def _to_rgb(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return np.stack([frame] * 3, axis=-1).astype(np.uint8)
    if frame.shape[2] == 4:
        return frame[..., :3].astype(np.uint8)
    return frame.astype(np.uint8)


def _open_input(spec: str):
    """
    Returns (iterator, fps, focal_px_override). focal_px_override is None unless
    the source provides authoritative intrinsics (e.g. RealSense).
    """
    if spec == 'realsense' or spec.startswith('realsense:') or spec.startswith('realsense@'):
        # Treat 'realsense@FPS' as 'realsense:WxH@FPS' shorthand by parsing through.
        if spec.startswith('realsense@'):
            spec = 'realsense:640x480@' + spec.split('@', 1)[1]
        w, h, fps = _parse_realsense_spec(spec)
        it, fps_out, focal_px = _open_realsense(w, h, fps)
        return it, fps_out, focal_px
    if spec.startswith('webcam:'):
        try:
            idx = int(spec.split(':', 1)[1])
        except ValueError:
            raise ValueError(f"Bad webcam spec '{spec}'; expected 'webcam:N'")
        return _iter_webcam(idx), 30.0, None
    if os.path.isdir(spec):
        return _iter_image_dir(spec), 10.0, None
    if os.path.isfile(spec):
        # Probe fps without consuming the iterator.
        try:
            r = imageio.get_reader(spec)
            fps = float(r.get_meta_data().get('fps', 30.0)) or 30.0
            r.close()
        except Exception:
            fps = 30.0
        return _iter_video(spec), fps, None
    raise FileNotFoundError(f"Input not found: {spec}")


def _try_open_display():
    """Try interactive matplotlib backends. Returns pyplot module or None."""
    if not os.environ.get('DISPLAY') and sys.platform.startswith('linux'):
        return None
    try:
        import matplotlib
    except ImportError:
        return None
    for backend in ('TkAgg', 'QtAgg', 'Qt5Agg', 'GTK3Agg'):
        try:
            matplotlib.use(backend, force=True)
            import matplotlib.pyplot as plt
            plt.ion()
            return plt
        except Exception:
            continue
    return None


def main(argv=None):
    args = _parse_args(argv)
    config = _load_config(args.config)

    iterator, fps, focal_px_override = _open_input(args.input)
    focal_px = focal_px_override if focal_px_override is not None else args.focal_px
    if focal_px_override is not None:
        sys.stderr.write(
            f"[visualizer] using camera intrinsics: focal_px={focal_px:.1f} "
            f"(--focal-px ignored)\n"
        )

    writer = None
    if args.output:
        writer = imageio.get_writer(args.output, fps=fps, codec='libx264', quality=7)

    plt = None
    img_handle = None
    fig = None
    if args.display:
        plt = _try_open_display()
        if plt is None:
            sys.stderr.write(
                "[visualizer] live display unavailable (no GUI backend); "
                "continuing headless. Pass --no-display to silence.\n"
            )
        else:
            fig, _ax = plt.subplots()
            try:
                fig.canvas.manager.set_window_title('stop_sign_visualizer')
            except Exception:
                pass
            _ax.set_axis_off()
            sys.stderr.write(
                "[visualizer] live preview is slow; pass --no-display "
                "for full-speed runs and watch the output video instead.\n"
            )

    stop_filter = StopSignFilter(config)
    n_frames = 0
    n_with_detection = 0
    t_start = time.time()
    last_t = t_start
    inst_fps = 0.0

    try:
        for frame_index, rgb, _ in iterator:
            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                inst_fps = 0.9 * inst_fps + 0.1 * (1.0 / dt) if inst_fps > 0 else (1.0 / dt)

            detection, debug_info = detect(rgb, config, focal_px, debug=True)
            filt_out = stop_filter.update(detection, now)

            if detection is not None:
                n_with_detection += 1

            # JSON line per frame.
            jrec = {
                'frame_index': frame_index,
                'state': _STATE_NAMES.get(filt_out.state, 'UNKNOWN'),
                'in_cooldown': filt_out.in_cooldown,
                'accepted_bbox': list(detection.bbox) if detection is not None else None,
                'distance_m': float(detection.distance_m) if detection is not None else None,
                'confidence': float(detection.confidence) if detection is not None else None,
                'n_candidates': len(debug_info.all_candidates) if debug_info is not None else 0,
            }
            sys.stdout.write(json.dumps(jrec) + '\n')
            sys.stdout.flush()

            if args.no_overlay:
                annotated = rgb
            else:
                annotated = annotate_frame(
                    rgb, debug_info, filt_out,
                    frame_index=frame_index, fps=inst_fps,
                )

            if writer is not None:
                writer.append_data(annotated)

            if plt is not None and fig is not None:
                ax = fig.axes[0]
                if img_handle is None:
                    img_handle = ax.imshow(annotated)
                else:
                    img_handle.set_data(annotated)
                fig.canvas.draw_idle()
                plt.pause(0.001)
                if not plt.fignum_exists(fig.number):
                    sys.stderr.write("[visualizer] preview window closed; exiting.\n")
                    break

            n_frames += 1
    except KeyboardInterrupt:
        sys.stderr.write("\n[visualizer] interrupted\n")
    finally:
        if writer is not None:
            writer.close()
        if plt is not None:
            plt.ioff()
            plt.close('all')

    elapsed = max(1e-6, time.time() - t_start)
    pct = (100.0 * n_with_detection / n_frames) if n_frames else 0.0
    mean_fps = n_frames / elapsed
    sys.stderr.write(
        f"[visualizer] frames={n_frames} detections={n_with_detection} "
        f"({pct:.1f}%) mean_fps={mean_fps:.1f}\n"
    )


if __name__ == '__main__':
    main()
