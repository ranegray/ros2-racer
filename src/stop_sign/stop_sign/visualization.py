"""
Drawing helpers for stop sign visualization.

Shared between the standalone CLI visualizer (visualizer.py) and the ROS node's
/stop_sign/debug_image publisher. PIL-based — no cv2.
"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from stop_sign.algorithm.temporal import STATE_NONE, STATE_DETECTED, STATE_APPROACH


COLOR_ACCEPTED = (0, 255, 0)         # green
COLOR_REJECT_FILL = (255, 220, 0)    # yellow
COLOR_REJECT_ASPECT = (255, 140, 0)  # orange
COLOR_REJECT_OTHER = (255, 255, 255)  # white
COLOR_HUD = (255, 255, 255)
COLOR_HUD_BG = (0, 0, 0)
COLOR_MASK = np.array([255, 0, 0], dtype=np.uint8)


_STATE_LABELS = {STATE_NONE: 'NONE', STATE_DETECTED: 'DETECTED', STATE_APPROACH: 'APPROACH'}


def _font():
    try:
        return ImageFont.load_default()
    except Exception:
        return None


def overlay_mask(rgb: np.ndarray, mask: np.ndarray, alpha: float = 0.4) -> np.ndarray:
    """Alpha-blend a red overlay onto rgb wherever mask is True."""
    if mask is None:
        return rgb.copy()
    overlay = rgb.copy()
    overlay[mask] = COLOR_MASK
    blended = (rgb.astype(np.float32) * (1.0 - alpha) +
               overlay.astype(np.float32) * alpha).astype(np.uint8)
    return blended


def _bbox_color(reason: str) -> tuple:
    if reason == 'accepted':
        return COLOR_ACCEPTED
    if reason == 'fill':
        return COLOR_REJECT_FILL
    if reason == 'aspect':
        return COLOR_REJECT_ASPECT
    return COLOR_REJECT_OTHER


def annotate_frame(rgb: np.ndarray,
                   debug_info,
                   filter_output,
                   frame_index: int = 0,
                   fps: float = 0.0) -> np.ndarray:
    """
    Render an annotated copy of `rgb` showing mask overlay, candidate bboxes,
    accepted detection labels, and a HUD.

    Args:
      rgb: HxWx3 uint8 RGB frame.
      debug_info: DetectionDebugInfo from detect(..., debug=True), or None.
      filter_output: FilterOutput from StopSignFilter.update(), or None.
      frame_index, fps: metadata for HUD.

    Returns: HxWx3 uint8 RGB annotated frame.
    """
    if debug_info is not None and debug_info.mask is not None:
        canvas = overlay_mask(rgb, debug_info.mask)
    else:
        canvas = rgb.copy()

    img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(img)
    font = _font()

    n_candidates = 0
    n_accepted = 0
    if debug_info is not None:
        for cand in debug_info.all_candidates:
            n_candidates += 1
            x, y, w, h = cand['bbox']
            color = _bbox_color(cand['reason'])
            draw.rectangle([x, y, x + w, y + h], outline=color, width=2)
            if cand['reason'] == 'accepted':
                n_accepted += 1

    accepted = debug_info.accepted if debug_info is not None else None
    if accepted is not None:
        x, y, w, h = accepted.bbox
        label = f"{accepted.distance_m:.2f} m | conf={accepted.confidence:.2f}"
        text_y = max(0, y - 14)
        draw.rectangle([x, text_y, x + 8 * len(label), text_y + 12], fill=COLOR_HUD_BG)
        draw.text((x + 2, text_y), label, fill=COLOR_HUD, font=font)

    # HUD
    state_label = 'NONE'
    cooldown_label = ''
    if filter_output is not None:
        state_label = _STATE_LABELS.get(filter_output.state, '?')
        if filter_output.in_cooldown:
            cooldown_label = f" | cooldown {filter_output.cooldown_remaining_sec:.1f}s"
    hud_lines = [
        f"frame {frame_index}  fps {fps:.1f}",
        f"state {state_label}{cooldown_label}",
        f"candidates {n_candidates}  accepted {n_accepted}",
    ]
    pad = 4
    line_h = 12
    box_w = 230
    box_h = pad * 2 + line_h * len(hud_lines)
    draw.rectangle([0, 0, box_w, box_h], fill=COLOR_HUD_BG)
    for i, line in enumerate(hud_lines):
        draw.text((pad, pad + i * line_h), line, fill=COLOR_HUD, font=font)

    return np.asarray(img, dtype=np.uint8).copy()
