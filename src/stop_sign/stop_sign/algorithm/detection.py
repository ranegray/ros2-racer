"""
Stop sign detection algorithm.

Pure functions, no ROS imports. Color + shape pipeline:
  HSV mask (red, two-band) -> morphology -> connected components ->
  area filter -> aspect+fill shape filter -> distance estimation.

Uses numpy, scipy.ndimage, skimage.color. No cv2.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy import ndimage
from skimage.color import rgb2hsv


@dataclass
class StopSignDetection:
    bbox: tuple              # (x, y, w, h) in image coords
    distance_m: float        # estimated, from apparent size
    confidence: float        # [0, 1]
    blob_pixels: int


@dataclass
class DetectionDebugInfo:
    """Optional intermediate outputs for visualization. Only populated if requested."""
    mask: Optional[np.ndarray] = None
    all_candidates: list = field(default_factory=list)
    accepted: Optional[StopSignDetection] = None


def _hsv_red_mask(rgb: np.ndarray, config: dict) -> np.ndarray:
    """Two-band hue mask for red. Returns bool array."""
    hsv = rgb2hsv(rgb)  # floats in [0, 1]
    hue = hsv[..., 0] * 360.0
    sat = hsv[..., 1] * 100.0
    val = hsv[..., 2] * 100.0

    h1l = float(config['hue_lower_1'])
    h1u = float(config['hue_upper_1'])
    h2l = float(config['hue_lower_2'])
    h2u = float(config['hue_upper_2'])
    sl = float(config['sat_lower'])
    su = float(config['sat_upper'])
    vl = float(config['val_lower'])
    vu = float(config['val_upper'])

    band1 = (hue >= h1l) & (hue <= h1u)
    band2 = (hue >= h2l) & (hue <= h2u)
    sat_ok = (sat >= sl) & (sat <= su)
    val_ok = (val >= vl) & (val <= vu)
    return (band1 | band2) & sat_ok & val_ok


def _clean_mask(mask: np.ndarray, open_k: int, close_k: int) -> np.ndarray:
    if open_k > 1:
        mask = ndimage.binary_opening(mask, structure=np.ones((open_k, open_k), dtype=bool))
    if close_k > 1:
        mask = ndimage.binary_closing(mask, structure=np.ones((close_k, close_k), dtype=bool))
    return mask


def _shape_test(blob_area: int, bbox_w: int, bbox_h: int, config: dict) -> tuple:
    """
    Returns (passed, reason, margin) where margin in [0,1] reflects how
    comfortably the blob is inside the shape window (used for confidence).
    reason is 'accepted' if passed, else 'aspect' or 'fill'.
    """
    if bbox_w <= 0 or bbox_h <= 0:
        return False, 'aspect', 0.0
    aspect = bbox_w / bbox_h
    a_min = float(config['aspect_min'])
    a_max = float(config['aspect_max'])
    if aspect < a_min or aspect > a_max:
        return False, 'aspect', 0.0

    bbox_area = bbox_w * bbox_h
    if bbox_area <= 0:
        return False, 'fill', 0.0
    fill = blob_area / bbox_area
    f_min = float(config['fill_min'])
    f_max = float(config['fill_max'])
    if fill < f_min or fill > f_max:
        return False, 'fill', 0.0

    # Margin: distance from window edges, normalized by half-width.
    f_mid = 0.5 * (f_min + f_max)
    f_half = 0.5 * (f_max - f_min)
    a_mid = 0.5 * (a_min + a_max)
    a_half = 0.5 * (a_max - a_min)
    fill_margin = max(0.0, 1.0 - abs(fill - f_mid) / max(f_half, 1e-6))
    aspect_margin = max(0.0, 1.0 - abs(aspect - a_mid) / max(a_half, 1e-6))
    margin = min(1.0, 0.5 * (fill_margin + aspect_margin))
    return True, 'accepted', margin


def detect(rgb: np.ndarray,
           config: dict,
           focal_px: float,
           debug: bool = False):
    """
    Run color + shape pipeline.

    Returns (best_detection_or_None, debug_info_or_None).
    debug_info is populated only if debug=True; otherwise None.
    The ROS node calls with debug=False; the visualizer calls with debug=True.
    """
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"detect() expects HxWx3 RGB; got shape {rgb.shape}")

    raw_mask = _hsv_red_mask(rgb, config)
    open_k = int(config.get('morph_open_kernel', 3))
    close_k = int(config.get('morph_close_kernel', 5))
    mask = _clean_mask(raw_mask, open_k, close_k)

    debug_info = DetectionDebugInfo(mask=mask.copy()) if debug else None

    labeled, n_labels = ndimage.label(mask)
    if n_labels == 0:
        return None, debug_info

    min_area = int(config['min_area_px'])
    max_area = int(config['max_area_px'])
    real_size_m = float(config['real_size_m'])

    best: Optional[StopSignDetection] = None
    best_blob_area = -1

    # ndimage.find_objects returns slices indexed 0..n_labels-1 by label-1.
    slices = ndimage.find_objects(labeled)
    for label_idx, sl in enumerate(slices, start=1):
        if sl is None:
            continue
        sub = labeled[sl] == label_idx
        blob_area = int(sub.sum())
        if blob_area < min_area or blob_area > max_area:
            continue

        y0, x0 = sl[0].start, sl[1].start
        bbox_h = sl[0].stop - sl[0].start
        bbox_w = sl[1].stop - sl[1].start
        bbox = (int(x0), int(y0), int(bbox_w), int(bbox_h))

        passed, reason, margin = _shape_test(blob_area, bbox_w, bbox_h, config)

        if debug_info is not None:
            debug_info.all_candidates.append({
                'bbox': bbox,
                'blob_pixels': blob_area,
                'reason': reason,
            })

        if not passed:
            continue

        observed_size_px = float(max(bbox_w, bbox_h))
        if observed_size_px <= 0:
            continue
        distance_m = (real_size_m * float(focal_px)) / observed_size_px

        size_conf = min(1.0, blob_area / 1000.0)
        confidence = float(min(1.0, max(0.0, size_conf * margin)))

        det = StopSignDetection(
            bbox=bbox,
            distance_m=float(distance_m),
            confidence=confidence,
            blob_pixels=blob_area,
        )

        if blob_area > best_blob_area:
            best = det
            best_blob_area = blob_area

    if debug_info is not None:
        debug_info.accepted = best

    return best, debug_info
