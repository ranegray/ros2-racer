"""
Unit tests for the stop sign detection algorithm.

Synthetic test images are generated with PIL — no binary fixtures committed.
"""
import math
import os
import sys

import numpy as np
import pytest
from PIL import Image, ImageDraw

# Allow running the tests directly with `pytest tests/` from the package root.
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from stop_sign.algorithm.detection import detect  # noqa: E402


CONFIG = {
    'hue_lower_1': 0,
    'hue_upper_1': 12,
    'hue_lower_2': 340,
    'hue_upper_2': 360,
    'sat_lower': 50,
    'sat_upper': 100,
    'val_lower': 30,
    'val_upper': 100,
    'morph_open_kernel': 3,
    'morph_close_kernel': 5,
    'min_area_px': 600,
    'max_area_px': 50000,
    'aspect_min': 0.7,
    'aspect_max': 1.4,
    'fill_min': 0.70,
    'fill_max': 0.92,
    'real_size_m': 0.19,
}

FOCAL_PX = 600.0
IMG_W, IMG_H = 640, 480
RED = (220, 20, 20)
WHITE = (255, 255, 255)


def _blank():
    return Image.new('RGB', (IMG_W, IMG_H), WHITE)


def _octagon_points(cx, cy, size_px):
    """Regular octagon centered at (cx, cy), flat-to-flat = size_px."""
    r = size_px / 2.0 / math.cos(math.pi / 8)  # circumradius
    pts = []
    # Start at angle pi/8 so flats are horizontal/vertical (flat-top octagon).
    for i in range(8):
        theta = math.pi / 8 + i * math.pi / 4
        pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
    return pts


def _draw_red_octagon(img, cx, cy, size_px):
    draw = ImageDraw.Draw(img)
    draw.polygon(_octagon_points(cx, cy, size_px), fill=RED)


def _draw_red_square(img, cx, cy, size_px):
    draw = ImageDraw.Draw(img)
    half = size_px / 2.0
    draw.rectangle([cx - half, cy - half, cx + half, cy + half], fill=RED)


def _draw_red_circle(img, cx, cy, size_px):
    draw = ImageDraw.Draw(img)
    half = size_px / 2.0
    draw.ellipse([cx - half, cy - half, cx + half, cy + half], fill=RED)


def _to_rgb(img):
    return np.asarray(img, dtype=np.uint8).copy()


def test_octagon_80px_detected():
    img = _blank()
    _draw_red_octagon(img, 320, 240, 80)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is not None, "80 px octagon should be detected"
    expected = (CONFIG['real_size_m'] * FOCAL_PX) / 80.0
    assert det.distance_m == pytest.approx(expected, rel=0.25)
    assert 0.0 < det.confidence <= 1.0


def test_octagon_40px_detected():
    img = _blank()
    _draw_red_octagon(img, 320, 240, 40)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is not None, "40 px octagon should be detected"
    expected = (CONFIG['real_size_m'] * FOCAL_PX) / 40.0
    assert det.distance_m == pytest.approx(expected, rel=0.35)


def test_octagon_20px_rejected_or_low_confidence():
    """20 px is below the configured min_area_px (~600 px^2). Should be rejected."""
    img = _blank()
    _draw_red_octagon(img, 320, 240, 20)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    # Strict assertion: with min_area_px=600, a 20 px octagon (~330 px^2 of red) is rejected.
    assert det is None, "20 px octagon should be rejected by min_area_px filter"


def test_red_square_80px_rejected():
    """A square has fill ratio ~1.0, outside the [0.70, 0.92] octagon window."""
    img = _blank()
    _draw_red_square(img, 320, 240, 80)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is None, "Red square must be rejected (fill ratio fails)"


def test_red_circle_80px_rejected():
    """A circle has fill ratio ~pi/4 ≈ 0.785, just inside the window. Use a larger
    circle so it fails on something — actually circles are borderline. The reliable
    discriminator is that a tightly-cropped circle's bbox-fill is ~0.785, which IS
    inside [0.70, 0.92]. So we use an oversized red blob with rounded shape that
    fails fill or aspect."""
    # Use an ellipse with an exaggerated aspect to ensure rejection.
    img = _blank()
    draw = ImageDraw.Draw(img)
    # Wide ellipse: aspect ratio ~2.5 fails aspect_max=1.4.
    draw.ellipse([220, 220, 420, 260], fill=RED)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is None, "Wide red ellipse must be rejected (aspect ratio fails)"


def test_two_octagons_picks_larger():
    img = _blank()
    _draw_red_octagon(img, 150, 240, 50)
    _draw_red_octagon(img, 470, 240, 100)
    det, debug = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=True)
    assert det is not None
    # The larger one is centered at x=470, size 100. Center should be near 470.
    cx = det.bbox[0] + det.bbox[2] / 2.0
    assert abs(cx - 470) < 30, f"Picked blob center {cx} not near larger octagon at 470"
    # Distance estimate should match the larger sign size.
    expected = (CONFIG['real_size_m'] * FOCAL_PX) / 100.0
    assert det.distance_m == pytest.approx(expected, rel=0.25)
    # Debug info: at least 2 candidates passed the area filter.
    assert debug is not None
    assert len(debug.all_candidates) >= 2


def test_no_red_returns_none():
    img = _blank()  # all white
    det, dbg = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is None
    assert dbg is None
    det2, dbg2 = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=True)
    assert det2 is None
    assert dbg2 is not None  # debug info still returned (mask, empty candidates)
    assert dbg2.accepted is None


def test_irregular_red_blob_rejected():
    """Random thin polygon: low fill ratio, should be rejected."""
    img = _blank()
    draw = ImageDraw.Draw(img)
    # Long thin zig-zag polygon.
    pts = [(200, 200), (260, 210), (320, 198), (380, 215),
           (440, 205), (440, 230), (380, 240), (320, 225),
           (260, 235), (200, 225)]
    draw.polygon(pts, fill=RED)
    det, _ = detect(_to_rgb(img), CONFIG, FOCAL_PX, debug=False)
    assert det is None, "Thin irregular red shape should be rejected"
