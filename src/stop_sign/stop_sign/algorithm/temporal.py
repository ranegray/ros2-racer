"""
Temporal filter for stop sign detections.

Wraps raw per-frame detections in:
  - position-consistency confirmation (N consecutive frames agreeing)
  - distance-based state thresholding (NONE / DETECTED / APPROACH)
  - post-APPROACH cooldown to avoid re-triggering on the same sign

Pure Python, no ROS imports. Used by both the ROS node and the standalone
visualizer.
"""
from collections import deque
from dataclasses import dataclass
from typing import Optional


STATE_NONE = 0
STATE_DETECTED = 1
STATE_APPROACH = 2


@dataclass
class FilterOutput:
    state: int
    detection: Optional[object]   # the StopSignDetection if any (else None)
    in_cooldown: bool
    cooldown_remaining_sec: float


class StopSignFilter:
    """
    Stateful per-frame filter. Call update(detection, now_sec) once per frame.

    Args read from config:
      consecutive_frames     - frames of position-consistent APPROACH-range
                               detections required before publishing APPROACH
      position_tolerance_px  - max bbox-center jitter between consecutive frames
      cooldown_sec           - suppression window after leaving APPROACH
      approach_distance_m    - APPROACH triggers below this distance
      detected_distance_m    - signs farther than this are reported as NONE
    """

    def __init__(self, config: dict):
        self.consecutive_frames = int(config.get('consecutive_frames', 3))
        self.position_tolerance_px = float(config.get('position_tolerance_px', 40))
        self.cooldown_sec = float(config.get('cooldown_sec', 3.0))
        self.approach_distance_m = float(config.get('approach_distance_m', 1.0))
        self.detected_distance_m = float(config.get('detected_distance_m', 2.5))

        self._recent_centers = deque(maxlen=self.consecutive_frames)
        self._cooldown_until: Optional[float] = None
        self._was_approach = False

    def reset(self):
        self._recent_centers.clear()
        self._cooldown_until = None
        self._was_approach = False

    def _bbox_center(self, det) -> tuple:
        x, y, w, h = det.bbox
        return (x + w / 2.0, y + h / 2.0)

    def _positions_consistent(self) -> bool:
        if len(self._recent_centers) < self.consecutive_frames:
            return False
        xs = [c[0] for c in self._recent_centers]
        ys = [c[1] for c in self._recent_centers]
        if max(xs) - min(xs) > self.position_tolerance_px:
            return False
        if max(ys) - min(ys) > self.position_tolerance_px:
            return False
        return True

    def update(self, detection, now_sec: float) -> FilterOutput:
        # Cooldown short-circuit: while cooling down, suppress APPROACH.
        in_cooldown = False
        cooldown_remaining = 0.0
        if self._cooldown_until is not None:
            if now_sec < self._cooldown_until:
                in_cooldown = True
                cooldown_remaining = max(0.0, self._cooldown_until - now_sec)
            else:
                self._cooldown_until = None

        if detection is None:
            self._recent_centers.clear()
            if self._was_approach:
                self._cooldown_until = now_sec + self.cooldown_sec
                self._was_approach = False
                in_cooldown = True
                cooldown_remaining = self.cooldown_sec
            return FilterOutput(STATE_NONE, None, in_cooldown, cooldown_remaining)

        d = float(detection.distance_m)

        if d >= self.detected_distance_m:
            # Too far to be a real action target — clear consistency buffer.
            self._recent_centers.clear()
            if self._was_approach:
                self._cooldown_until = now_sec + self.cooldown_sec
                self._was_approach = False
                in_cooldown = True
                cooldown_remaining = self.cooldown_sec
            return FilterOutput(STATE_NONE, detection, in_cooldown, cooldown_remaining)

        if d >= self.approach_distance_m:
            # Visible but not close enough to act on yet.
            self._recent_centers.clear()
            if self._was_approach:
                self._cooldown_until = now_sec + self.cooldown_sec
                self._was_approach = False
                in_cooldown = True
                cooldown_remaining = self.cooldown_sec
            state = STATE_NONE if in_cooldown else STATE_DETECTED
            return FilterOutput(state, detection, in_cooldown, cooldown_remaining)

        # Within approach distance.
        self._recent_centers.append(self._bbox_center(detection))

        if in_cooldown:
            # Cooldown wins: even a close, consistent sign is suppressed.
            return FilterOutput(STATE_NONE, detection, in_cooldown, cooldown_remaining)

        if self._positions_consistent():
            self._was_approach = True
            return FilterOutput(STATE_APPROACH, detection, False, 0.0)

        # Within range but not yet confirmed — show as DETECTED.
        return FilterOutput(STATE_DETECTED, detection, False, 0.0)
