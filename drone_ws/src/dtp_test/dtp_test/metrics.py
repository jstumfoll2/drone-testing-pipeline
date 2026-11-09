"""Pass/fail metrics for landing runs, computed from recorded samples.

All positions are world-frame (Gazebo ENU), metres. These functions are pure so
they can score live runs, recorded bags, and unit-test fixtures alike.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class PoseSample:
    t: float  # seconds
    position: Vec3


@dataclass(frozen=True)
class LandingResult:
    landed: bool
    landing_error_m: Optional[float]  # horizontal distance to marker centre
    time_to_land_s: Optional[float]
    max_altitude_m: float
    passed: bool
    reason: str


def horizontal_error(position: Sequence[float], target: Sequence[float]) -> float:
    return float(np.hypot(position[0] - target[0], position[1] - target[1]))


def detect_touchdown(
    samples: Sequence[PoseSample],
    ground_z: float = 0.0,
    height_tol_m: float = 0.15,
    still_speed_mps: float = 0.05,
    settle_s: float = 1.0,
) -> Optional[int]:
    """Index of the first sample where the vehicle is on the ground and stays still.

    "On the ground" = within ``height_tol_m`` of ``ground_z`` and moving slower than
    ``still_speed_mps`` for at least ``settle_s``.
    """
    if len(samples) < 2:
        return None
    t = np.array([s.t for s in samples])
    p = np.array([s.position for s in samples], dtype=float)
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("samples must be strictly increasing in time")
    speed = np.r_[np.inf, np.linalg.norm(np.diff(p, axis=0), axis=1) / dt]
    low = (p[:, 2] - ground_z) <= height_tol_m
    still = speed <= still_speed_mps
    ok = low & still
    start = None
    for i, flag in enumerate(ok):
        if flag and start is None:
            start = i
        elif not flag:
            start = None
        if start is not None and t[i] - t[start] >= settle_s:
            return start
    return None


def score_landing(
    samples: Sequence[PoseSample],
    marker_position: Sequence[float],
    max_error_m: float,
    timeout_s: float,
    takeoff_altitude_m: float = 1.0,
    ground_z: float = 0.0,
) -> LandingResult:
    """Score one landing run against the scenario thresholds.

    The run must first climb above half the take-off altitude (so a vehicle that
    never left the pad does not count as a landing), then touch down.
    """
    if not samples:
        return LandingResult(False, None, None, 0.0, False, "no samples")
    t0 = samples[0].t
    z = np.array([s.position[2] for s in samples]) - ground_z
    max_alt = float(z.max())
    airborne = np.nonzero(z >= 0.5 * takeoff_altitude_m)[0]
    if airborne.size == 0:
        return LandingResult(False, None, None, max_alt, False, "never took off")

    after = samples[airborne[0]:]
    idx = detect_touchdown(after, ground_z=ground_z)
    if idx is None:
        return LandingResult(False, None, None, max_alt, False, "did not land")
    touchdown = after[idx]
    elapsed = touchdown.t - t0
    err = horizontal_error(touchdown.position, marker_position)
    if elapsed > timeout_s:
        return LandingResult(True, err, elapsed, max_alt, False, f"timeout ({elapsed:.1f} s)")
    if err > max_error_m:
        return LandingResult(True, err, elapsed, max_alt, False,
                             f"landing error {err:.3f} m > {max_error_m:.3f} m")
    return LandingResult(True, err, elapsed, max_alt, True, "ok")


def detection_rate(frames: int, detections: int) -> float:
    if frames < 0 or detections < 0 or detections > frames:
        raise ValueError("need 0 <= detections <= frames")
    return 0.0 if frames == 0 else detections / frames


def latency_stats_ms(capture_stamps_s: Iterable[float], output_stamps_s: Iterable[float]) -> dict:
    """End-to-end latency (output time - capture time) summary in milliseconds."""
    cap = np.asarray(list(capture_stamps_s), dtype=float)
    out = np.asarray(list(output_stamps_s), dtype=float)
    if cap.shape != out.shape:
        raise ValueError("capture and output stamps must pair up")
    if cap.size == 0:
        return {"n": 0}
    lat = (out - cap) * 1000.0
    if np.any(lat < 0):
        raise ValueError("output stamp earlier than capture stamp; check clock sync")
    return {
        "n": int(lat.size),
        "mean": float(lat.mean()),
        "p50": float(np.percentile(lat, 50)),
        "p95": float(np.percentile(lat, 95)),
        "max": float(lat.max()),
    }


def summarize(results: Sequence[LandingResult]) -> dict:
    """Campaign summary: success rate and landing-error distribution."""
    n = len(results)
    errors: List[float] = [r.landing_error_m for r in results if r.landing_error_m is not None]
    summary = {
        "runs": n,
        "passed": sum(r.passed for r in results),
        "success_rate": 0.0 if n == 0 else sum(r.passed for r in results) / n,
        "landed": sum(r.landed for r in results),
    }
    if errors:
        e = np.asarray(errors)
        summary.update(
            error_mean_m=float(e.mean()),
            error_p95_m=float(np.percentile(e, 95)),
            error_max_m=float(e.max()),
        )
    return summary
