import numpy as np
import pytest

from dtp_test.metrics import (
    PoseSample,
    detect_touchdown,
    detection_rate,
    horizontal_error,
    latency_stats_ms,
    score_landing,
    summarize,
)


def flight(final_xy=(0.02, 0.0), alt=5.0, rate_hz=20, hover_s=5.0, settle_s=3.0):
    """Synthetic take-off, hover, descend and land trajectory."""
    samples, t = [], 0.0
    dt = 1.0 / rate_hz
    # climb at 1 m/s
    z = 0.0
    while z < alt:
        samples.append(PoseSample(t, (0.0, 0.0, z)))
        z += 1.0 * dt
        t += dt
    for _ in range(int(hover_s * rate_hz)):
        samples.append(PoseSample(t, (0.0, 0.0, alt)))
        t += dt
    # descend at 0.5 m/s while drifting to final_xy
    steps = int(alt / 0.5 * rate_hz)
    for i in range(steps + 1):
        f = i / steps
        samples.append(PoseSample(t, (final_xy[0] * f, final_xy[1] * f, alt * (1 - f))))
        t += dt
    for _ in range(int(settle_s * rate_hz)):
        samples.append(PoseSample(t, (final_xy[0], final_xy[1], 0.0)))
        t += dt
    return samples


def test_horizontal_error_ignores_height():
    assert horizontal_error((3, 4, 10), (0, 0, 0)) == pytest.approx(5.0)


def test_precise_landing_passes():
    r = score_landing(flight((0.03, -0.02)), (0, 0, 0), max_error_m=0.05, timeout_s=120,
                      takeoff_altitude_m=5)
    assert r.passed and r.landed
    assert r.landing_error_m == pytest.approx(np.hypot(0.03, 0.02), abs=1e-6)
    assert r.max_altitude_m == pytest.approx(5.0, abs=0.06)


def test_inaccurate_landing_fails():
    r = score_landing(flight((0.2, 0.0)), (0, 0, 0), 0.05, 120, takeoff_altitude_m=5)
    assert r.landed and not r.passed
    assert "landing error" in r.reason


def test_timeout_fails():
    r = score_landing(flight(), (0, 0, 0), 0.05, timeout_s=5, takeoff_altitude_m=5)
    assert not r.passed and "timeout" in r.reason


def test_never_took_off():
    samples = [PoseSample(i * 0.1, (0, 0, 0)) for i in range(100)]
    r = score_landing(samples, (0, 0, 0), 0.05, 120, takeoff_altitude_m=5)
    assert not r.passed and r.reason == "never took off"


def test_did_not_land():
    samples = flight()[:150]  # cut off mid-air
    r = score_landing(samples, (0, 0, 0), 0.05, 120, takeoff_altitude_m=5)
    assert r.reason == "did not land"


def test_touchdown_requires_settling():
    # Touches the ground for 0.5 s only, then bounces back up.
    samples = [PoseSample(i * 0.1, (0, 0, 0.0 if 10 <= i < 15 else 1.0)) for i in range(30)]
    assert detect_touchdown(samples, settle_s=1.0) is None


def test_touchdown_rejects_non_monotonic_time():
    with pytest.raises(ValueError):
        detect_touchdown([PoseSample(1.0, (0, 0, 0)), PoseSample(1.0, (0, 0, 0))])


def test_detection_rate():
    assert detection_rate(0, 0) == 0.0
    assert detection_rate(10, 9) == 0.9
    with pytest.raises(ValueError):
        detection_rate(5, 6)


def test_latency_stats():
    cap = [0.0, 1.0, 2.0, 3.0]
    out = [0.020, 1.030, 2.025, 3.100]
    s = latency_stats_ms(cap, out)
    assert s["n"] == 4
    assert s["max"] == pytest.approx(100.0)
    assert s["p50"] == pytest.approx(27.5)
    with pytest.raises(ValueError):
        latency_stats_ms([1.0], [0.5])


def test_summarize():
    good = score_landing(flight((0.01, 0)), (0, 0, 0), 0.05, 120, takeoff_altitude_m=5)
    bad = score_landing(flight((0.3, 0)), (0, 0, 0), 0.05, 120, takeoff_altitude_m=5)
    s = summarize([good, bad])
    assert s["runs"] == 2 and s["passed"] == 1 and s["success_rate"] == 0.5
    assert s["error_max_m"] == pytest.approx(0.3, abs=1e-6)
    assert summarize([])["runs"] == 0
