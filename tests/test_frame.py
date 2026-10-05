import math

import pytest

from sim.frame import estimate_gyro_drift, gps_to_enu, to_enu, to_gps


def test_worked_example_from_report():
    lat, lon = to_gps(10, 5, 36.8000, 10.1800, 90)      # robot faces East
    assert lat == pytest.approx(36.800045, abs=1e-6)
    assert lon == pytest.approx(10.180112, abs=1e-6)


@pytest.mark.parametrize("psi,x,y,east,north", [
    (0, 1, 0, 0, 1),        # facing North: forward = North
    (0, 0, 1, -1, 0),       # facing North: left = West
    (90, 1, 0, 1, 0),       # facing East
    (270, 1, 0, -1, 0),     # facing West (our building)
    (270, 0, 1, 0, -1),     # facing West: left = South
])
def test_axes(psi, x, y, east, north):
    e, n = to_enu(x, y, psi)
    assert (e, n) == pytest.approx((east, north), abs=1e-9)


def test_gps_roundtrip():
    lat, lon = to_gps(-7.3, 4.1, 36.8000, 10.1800, 270)
    e, n = gps_to_enu(lat, lon, 36.8000, 10.1800)
    assert (e, n) == pytest.approx(to_enu(-7.3, 4.1, 270), abs=1e-3)


def test_gyro_drift_recovered_from_loop_closure():
    # square loop 10 m each side, driven with a heading error growing 0.01 rad per metre
    beta, path, x, y, d, h = 0.01, [(0.0, 0.0, 0.0)], 0.0, 0.0, 0.0, 0.0
    for side in range(4):
        for _ in range(20):
            d += 0.5
            a = side * math.pi / 2 + beta * d
            x, y = x + 0.5 * math.cos(a), y + 0.5 * math.sin(a)
            path.append((x, y, d))
    est, before, after = estimate_gyro_drift(path)
    assert before > 1.0
    assert est == pytest.approx(beta, rel=0.05)
    assert after < 0.05
