import math

import numpy as np
import pytest

from pathfollow import tracks as T
from pathfollow.profile import LIMITS, speed_profile
from pathfollow.simulate import simulate_lap
from pathfollow.vehicle import CAR, DynamicBicycle, KinematicBicycle

NAMES = list(T.TRACKS)


@pytest.fixture(scope="module", params=NAMES)
def track(request):
    return T.get_track(request.param)


def test_track_is_closed_ccw_and_uniform(track):
    d = np.hypot(np.diff(np.r_[track.x, track.x[0]]), np.diff(np.r_[track.y, track.y[0]]))
    assert np.all(np.abs(d - 1.0) < 0.05)                              # 1 m sampling (arc length)
    assert track.psi[-1] - track.psi[0] > 0
    assert abs((track.kappa * 1.0).sum() - 2 * math.pi) < 0.25         # total turning = one loop, CCW


def test_curvature_matches_geometry(track):
    h = np.gradient(track.psi)                                         # d psi / ds with ds = 1 m
    err = np.abs(h - track.kappa)
    assert np.median(err) < 0.002 and np.mean(err < 0.01) > 0.95        # arc/line junctions differ by one sample


def test_imola_length_and_radius():
    t = T.imola()
    assert abs(t.length - 4909) < 1.0
    assert 1 / np.abs(t.kappa).max() > 15


def test_project_recovers_offset():
    t = T.circle(100)
    x, y, psi, _ = t.lookup(50.0)
    i, s, e, *_ = t.project(x - 2 * math.sin(psi), y + 2 * math.cos(psi), 40)    # 2 m to the left
    assert e == pytest.approx(2.0, abs=0.05) and s == pytest.approx(50.0, abs=1.0)


def test_models_agree_in_steady_low_speed_turn():
    k, d = KinematicBicycle(CAR), DynamicBicycle(CAR)
    sk, sd = k.initial_state(0, 0, 0, 3.0), d.initial_state(0, 0, 0, 3.0)
    for _ in range(300):
        sk, sd = k.step(sk, 0.1, 0.0, 0.01), d.step(sd, 0.1, 0.0, 0.01)
    assert math.hypot(k.rear_axle(sk)[0] - d.rear_axle(sd)[0], k.rear_axle(sk)[1] - d.rear_axle(sd)[1]) < 0.3


def test_speed_profile_respects_limits(track):
    v = speed_profile(track, LIMITS)
    assert v.max() <= LIMITS.v_max + 1e-9
    assert np.all(v ** 2 * np.abs(track.kappa) <= LIMITS.a_lat * 1.01 + 1e-9)
    dv = np.diff(np.r_[v, v[0]])
    assert dv.max() * v.mean() < 1e9 and np.all(v[1:] ** 2 - v[:-1] ** 2 <= 2 * LIMITS.a_acc * 1.0 + 1e-6)


@pytest.mark.parametrize("controller", ["pure_pursuit", "stanley"])
def test_kinematic_laps_complete_on_track(track, controller):
    r = simulate_lap(track, controller, "kinematic")
    assert r.metrics["completed"] and r.metrics["max_cross_track_m"] < 1.5


def test_stanley_dynamic_laps_stay_on_track(track):
    r = simulate_lap(track, "stanley", "dynamic")
    m = r.metrics
    assert m["completed"] and m["max_cross_track_m"] < track.width / 2 and m["time_off_track_pct"] == 0
    assert m["max_lateral_accel_ms2"] < 9.5                              # within tyre limit (mu g ~ 9.8)


def test_recovers_from_initial_offset():
    t = T.circle()
    r = simulate_lap(t, "stanley", "kinematic", offset=3.0, heading_offset=0.2)
    assert r.metrics["completed"] and abs(r.e[-20:]).max() < 0.5


def test_unknown_track_raises():
    with pytest.raises(ValueError):
        T.get_track("monza")
