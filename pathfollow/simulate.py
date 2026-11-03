"""Closed-loop simulation of one lap and the performance metrics."""
import math
from dataclasses import dataclass, field

import numpy as np

from . import controllers as C
from .profile import LIMITS, speed_profile
from .vehicle import CAR, MODELS


@dataclass
class Run:
    track: object
    controller: str
    model: str
    t: np.ndarray
    x: np.ndarray            # rear axle
    y: np.ndarray
    psi: np.ndarray
    v: np.ndarray
    delta: np.ndarray
    a: np.ndarray
    s: np.ndarray            # cumulative arc length
    e: np.ndarray            # signed cross-track error of the rear axle
    e_psi: np.ndarray
    ay: np.ndarray
    v_ref: np.ndarray
    completed: bool
    metrics: dict = field(default_factory=dict)


def simulate_lap(track, controller="stanley", model="kinematic", dt=0.01, car=CAR, lim=LIMITS,
                 v0=1.0, offset=0.0, heading_offset=0.0, t_max=None, ctrl_kwargs=None):
    """Drive one lap from a standing start (v0) at s = 0, optionally with an initial lateral /
    heading offset.  Returns a :class:`Run` (outputs sampled every 0.05 s)."""
    veh = MODELS[model](car)
    ctrl = C.CONTROLLERS[controller](car, **(ctrl_kwargs or {}))
    pi = C.SpeedPI(lim)
    v_prof = speed_profile(track, lim)
    x0, y0, psi0, _ = track.lookup(0.0)
    x0 += -offset * math.sin(psi0)
    y0 += offset * math.cos(psi0)
    state = veh.initial_state(x0, y0, psi0 + heading_offset, v0)
    delta = 0.0
    idx, s_cum, s_prev = 0, 0.0, 0.0
    t_max = t_max or 3.0 * track.length / 3.0
    rec = {k: [] for k in ("t", "x", "y", "psi", "v", "delta", "a", "s", "e", "e_psi", "ay", "v_ref")}
    n_steps = int(t_max / dt)
    completed = False
    sub = int(round(0.05 / dt))
    stat = dict(sq=0.0, emax=0.0, off=0, n=0, aymax=0.0)
    for k in range(n_steps):
        x, y, psi, v = veh.rear_axle(state)
        idx, s_loc, e, psi_ref, kap = track.project(x, y, idx)
        ds = (s_loc - s_prev + track.length / 2) % track.length - track.length / 2       # wrapped increment
        s_cum += ds
        s_prev = s_loc
        a_cmd, v_ref = pi.accel(track, v_prof, s_loc, v, dt)
        delta_cmd = ctrl.steer(track, idx, s_loc, e, psi_ref, kap, x, y, psi, v)
        delta_cmd = max(-car.delta_max, min(car.delta_max, delta_cmd))
        dmax = car.delta_rate * dt
        delta += max(-dmax, min(dmax, delta_cmd - delta))                                 # steering rate limit
        ay = veh.lateral_accel(state, delta)
        stat["sq"] += e * e
        stat["n"] += 1
        stat["emax"] = max(stat["emax"], abs(e))
        stat["aymax"] = max(stat["aymax"], abs(ay))
        stat["off"] += abs(e) > track.width / 2
        if k % sub == 0:
            for key, val in zip(rec, (k * dt, x, y, psi, v, delta, a_cmd, s_cum, e, C.wrap(psi - psi_ref), ay, v_ref)):
                rec[key].append(val)
        if s_cum >= track.length:
            completed = True
            break
        if abs(e) > 40.0:                                                                  # lost the track
            break
        state = veh.step(state, delta, a_cmd, dt)
    R = {k: np.array(v) for k, v in rec.items()}
    t_lap = (k + 1) * dt if completed else float("nan")
    metrics = dict(
        completed=bool(completed), lap_time_s=t_lap,
        mean_speed_ms=float(track.length / t_lap) if completed else float("nan"),
        rms_cross_track_m=float(math.sqrt(stat["sq"] / stat["n"])), max_cross_track_m=float(stat["emax"]),
        time_off_track_pct=float(100.0 * stat["off"] / stat["n"]),
        max_lateral_accel_ms2=float(stat["aymax"]), max_speed_ms=float(R["v"].max()),
        max_steering_deg=float(np.degrees(np.abs(R["delta"]).max())),
    )
    return Run(track, controller, model, R["t"], R["x"], R["y"], R["psi"], R["v"], R["delta"], R["a"], R["s"],
               R["e"], R["e_psi"], R["ay"], R["v_ref"], completed, metrics)
