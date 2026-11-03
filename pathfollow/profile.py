"""Speed profile along the track from curvature and acceleration limits.

    v_curve(s) = sqrt(a_lat_max / |kappa(s)|)          (lateral-acceleration limit)
    forward pass:   v_{i+1}^2 <= v_i^2 + 2 a_acc  ds   (engine / traction limit)
    backward pass:  v_i^2     <= v_{i+1}^2 + 2 a_brk ds (braking limit)

The track is a closed loop, so both passes are repeated twice around the lap.  Longitudinal and
lateral limits are treated independently (no combined-slip "g-g" coupling), which is a simplification.
"""
from dataclasses import dataclass

import numpy as np

from .tracks import DS


@dataclass(frozen=True)
class Limits:
    a_lat: float = 7.0          # lateral acceleration limit used for planning [m/s^2]  (~0.71 g)
    a_acc: float = 3.0          # max acceleration [m/s^2]
    a_brk: float = 6.0          # max braking [m/s^2]
    v_max: float = 60.0         # top speed [m/s]  (216 km/h)


LIMITS = Limits()


def speed_profile(track, lim: Limits = LIMITS, v_start=None):
    k = np.maximum(np.abs(track.kappa), 1e-9)
    v2 = np.minimum(lim.a_lat / k, lim.v_max ** 2)
    n = len(v2)
    for _ in range(2):                                  # closed loop: go around twice
        for i in range(2 * n):
            a, b = i % n, (i + 1) % n
            v2[b] = min(v2[b], v2[a] + 2 * lim.a_acc * DS)
        for i in range(2 * n, 0, -1):
            a, b = (i - 1) % n, i % n
            v2[a] = min(v2[a], v2[b] + 2 * lim.a_brk * DS)
    v = np.sqrt(v2)
    return v


def speed_at(track, v_prof, s):
    """Interpolated reference speed and its derivative along s."""
    n = len(v_prof)
    s = s % track.length
    i = int(s // DS) % n
    j = (i + 1) % n
    f = (s - i * DS) / DS
    v = v_prof[i] + f * (v_prof[j] - v_prof[i])
    return v, (v_prof[j] - v_prof[i]) / DS
