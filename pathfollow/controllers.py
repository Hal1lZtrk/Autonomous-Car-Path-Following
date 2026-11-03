"""Controllers.

Lateral:   ``PurePursuit``  geometric, aims at a point ``Ld = ld0 + kv v`` ahead on the centreline
           ``Stanley``      front-axle cross-track + heading error law, with curvature feed-forward
Longitudinal: ``SpeedPI``   PI on the speed error with feed-forward of the planned acceleration

All lateral controllers take the rear-axle pose ``(x, y, psi, v)`` and return a steering angle.
"""
import math

from .profile import speed_at


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    name = "pure_pursuit"

    def __init__(self, car, ld0=4.0, kv=0.45, ld_min=4.0):
        self.L, self.ld0, self.kv, self.ld_min = car.wheelbase, ld0, kv, ld_min

    def steer(self, track, i, s, e, psi_ref, kappa_ref, x, y, psi, v):
        ld = max(self.ld_min, self.ld0 + self.kv * v)
        xt, yt, _, _ = track.lookup(s + ld)
        alpha = wrap(math.atan2(yt - y, xt - x) - psi)
        return math.atan2(2.0 * self.L * math.sin(alpha), ld)


class Stanley:
    name = "stanley"

    def __init__(self, car, k=2.0, k_soft=1.0, feedforward=True):
        self.L, self.k, self.k_soft, self.ff = car.wheelbase, k, k_soft, feedforward
        self._hint = 0

    def steer(self, track, i, s, e, psi_ref, kappa_ref, x, y, psi, v):
        xf, yf = x + self.L * math.cos(psi), y + self.L * math.sin(psi)       # front axle
        i_f, _, e_f, psi_f, kap_f = track.project(xf, yf, i)
        delta = wrap(psi_f - psi) - math.atan2(self.k * e_f, self.k_soft + v)      # e_f > 0: car is left of the path
        if self.ff:
            delta += math.atan(self.L * kap_f)
        return delta


class SpeedPI:
    def __init__(self, lim, kp=1.2, ki=0.25, lookahead_t=0.3):
        self.lim, self.kp, self.ki, self.la = lim, kp, ki, lookahead_t
        self.integral = 0.0

    def accel(self, track, v_prof, s, v, dt):
        v_ref, dvds = speed_at(track, v_prof, s + self.la * v)
        a_ff = v_ref * dvds
        err = v_ref - v
        self.integral = max(-3.0, min(3.0, self.integral + err * dt))
        a = a_ff + self.kp * err + self.ki * self.integral
        return max(-self.lim.a_brk, min(self.lim.a_acc, a)), v_ref


CONTROLLERS = {"pure_pursuit": PurePursuit, "stanley": Stanley}
