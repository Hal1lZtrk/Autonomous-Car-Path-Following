"""Vehicle models (pure-Python scalar RK4 for speed).

``KinematicBicycle``   state [x, y, psi, v]                  reference point: rear axle
``DynamicBicycle``     state [x, y, psi, vx, vy, r]          reference point: centre of gravity,
                       linear tyres with a friction-circle style saturation

Both take the front steering angle ``delta`` and the longitudinal acceleration ``a`` as inputs.
The steering angle is limited in magnitude and rate (``DELTA_MAX``, ``DELTA_RATE``), the
acceleration by the controller.  ``rear_axle()`` gives (x, y, psi, v) of the rear axle for the
controllers in both cases.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CarParams:
    wheelbase: float = 2.7          # L [m]
    a: float = 1.2                  # CG -> front axle [m]
    mass: float = 1500.0            # [kg]
    Iz: float = 2500.0              # yaw inertia [kg m^2]
    Cf: float = 100e3               # front cornering stiffness [N/rad]
    Cr: float = 120e3               # rear cornering stiffness [N/rad]
    mu: float = 1.0                 # tyre-road friction
    g: float = 9.81
    delta_max: float = 0.45         # steering limit [rad]
    delta_rate: float = 0.8         # steering rate limit [rad/s]

    @property
    def b(self):
        return self.wheelbase - self.a


CAR = CarParams()


class KinematicBicycle:
    name = "kinematic"

    def __init__(self, p: CarParams = CAR):
        self.p = p

    def initial_state(self, x, y, psi, v):
        return (x, y, psi, v)

    @staticmethod
    def _f(s, delta, a, L):
        x, y, psi, v = s
        return (v * math.cos(psi), v * math.sin(psi), v * math.tan(delta) / L, a)

    def step(self, s, delta, a, dt):
        L = self.p.wheelbase
        f = self._f
        k1 = f(s, delta, a, L)
        s2 = tuple(si + 0.5 * dt * ki for si, ki in zip(s, k1))
        k2 = f(s2, delta, a, L)
        s3 = tuple(si + 0.5 * dt * ki for si, ki in zip(s, k2))
        k3 = f(s3, delta, a, L)
        s4 = tuple(si + dt * ki for si, ki in zip(s, k3))
        k4 = f(s4, delta, a, L)
        return tuple(si + dt / 6.0 * (a1 + 2 * a2 + 2 * a3 + a4) for si, a1, a2, a3, a4 in zip(s, k1, k2, k3, k4))

    def rear_axle(self, s):
        return s[0], s[1], s[2], s[3]

    def lateral_accel(self, s, delta):
        return s[3] ** 2 * math.tan(delta) / self.p.wheelbase


class DynamicBicycle:
    name = "dynamic"

    def __init__(self, p: CarParams = CAR):
        self.p = p
        self.Fz_f = p.mass * p.g * p.b / p.wheelbase
        self.Fz_r = p.mass * p.g * p.a / p.wheelbase

    def initial_state(self, x_rear, y_rear, psi, v):
        b = self.p.b
        return (x_rear + b * math.cos(psi), y_rear + b * math.sin(psi), psi, v, 0.0, 0.0)

    def _tyre(self, C, alpha, Fz):
        """Linear tyre force with smooth saturation at mu*Fz."""
        fmax = self.p.mu * Fz
        return fmax * math.tanh(C * alpha / fmax)

    def _f(self, s, delta, a):
        p = self.p
        x, y, psi, vx, vy, r = s
        vxs = max(vx, 1.0)                                  # avoid the low-speed singularity of slip angles
        af = delta - math.atan2(vy + p.a * r, vxs)
        ar = -math.atan2(vy - p.b * r, vxs)
        Ff = self._tyre(p.Cf, af, self.Fz_f)
        Fr = self._tyre(p.Cr, ar, self.Fz_r)
        c, sn = math.cos(psi), math.sin(psi)
        return (vx * c - vy * sn, vx * sn + vy * c, r, a + vy * r,
                (Ff * math.cos(delta) + Fr) / p.mass - vx * r,
                (p.a * Ff * math.cos(delta) - p.b * Fr) / p.Iz)

    def step(self, s, delta, a, dt):
        f = self._f
        k1 = f(s, delta, a)
        k2 = f(tuple(si + 0.5 * dt * ki for si, ki in zip(s, k1)), delta, a)
        k3 = f(tuple(si + 0.5 * dt * ki for si, ki in zip(s, k2)), delta, a)
        k4 = f(tuple(si + dt * ki for si, ki in zip(s, k3)), delta, a)
        return tuple(si + dt / 6.0 * (a1 + 2 * a2 + 2 * a3 + a4) for si, a1, a2, a3, a4 in zip(s, k1, k2, k3, k4))

    def rear_axle(self, s):
        x, y, psi, vx = s[0], s[1], s[2], s[3]
        return x - self.p.b * math.cos(psi), y - self.p.b * math.sin(psi), psi, vx

    def lateral_accel(self, s, delta):
        """Lateral acceleration of the CG in the body frame: v_y' + v_x r = (Ff cos(delta) + Fr) / m."""
        return self._f(s, delta, 0.0)[4] + s[3] * s[5]


MODELS = {"kinematic": KinematicBicycle, "dynamic": DynamicBicycle}
