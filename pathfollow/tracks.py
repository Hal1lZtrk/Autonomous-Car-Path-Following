"""Closed tracks: circle, rounded square, rounded trapezoid and the Imola circuit.

Every track is stored as a centreline sampled every ``DS = 1 m`` with exact heading ``psi`` and
signed curvature ``kappa`` (positive = left turn).  All tracks run counter-clockwise and the car
starts at ``s = 0``.  ``Track.project`` gives the arc length, the signed cross-track error
(positive = car is to the left of the centreline) and the reference heading for a car position.
"""
import os
from dataclasses import dataclass

import numpy as np

DS = 1.0                      # centreline sampling [m]
TRACK_WIDTH = 10.0            # full track width [m]
IMOLA_LENGTH = 4909.0         # lap length printed on the official map [m]
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "imola_centerline.csv")


@dataclass
class Track:
    name: str
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray            # heading [rad], unwrapped
    kappa: np.ndarray          # curvature [1/m]
    length: float
    width: float = TRACK_WIDTH

    @property
    def n(self):
        return len(self.s)

    def lookup(self, s):
        """Interpolated (x, y, psi, kappa) at arc length s (periodic)."""
        s = s % self.length
        i = int(s // DS) % self.n
        j = (i + 1) % self.n
        f = (s - self.s[i]) / DS
        psi_j = self.psi[j] if j != 0 else self.psi[0] + 2 * np.pi * round((self.psi[-1] - self.psi[0]) / (2 * np.pi))
        return (self.x[i] + f * (self.x[j] - self.x[i]), self.y[i] + f * (self.y[j] - self.y[i]),
                self.psi[i] + f * (psi_j - self.psi[i]), self.kappa[i] + f * (self.kappa[j] - self.kappa[i]))

    def project(self, px, py, hint=0, back=20, ahead=80):
        """Nearest centreline sample in a window around ``hint`` (index).  Returns
        (index, s, e, psi_ref, kappa_ref) with e the signed lateral error (left positive)."""
        n = self.n
        best, bi = 1e18, hint
        for k in range(-back, ahead + 1):
            i = (hint + k) % n
            d = (self.x[i] - px) ** 2 + (self.y[i] - py) ** 2
            if d < best:
                best, bi = d, i
        i = bi
        c, sn = np.cos(self.psi[i]), np.sin(self.psi[i])
        dx, dy = px - self.x[i], py - self.y[i]
        e = -dx * sn + dy * c
        s = self.s[i] + dx * c + dy * sn
        return i, s, e, self.psi[i], self.kappa[i]


# ------------------------------------------------------------------ construction helpers
def _finish(name, x, y, psi, kappa, width=TRACK_WIDTH):
    s = np.arange(len(x)) * DS
    return Track(name, s, np.asarray(x), np.asarray(y), np.unwrap(psi), np.asarray(kappa), len(x) * DS, width)


def circle(radius=100.0):
    n = int(round(2 * np.pi * radius / DS))
    t = np.arange(n) / n * 2 * np.pi
    return _finish("circle", radius * np.cos(t), radius * np.sin(t), t + np.pi / 2, np.full(n, 1.0 / radius))


def rounded_polygon(name, vertices, radius):
    """Counter-clockwise polygon whose corners are replaced by tangent arcs of the given radius."""
    V = np.asarray(vertices, dtype=float)
    m = len(V)
    segs = []                                              # list of (kind, params)
    tan_len = []
    for i in range(m):
        a, b, c = V[i - 1], V[i], V[(i + 1) % m]
        u, w = (b - a) / np.linalg.norm(b - a), (c - b) / np.linalg.norm(c - b)
        turn = np.arctan2(u[0] * w[1] - u[1] * w[0], u @ w)           # >0 left turn
        if turn <= 0:
            raise ValueError("vertices must form a convex counter-clockwise polygon")
        tan_len.append(radius * np.tan(turn / 2))
    xs, ys, ps, ks = [], [], [], []
    for i in range(m):
        a, b, c = V[i], V[(i + 1) % m], V[(i + 2) % m]
        u = (b - a) / np.linalg.norm(b - a)
        w = (c - b) / np.linalg.norm(c - b)
        # straight part of edge a->b
        p0 = a + u * tan_len[i]
        p1 = b - u * tan_len[(i + 1) % m]
        L = np.linalg.norm(p1 - p0)
        n = max(int(round(L / DS)), 1)
        for k in range(n):
            q = p0 + u * (L * k / n)
            xs.append(q[0]); ys.append(q[1]); ps.append(np.arctan2(u[1], u[0])); ks.append(0.0)
        # arc at vertex b
        turn = np.arctan2(u[0] * w[1] - u[1] * w[0], u @ w)
        centre = p1 + radius * np.array([-u[1], u[0]])
        a0 = np.arctan2(p1[1] - centre[1], p1[0] - centre[0])
        na = max(int(round(radius * turn / DS)), 1)
        for k in range(na):
            ang = a0 + turn * k / na
            xs.append(centre[0] + radius * np.cos(ang)); ys.append(centre[1] + radius * np.sin(ang))
            ps.append(ang + np.pi / 2); ks.append(1.0 / radius)
    xs, ys = np.array(xs), np.array(ys)
    # the loop above is not exactly DS-uniform; resample on true arc length so that index == s / DS
    d = np.hypot(np.diff(np.r_[xs, xs[0]]), np.diff(np.r_[ys, ys[0]]))
    sc = np.r_[0, np.cumsum(d)]
    L = sc[-1]
    n = int(round(L / DS))
    S = np.arange(n) * (L / n)
    ps = np.unwrap(np.array(ps))
    xc, yc = np.r_[xs, xs[0]], np.r_[ys, ys[0]]                       # close the loop
    pc, kc = np.r_[ps, ps[0] + 2 * np.pi], np.r_[ks, ks[0]]
    return _finish(name, np.interp(S, sc, xc), np.interp(S, sc, yc), np.interp(S, sc, pc), np.interp(S, sc, kc))


def square(side=300.0, corner_radius=40.0):
    h = side / 2
    return rounded_polygon("square", [(-h, -h), (h, -h), (h, h), (-h, h)], corner_radius)


def trapezoid(bottom=400.0, top=200.0, height=200.0, corner_radius=40.0):
    """Isosceles trapezoid (long base at the bottom)."""
    return rounded_polygon("trapezoid", [(-bottom / 2, 0), (bottom / 2, 0), (top / 2, height), (-top / 2, height)],
                           corner_radius)


def imola(smoothing=0.8):
    """Imola centreline traced from the official track map (see README, 'Imola data')."""
    from scipy.interpolate import splev, splprep
    xy = np.loadtxt(DATA, delimiter=",", skiprows=1)
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=len(xy) * smoothing ** 2, per=True, k=5)
    u = np.linspace(0, 1, 20000, endpoint=False)
    x, y = splev(u, tck)
    d = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))
    sc = np.r_[0, np.cumsum(d)]
    scale = IMOLA_LENGTH / sc[-1]                                        # keep the printed lap length
    n = int(round(IMOLA_LENGTH / DS))
    S = np.arange(n) * (IMOLA_LENGTH / n) / scale
    uu = np.interp(S, sc[:-1], u)
    x, y = splev(uu, tck)
    dx, dy = splev(uu, tck, der=1)
    ddx, ddy = splev(uu, tck, der=2)
    kappa = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5 / scale
    return _finish("imola", np.asarray(x) * scale, np.asarray(y) * scale, np.arctan2(dy, dx), kappa)


TRACKS = {"circle": circle, "square": square, "trapezoid": trapezoid, "imola": imola}


def get_track(name):
    try:
        return TRACKS[name]()
    except KeyError:
        raise ValueError(f"unknown track '{name}', choose one of {list(TRACKS)}") from None
