"""Figures.  Saved as PNG in ``results/figures`` and, by default, also opened in windows
(``--no-show`` or ``PATHFOLLOW_NO_SHOW=1`` for headless runs)."""
import os
import sys

import matplotlib

SHOW = not (os.environ.get("PATHFOLLOW_NO_SHOW") or "--no-show" in sys.argv)
if SHOW and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    SHOW = False
if not SHOW:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

COLORS = {"pure_pursuit": "#d1495b", "stanley": "#1f5fbf"}
LABELS = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley"}
plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False,
                     "axes.spines.right": False, "font.size": 10, "figure.dpi": 100})


def _finish(fig, path):
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=140)
    if not SHOW:
        plt.close(fig)


def show():
    if SHOW and plt.get_fignums():
        plt.show()


def track_edges(track):
    nx, ny = -np.sin(track.psi), np.cos(track.psi)
    h = track.width / 2
    return (track.x + h * nx, track.y + h * ny), (track.x - h * nx, track.y - h * ny)


def draw_track(ax, track):
    (xl, yl), (xr, yr) = track_edges(track)
    ax.fill(np.r_[xl, xl[0], xr[::-1], xr[0]], np.r_[yl, yl[0], yr[::-1], yr[0]], color="0.88", lw=0)
    for xe, ye in ((xl, yl), (xr, yr)):
        ax.plot(np.r_[xe, xe[0]], np.r_[ye, ye[0]], color="0.35", lw=0.8)
    ax.plot(np.r_[track.x, track.x[0]], np.r_[track.y, track.y[0]], color="0.6", lw=0.6, ls="--")
    ax.plot([xl[0], xr[0]], [yl[0], yr[0]], color="k", lw=2.5)                       # start line
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def plot_overview(tracks, path):
    fig, axs = plt.subplots(1, len(tracks), figsize=(4.2 * len(tracks), 4.4))
    for ax, tr in zip(np.atleast_1d(axs), tracks):
        draw_track(ax, tr)
        ax.set_title(f"{tr.name}  ({tr.length:.0f} m)")
    fig.suptitle("The four tracks (counter-clockwise, black bar = start line, width 10 m)")
    _finish(fig, path)


def plot_trajectory(runs, path):
    """Driven line on the track; colour = speed (first run) and optional second run overlay."""
    tr = runs[0].track
    fig, ax = plt.subplots(figsize=(8, 7))
    draw_track(ax, tr)
    r = runs[0]
    pts = np.c_[r.x, r.y].reshape(-1, 1, 2)
    lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap="viridis", linewidth=2.6)
    lc.set_array(r.v[:-1])
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, shrink=0.7, label="speed [m/s]")
    ax.set_title(f"{tr.name}: {LABELS[r.controller]}, {r.model} model — lap {r.metrics['lap_time_s']:.1f} s, "
                 f"max error {r.metrics['max_cross_track_m']:.2f} m")
    _finish(fig, path)


def plot_signals(runs, path):
    tr = runs[0].track
    fig, ax = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    for r in runs:
        c, lab = COLORS[r.controller], f"{LABELS[r.controller]} ({r.model})"
        ax[1].plot(r.s, r.e, color=c, label=lab)
        ax[2].plot(r.s, np.degrees(r.delta), color=c)
        ax[3].plot(r.s, r.ay, color=c)
        ax[4].plot(r.s, r.e_psi * 180 / np.pi, color=c)
    r0 = runs[0]
    ax[0].plot(r0.s, r0.v_ref, color="k", ls="--", label="planned speed")
    for r in runs:
        ax[0].plot(r.s, r.v, color=COLORS[r.controller])
    ax[0].set_ylabel("speed [m/s]")
    ax[0].legend(ncol=3)
    ax[1].axhline(tr.width / 2, color="r", lw=0.8)
    ax[1].axhline(-tr.width / 2, color="r", lw=0.8)
    ax[1].set_ylabel("cross-track error [m]")
    ax[1].legend(ncol=2)
    ax[2].set_ylabel("steering [deg]")
    ax[3].set_ylabel("lateral accel. [m/s²]")
    ax[4].set_ylabel("heading error [deg]")
    ax[4].set_xlabel("distance driven [m]")
    ax[0].set_title(f"Track '{tr.name}': signals along the lap (red lines = track edges)")
    _finish(fig, path)


def plot_comparison(rows, path):
    """rows: list of dicts with track, controller, model, metrics."""
    tracks = list(dict.fromkeys(r["track"] for r in rows))
    combos = list(dict.fromkeys((r["controller"], r["model"]) for r in rows))
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    w = 0.8 / len(combos)
    for j, (c, m) in enumerate(combos):
        vals_rms = [next(r["metrics"]["rms_cross_track_m"] for r in rows if (r["track"], r["controller"], r["model"]) == (t, c, m)) for t in tracks]
        vals_max = [next(r["metrics"]["max_cross_track_m"] for r in rows if (r["track"], r["controller"], r["model"]) == (t, c, m)) for t in tracks]
        hatch = "//" if m == "dynamic" else None
        x = np.arange(len(tracks)) + j * w
        ax[0].bar(x, vals_rms, w, color=COLORS[c], hatch=hatch, edgecolor="w", label=f"{LABELS[c]} / {m}")
        ax[1].bar(x, vals_max, w, color=COLORS[c], hatch=hatch, edgecolor="w")
    for a in ax:
        a.set_xticks(np.arange(len(tracks)) + 0.4 - w / 2)
        a.set_xticklabels(tracks)
    ax[0].set_ylabel("RMS cross-track error [m]")
    ax[1].set_ylabel("max cross-track error [m]")
    ax[1].axhline(5, color="r", lw=0.8)
    ax[1].text(0, 5.1, "track edge", color="r", fontsize=8)
    ax[0].legend(fontsize=8)
    fig.suptitle("Tracking error, all tracks and controllers")
    _finish(fig, path)
