"""Top-down animation of a lap (window, GIF or MP4)."""
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from . import plotting as P
from .vehicle import CAR


def animate(run, path=None, fps=25, speedup=None, show=False):
    plt = P.plt
    tr = run.track
    speedup = speedup or max(1.0, run.t[-1] / 14.0)                 # ~14 s clip
    frames = np.arange(0, len(run.t), max(1, int(round(speedup * fps * 0.05))))
    fig, ax = plt.subplots(figsize=(7.2, 6.6))
    P.draw_track(ax, tr)
    trail, = ax.plot([], [], color=P.COLORS[run.controller], lw=2)
    body, = ax.plot([], [], color="k", lw=3, solid_capstyle="round")
    pt, = ax.plot([], [], "o", color="orange", ms=6)
    txt = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", family="monospace")
    ax.set_title(f"{tr.name}: {P.LABELS[run.controller]}, {run.model} model")
    span = max(np.ptp(tr.x), np.ptp(tr.y))

    def update(k):
        i = frames[k]
        psi = run.psi[i]
        f = np.array([np.cos(psi), np.sin(psi)])
        a = np.array([run.x[i], run.y[i]]) - 0.0 * f
        b = a + f * CAR.wheelbase * max(1.0, span / 250)
        body.set_data([a[0], b[0]], [a[1], b[1]])
        trail.set_data(run.x[:i + 1], run.y[:i + 1])
        txt.set_text(f"t = {run.t[i]:5.1f} s\nv = {run.v[i]*3.6:5.0f} km/h\ne = {run.e[i]:+5.2f} m")
        return body, trail, txt

    ani = FuncAnimation(fig, update, frames=len(frames), interval=1000 / fps, blit=False)
    if path:
        if path.endswith(".mp4"):
            ani.save(path, fps=fps, dpi=110)
        else:
            ani.save(path, writer=PillowWriter(fps=fps), dpi=80)
    if show and P.SHOW:
        P.plt.show()
    else:
        P.plt.close(fig)
    return ani
