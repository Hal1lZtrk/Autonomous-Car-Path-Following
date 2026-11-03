"""Trace the racing line printed on a track map image (blue line) into a closed centreline.

This is how ``pathfollow/data/imola_centerline.csv`` was produced.  The map image itself is not
distributed with this repository.

    python tools/trace_map.py MAP.jpg --length 4909 --start-px 992 865 --out centerline.csv

Requires scikit-image (only for this script).  Steps: blue-pixel mask -> skeleton -> ordered loop
-> periodic smoothing spline -> counter-clockwise orientation -> scale to the printed lap
length -> start at the start line -> resample every 5 m.
"""
import argparse

import numpy as np
from PIL import Image
from scipy import ndimage as ndi
from scipy.interpolate import splev, splprep
from skimage.morphology import remove_small_holes, skeletonize


def trace(path, length_m, start_px, smooth_px=0.9, step_m=5.0):
    a = np.asarray(Image.open(path).convert("RGB")).astype(int)
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    mask = (B > 200) & (R < 90) & (G > 100) & (G < 190)               # the blue line
    lab, n = ndi.label(mask, structure=np.ones((3, 3)))
    sizes = ndi.sum(mask, lab, range(1, n + 1))
    mask = lab == (1 + int(np.argmax(sizes)))                          # largest component
    sk = skeletonize(remove_small_holes(mask, max_size=3000))
    nb = lambda m: np.where(m, ndi.convolve(m.astype(int), np.ones((3, 3), int), mode="constant") - 1, 0)
    for _ in range(60):                                                # prune spurs
        ends = nb(sk) == 1
        if not ends.any():
            break
        sk &= ~ends
    pts = set(map(tuple, np.argwhere(sk)))                             # (row, col)
    cur = min(pts)
    path, seen = [cur], {cur}
    while True:                                                        # walk the loop
        c = [(cur[0] + dy, cur[1] + dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)
             if (dy or dx) and (cur[0] + dy, cur[1] + dx) in pts and (cur[0] + dy, cur[1] + dx) not in seen]
        if not c:
            break
        c.sort(key=lambda q: abs(q[0] - cur[0]) + abs(q[1] - cur[1]))
        cur = c[0]
        path.append(cur)
        seen.add(cur)
    p = np.array(path, dtype=float)
    xy = np.c_[p[:, 1], -p[:, 0]]                                      # x right, y up
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=len(xy) * smooth_px ** 2, per=True, k=3)
    x, y = splev(np.linspace(0, 1, 20001), tck)
    d = np.c_[x, y]
    if 0.5 * np.sum(d[:-1, 0] * d[1:, 1] - d[1:, 0] * d[:-1, 1]) < 0:  # make it counter-clockwise
        d = d[::-1]
    tgt = np.array([start_px[0], -start_px[1]], dtype=float)
    d = np.roll(d, -int(np.argmin(np.hypot(d[:, 0] - tgt[0], d[:, 1] - tgt[1]))), axis=0)
    s = np.r_[0, np.cumsum(np.hypot(np.diff(d[:, 0]), np.diff(d[:, 1])))]
    k = length_m / s[-1]
    S = np.arange(0, length_m, step_m)
    xr, yr = np.interp(S, s * k, d[:, 0] * k), np.interp(S, s * k, d[:, 1] * k)
    return np.c_[xr - xr[0], yr - yr[0]], k


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--length", type=float, default=4909.0)
    ap.add_argument("--start-px", type=float, nargs=2, default=(992, 865))
    ap.add_argument("--out", default="centerline.csv")
    a = ap.parse_args()
    xy, k = trace(a.image, a.length, a.start_px)
    np.savetxt(a.out, xy, delimiter=",", header="x_m,y_m", comments="", fmt="%.3f")
    print(f"{len(xy)} points, {k:.4f} m/pixel -> {a.out}")
