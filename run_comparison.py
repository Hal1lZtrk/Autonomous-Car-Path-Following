"""All tracks x both controllers x both vehicle models -> results table and comparison figure."""
import json
import os

from pathfollow import plotting as P
from pathfollow.simulate import simulate_lap
from pathfollow.tracks import TRACKS, get_track

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
rows = []
for tn in TRACKS:
    tr = get_track(tn)
    for c in ("pure_pursuit", "stanley"):
        for m in ("kinematic", "dynamic"):
            r = simulate_lap(tr, c, m)
            rows.append(dict(track=tn, controller=c, model=m, metrics=r.metrics))
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "comparison.json"), "w") as f:
    json.dump(rows, f, indent=2)
lines = ["| track | controller | model | lap [s] | RMS e [m] | max e [m] | off-track [%] | max a_y [m/s²] |",
         "|---|---|---|---|---|---|---|---|"]
for r in rows:
    m = r["metrics"]
    lines.append(f"| {r['track']} | {r['controller']} | {r['model']} | {m['lap_time_s']:.1f} | {m['rms_cross_track_m']:.2f} | "
                 f"{m['max_cross_track_m']:.2f} | {m['time_off_track_pct']:.1f} | {m['max_lateral_accel_ms2']:.1f} |")
open(os.path.join(OUT, "comparison.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
P.plot_comparison(rows, os.path.join(OUT, "figures", "comparison.png"))
P.show()
