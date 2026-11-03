"""Drive one lap on a chosen track and show/save the figures.

    python run_simulation.py                     # asks which track to drive
    python run_simulation.py --track imola --controller stanley --model dynamic
    python run_simulation.py --track circle --no-show
"""
import argparse
import json
import os

from pathfollow import plotting as P
from pathfollow.simulate import simulate_lap
from pathfollow.tracks import TRACKS, get_track

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


def choose_track():
    names = list(TRACKS)
    print("Select a track:")
    for i, n in enumerate(names, 1):
        print(f"  {i}) {n}")
    while True:
        a = input("Number or name: ").strip().lower()
        if a in names:
            return a
        if a.isdigit() and 1 <= int(a) <= len(names):
            return names[int(a) - 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", choices=list(TRACKS))
    ap.add_argument("--controller", choices=["pure_pursuit", "stanley", "both"], default="both")
    ap.add_argument("--model", choices=["kinematic", "dynamic"], default="dynamic")
    ap.add_argument("--no-show", action="store_true", help="only write files")
    a = ap.parse_args()
    name = a.track or choose_track()
    track = get_track(name)
    ctrls = ["pure_pursuit", "stanley"] if a.controller == "both" else [a.controller]
    runs = []
    for c in ctrls:
        r = simulate_lap(track, c, a.model)
        runs.append(r)
        m = r.metrics
        print(f"{name:9s} {c:12s} {a.model:9s} lap {m['lap_time_s']:6.1f} s  rms {m['rms_cross_track_m']:.2f} m  "
              f"max {m['max_cross_track_m']:.2f} m  off-track {m['time_off_track_pct']:.1f} %  a_y,max {m['max_lateral_accel_ms2']:.1f} m/s²")
    fig = os.path.join(OUT, "figures")
    P.plot_trajectory(sorted(runs, key=lambda r: r.controller != "stanley")[:1], os.path.join(fig, f"{name}_trajectory.png"))
    P.plot_signals(runs, os.path.join(fig, f"{name}_signals.png"))
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{name}_{a.model}_metrics.json"), "w") as f:
        json.dump({r.controller: r.metrics for r in runs}, f, indent=2)
    P.show()


if __name__ == "__main__":
    main()
