"""Animation of a lap.   python run_animation.py --track imola [--gif | --mp4]"""
import argparse
import os

from pathfollow.animation import animate
from pathfollow.simulate import simulate_lap
from pathfollow.tracks import TRACKS, get_track

ap = argparse.ArgumentParser()
ap.add_argument("--track", choices=list(TRACKS) + ["all"], help="omit to choose interactively")
ap.add_argument("--controller", choices=["pure_pursuit", "stanley"], default="stanley")
ap.add_argument("--model", choices=["kinematic", "dynamic"], default="dynamic")
ap.add_argument("--gif", action="store_true")
ap.add_argument("--mp4", action="store_true")
ap.add_argument("--no-show", action="store_true")
a = ap.parse_args()
if a.track is None:
    names = list(TRACKS)
    print("Select a track:")
    for i, n in enumerate(names, 1):
        print(f"  {i}) {n}")
    while a.track is None:
        c = input("Number or name: ").strip().lower()
        a.track = c if c in names else names[int(c) - 1] if c.isdigit() and 1 <= int(c) <= len(names) else None
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(out, exist_ok=True)
ext = "mp4" if a.mp4 else "gif" if a.gif else None
for name in (list(TRACKS) if a.track == "all" else [a.track]):
    run = simulate_lap(get_track(name), a.controller, a.model)
    animate(run, os.path.join(out, f"{name}_{a.controller}.{ext}") if ext else None, show=not ext)
