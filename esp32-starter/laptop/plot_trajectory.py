#!/usr/bin/env python3
"""
Plot a drone's 2D trajectory over time from telemetry.

Three ways to use it:

1. Live, next to a mission script (that script owns UDP 5005, so it logs via
   flight_log.FlightLogger and this tails the CSV):
       python3 laptop/plot_trajectory.py --file logs/flight_XXXX.csv --live

2. Live, on its own (no other script running — e.g. carry the drone by hand
   or watch a flight started from QGC). Listens on UDP 5005 and logs a CSV:
       python3 laptop/plot_trajectory.py --listen --drone-id 22

3. After the flight, from a saved log (optionally save a PNG):
       python3 laptop/plot_trajectory.py --file logs/flight_XXXX.csv --save run1.png

Overlay what you *asked* the drone to fly with --ref:
   --ref trajectory/ellipse_min_snap_traj.csv   (t,x,y,z; anchored where TRAJ began)
   --ref waypoints/waypoints_example.txt         (x,y per line; takeoff frame)

AprilTags: if the log has a matching flight_<time>_tags.csv (written by
tag_logger.py), each sighting is drawn as a small x and each tag's fused
position as a star labelled with its ID (hollow = not yet confirmed), with a
circle showing the spread of its sightings. --tags points at another file.

Axes: east (y) to the right, north (x) up, like a map seen from above.
Positions are shifted so the first one is (0, 0); --absolute shows PX4's raw values.
Left panel: path coloured by time, start ●, current/end ▲ with heading.
Right panels: x(t) and y(t), with nav-state changes marked.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

class LogTail:
    """Reads a FlightLogger CSV, picking up new rows on each poll()."""

    def __init__(self, path: str, drone_id: int | None, zero: bool = True):
        self.path = path
        self.drone_id = drone_id
        self.zero = zero           # shift so the first sample is (0, 0)
        self.origin = (0.0, 0.0)
        self._pos = 0
        self._header_done = False
        self.t, self.x, self.y, self.hdg, self.state = [], [], [], [], []

    def poll(self) -> bool:
        p = Path(self.path)
        if not p.exists():
            return False
        added = False
        with open(p, newline="") as f:
            f.seek(self._pos)
            while True:
                line = f.readline()
                if not line or not line.endswith("\n"):   # partial line: retry later
                    break
                self._pos = f.tell()
                if not self._header_done:
                    self._header_done = True
                    continue
                row = next(csv.reader([line]))
                if len(row) < 6:
                    continue
                did = int(row[1])
                if self.drone_id is None:
                    self.drone_id = did           # lock on to the first drone seen
                if did != self.drone_id:
                    continue
                self.t.append(float(row[0]))
                x, y = float(row[2]), float(row[3])
                if self.zero and not self.x:
                    self.origin = (x, y)
                self.x.append(x - self.origin[0])
                self.y.append(y - self.origin[1])
                self.hdg.append(float(row[4]))
                self.state.append(row[5])
                added = True
        return added


def load_reference(path: str) -> tuple[np.ndarray, np.ndarray, bool]:
    """Return (x, y, is_trajectory). Trajectory CSVs have a t column."""
    rows = []
    has_t = False
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [s.strip() for s in line.split(",")]
        try:
            vals = [float(v) for v in parts]
        except ValueError:                         # header row
            has_t = parts[0].lower() == "t"
            continue
        rows.append(vals)
    a = np.array(rows)
    if a.shape[1] >= 4:
        has_t = True
    if has_t:
        return a[:, 1], a[:, 2], True
    return a[:, 0], a[:, 1], False


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

class TrajectoryPlot:
    def __init__(self, title: str, ref=None):
        self.fig = plt.figure(figsize=(12, 6.5))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.25, 1])
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_x = self.fig.add_subplot(gs[0, 1])
        self.ax_y = self.fig.add_subplot(gs[1, 1], sharex=self.ax_x)
        self.fig.suptitle(title)
        self.ref = ref
        self.tags_path = None          # flight_<time>_tags.csv, drawn if present
        self._tag_artists = []
        self._tag_line = ""

        m = self.ax_map
        m.set_xlabel("y — east (m)")
        m.set_ylabel("x — north (m)")
        m.set_aspect("equal", adjustable="box")
        m.grid(True, alpha=0.3)
        self.ref_line, = m.plot([], [], "--", color="0.55", lw=1.2, label="reference")
        self.lc = LineCollection([], cmap="viridis", lw=2.2)
        m.add_collection(self.lc)
        self.start_pt, = m.plot([], [], "o", color="tab:green", ms=8, label="start")
        self.cur_pt, = m.plot([], [], "^", color="tab:red", ms=9, label="now / end")
        self.hdg_line, = m.plot([], [], "-", color="tab:red", lw=2)
        self.cbar = self.fig.colorbar(self.lc, ax=m, fraction=0.04, pad=0.02)
        self.cbar.set_label("time (s)")
        # legend entries for tags appear only once a tag has been drawn
        self._tag_legend = [m.plot([], [], "x", color="0.3", ms=5, label="_tag sighting")[0],
                            m.plot([], [], "*", color="0.3", ms=12, label="_tag estimate")[0]]
        m.legend(loc="upper right", fontsize=8)
        self.info = m.text(0.02, 0.02, "", transform=m.transAxes, fontsize=9,
                           family="monospace", va="bottom",
                           bbox=dict(boxstyle="round", fc="white", alpha=0.8))

        self.x_line, = self.ax_x.plot([], [], color="tab:blue", label="actual")
        self.y_line, = self.ax_y.plot([], [], color="tab:orange", label="actual")
        self.ax_x.set_ylabel("x north (m)")
        self.ax_y.set_ylabel("y east (m)")
        self.ax_y.set_xlabel("time (s)")
        for a in (self.ax_x, self.ax_y):
            a.grid(True, alpha=0.3)
        self._state_marks = []
        self.fig.tight_layout()

    def update(self, d: LogTail) -> None:
        if not d.t:
            return
        t = np.asarray(d.t)
        x = np.asarray(d.x)
        y = np.asarray(d.y)

        # Map view: plot east on the horizontal, north on the vertical
        pts = np.column_stack([y, x]).reshape(-1, 1, 2)
        if len(t) > 1:
            self.lc.set_segments(np.concatenate([pts[:-1], pts[1:]], axis=1))
            self.lc.set_array(t[:-1])
            self.lc.set_clim(t[0], max(t[-1], t[0] + 1e-3))
        self.start_pt.set_data([y[0]], [x[0]])
        self.cur_pt.set_data([y[-1]], [x[-1]])
        h = d.hdg[-1]                               # NED heading: 0 = north, +cw
        L = 0.25
        self.hdg_line.set_data([y[-1], y[-1] + L * math.sin(h)],
                               [x[-1], x[-1] + L * math.cos(h)])

        all_e, all_n = [y], [x]
        if self.ref is not None:
            rx, ry, is_traj = self.ref
            ox = oy = 0.0
            if is_traj:
                # send_trajectory flies the CSV relative to where the drone is
                # when it starts: anchor at the first TRAJ sample (else first sample)
                i0 = next((i for i, s in enumerate(d.state) if s == "TRAJ"), None)
                if i0 is not None:
                    ox, oy = x[i0] - rx[0], y[i0] - ry[0]
                else:
                    ox, oy = x[0] - rx[0], y[0] - ry[0]
            self.ref_line.set_data(ry + oy, rx + ox)
            if not is_traj:                          # waypoints: show each as a dot
                self.ref_line.set_marker("o")
                self.ref_line.set_label("waypoints")
                self.ax_map.legend(loc="upper right", fontsize=8)
            all_e.append(ry + oy)
            all_n.append(rx + ox)
        te, tn = self._draw_tags(d)
        all_e += te
        all_n += tn
        e = np.concatenate(all_e)
        n = np.concatenate(all_n)
        pad = 0.3
        span = max(e.max() - e.min(), n.max() - n.min()) + 2 * pad
        self.ax_map.set_xlim(e.min() - pad, e.max() + pad)
        # extra room at the bottom so the stats box never covers the path
        self.ax_map.set_ylim(n.min() - pad - 0.3 * span, n.max() + pad)

        # Distance flown and speed from the last two samples
        dist = float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else 0.0
        spd = 0.0
        if len(t) > 1 and t[-1] > t[-2]:
            spd = math.hypot(x[-1] - x[-2], y[-1] - y[-2]) / (t[-1] - t[-2])
        self.info.set_text(
            f"t   {t[-1]:7.2f} s   {d.state[-1]}\n"
            f"x   {x[-1]:+7.2f} m   y {y[-1]:+7.2f} m\n"
            f"hdg {math.degrees(h):+7.1f}°   v {spd:4.2f} m/s\n"
            f"flown {dist:6.2f} m   n={len(t)}" + self._tag_line)

        # Time series
        self.x_line.set_data(t, x)
        self.y_line.set_data(t, y)
        for a, v in ((self.ax_x, x), (self.ax_y, y)):
            a.set_xlim(t[0], max(t[-1], t[0] + 1))
            lo, hi = v.min(), v.max()
            a.set_ylim(lo - 0.2, hi + 0.2)

        # Vertical marks where the nav state changes (TRAJ, FLYING, ARRIVED, …)
        for m in self._state_marks:
            m.remove()
        self._state_marks = []
        for i in range(1, len(d.state)):
            if d.state[i] != d.state[i - 1]:
                for a in (self.ax_x, self.ax_y):
                    self._state_marks.append(a.axvline(t[i], color="0.7", lw=0.8, ls=":"))
                self._state_marks.append(self.ax_x.text(
                    t[i], 1.0, d.state[i], transform=self.ax_x.get_xaxis_transform(),
                    fontsize=7, rotation=90, va="top", ha="right", color="0.4"))


    def _draw_tags(self, d: LogTail) -> tuple[list, list]:
        """Draw tag sightings and fused estimates; returns extents for limits."""
        for a in self._tag_artists:
            a.remove()
        self._tag_artists = []
        self._tag_line = ""
        if not self.tags_path or not Path(self.tags_path).exists():
            return [], []
        try:
            from tag_logger import fuse_all, read_tag_csv
        except Exception as exc:                       # plotting must never fail
            self._tag_line = f"\ntags: can't load tag_logger ({exc})"
            return [], []
        rows = read_tag_csv(self.tags_path)
        ox, oy = d.origin
        m = self.ax_map
        cmap = plt.get_cmap("tab10")
        ext_e, ext_n = [], []
        used = [r for r in rows if r["used"]]
        for r in used:
            c = cmap(r["tag_id"] % 10)
            self._tag_artists += m.plot(r["tag_y"] - oy, r["tag_x"] - ox, "x",
                                        color=c, ms=5, alpha=0.7)
        fused = fuse_all(rows)
        for tid, f in fused.items():
            c = cmap(tid % 10)
            ex, ny = f["y"] - oy, f["x"] - ox
            self._tag_artists += m.plot(ex, ny, "*", ms=16, mec="k", mew=0.8,
                                        color=c if f["confirmed"] else "none")
            circ = plt.Circle((ex, ny), max(f["spread"], 0.03), color=c,
                              fill=False, ls="--", lw=1)
            m.add_patch(circ)
            self._tag_artists.append(circ)
            self._tag_artists.append(m.annotate(
                f"T{tid}", (ex, ny), xytext=(6, 6), textcoords="offset points",
                fontsize=9, fontweight="bold", color="k"))
            ext_e.append(np.array([ex]))
            ext_n.append(np.array([ny]))
        if fused and self._tag_legend[0].get_label().startswith("_"):
            for h in self._tag_legend:
                h.set_label(h.get_label()[1:])
            m.legend(loc="upper right", fontsize=8)
        if rows:
            n_conf = sum(f["confirmed"] for f in fused.values())
            self._tag_line = (f"\ntags {len(fused)} ({n_conf} confirmed), "
                              f"{len(used)}/{len(rows)} sightings used")
        return ext_e, ext_n


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="FlightLogger CSV to plot (or tail with --live)")
    src.add_argument("--listen", action="store_true",
                     help="Receive telemetry on UDP itself (only if no other script is running)")
    ap.add_argument("--live", action="store_true", help="Keep refreshing as the CSV grows")
    ap.add_argument("--drone-id", type=int, help="Drone to plot (default: first one seen)")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--ref", help="Reference trajectory CSV (t,x,y,z) or waypoints file (x,y)")
    ap.add_argument("--save", help="Save the final figure to this PNG and exit")
    ap.add_argument("--tags", help="Tag sightings CSV (default: <log>_tags.csv if it exists)")
    ap.add_argument("--no-tags", action="store_true",
                    help="With --listen: don't log AprilTag detections (UDP 5008)")
    ap.add_argument("--absolute", action="store_true",
                    help="Plot raw positions. Default: the first position is (0, 0)")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")

    ref = load_reference(args.ref) if args.ref else None

    comms = flog = tags = None
    if args.listen:
        # Imported here so --file works without the rest of laptop/ on the path
        sys.path.insert(0, str(Path(__file__).parent))
        from comms import CommsNode
        from flight_log import FlightLogger
        flog = FlightLogger(drone_id=args.drone_id)
        comms = CommsNode(listen_port=args.telem_port)
        comms.on_telemetry(flog.feed)
        if args.drone_id is not None and not args.no_tags:
            try:                     # tag logging is optional; never block the plot
                from tag_logger import PoseHistory, TagLogger
                poses = PoseHistory(args.drone_id)
                comms.on_telemetry(poses.feed)
                tags = TagLogger(flog.path, args.drone_id, poses, t0=flog.t0)
                if not tags.start():
                    tags = None
            except Exception as exc:
                print(f"[tags] not logging tags: {exc}")
                tags = None
        comms.start()
        path, live = flog.path, True
    else:
        path, live = args.file, args.live

    data = LogTail(path, args.drone_id, zero=not args.absolute)
    data.poll()
    title = f"Drone trajectory — {Path(path).name}"
    if not args.absolute:
        title += "  [start = (0, 0)]"
    plot = TrajectoryPlot(title, ref)
    plot.tags_path = args.tags or str(Path(path).with_suffix("")) + "_tags.csv"

    try:
        if live and not args.save:
            def tick(_frame):
                if data.poll():
                    plot.update(data)
                    plot.fig.suptitle(f"{title}  (drone {data.drone_id})")
                return []
            _anim = FuncAnimation(plot.fig, tick, interval=200, cache_frame_data=False)
            plt.show()
        else:
            if not data.t:
                print(f"No telemetry rows in {path}")
                return 1
            plot.update(data)
            plot.fig.suptitle(f"{title}  (drone {data.drone_id})")
            if args.save:
                plot.fig.savefig(args.save, dpi=130)
                print(f"saved {args.save}")
            else:
                plt.show()
    finally:
        if comms:
            comms.stop()
        if flog:
            flog.close()
        if tags:
            tags.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())