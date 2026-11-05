"""
AprilTag detection logger: turns the drone's own tag detections into
positions in the flight's local frame, logs every sighting, and fuses repeat
sightings of each tag into one estimate.

Source: the drone detects tags itself (~2 Hz, raw greyscale, the firmware's
quality gate) and sends every frame's detections to this laptop on UDP 5008
(protocol.parse_at_debug). Nothing has to be switched on.

For each detection:
  1. Capture time. The packet stamps the drone clock when processing ended
     (frame_ms) and how long it took (proc_ms). The drone-to-laptop clock
     offset is the minimum of (arrival - frame_ms) seen so far, so
     capture = frame_ms + offset - proc_ms on the laptop clock.
  2. Drone pose at that time, interpolated from telemetry. Only packets
     where the position actually changed are used as samples, so the 1 Hz
     PX4 position stream is interpolated instead of held stale.
  3. Camera -> drone frame with apriltag_host.camera_to_body() (the same 45°
     mount the rest of the laptop code uses; edit CAM_PITCH_DEG there when
     the camera is remounted), then drone -> local NED using the heading.
     Roll/pitch are not in telemetry and are ignored (a few cm at 0.5 m).
  4. A second, independent estimate: the ray through the tag's pixel centre
     hits the floor at the assumed flying height. It ignores tag size, so if
     the two estimates disagree by a constant factor, the printed tag size
     (TAG_SIZE_M = 0.12) or the height assumption is wrong.

Fusion per tag (a static landmark): drop sightings > OUTLIER_M from the
median, then a weighted mean with weight 1/range^2, since pose error grows
with distance. That is what a Kalman filter with range-dependent noise
reduces to for a tag that does not move. A tag counts as confirmed once it
has MIN_SIGHTINGS good sightings from different frames; tag16h5 throws false
positives, so single sightings are not trusted.

Files (next to the flight log, same t_s clock):
    flight_<time>_tags.csv          one row per detection with a pose, in the
                                    flight log's raw frame
    flight_<time>_tag_summary.csv   one row per tag, written by close(); has
                                    x/y from the start point and raw x/y
plot_trajectory.py shifts everything by the flight's start position.
"""

from __future__ import annotations

import csv
import json
import math
import socket
import threading
import time
from bisect import bisect_left
from pathlib import Path
from typing import Optional

from apriltag_host import CAM_POS_M, FIRMWARE_INTRINSICS, R_BODY_CAM, camera_to_body
from protocol import AT_DEBUG_PORT, TelemetryPacket, parse_at_debug

# --- quality / fusion settings ---------------------------------------------
ALTITUDE_M      = 0.5    # assumed drone height above the floor (CRUISE_ALT_M)
FLOOR_TOL_M     = 0.25   # reject if the tag isn't this close to floor level
MAX_RANGE_M     = 1.5    # reject sightings further than this
MAX_POSE_ERR    = 0.5    # same limit the firmware uses to trust a pose
REQUIRE_HAMMING0 = True  # tag16h5: only error-free decodes count
OUTLIER_M       = 0.30   # drop sightings this far from the tag's median
MIN_SIGHTINGS   = 2      # sightings needed to call a tag confirmed
PROCESS_DELAY_S = 1.5    # wait for the next 1 Hz position before placing a tag

CSV_FIELDS = ["t_s", "frame_ms", "tag_id", "hamming", "margin", "pose_err",
              "fwd_m", "right_m", "down_m", "range_m",
              "drone_x", "drone_y", "heading_rad",
              "tag_x", "tag_y", "ray_x", "ray_y", "used", "reject"]


# ---------------------------------------------------------------------------
# Drone pose history
# ---------------------------------------------------------------------------

class PoseHistory:
    """Telemetry positions/headings on the laptop clock, interpolated."""

    def __init__(self, drone_id: int) -> None:
        self.drone_id = drone_id
        self._lock = threading.Lock()
        self._pt: list[float] = []      # times where the position changed
        self._px: list[float] = []
        self._py: list[float] = []
        self._ht: list[float] = []      # heading samples (unwrapped)
        self._hv: list[float] = []
        self.last_rx = 0.0

    def feed(self, pkt: TelemetryPacket, _ip: str = "") -> None:
        if pkt.drone_id != self.drone_id:
            return
        now = time.monotonic()
        with self._lock:
            self.last_rx = now
            if not self._px or (pkt.ned_x, pkt.ned_y) != (self._px[-1], self._py[-1]):
                self._pt.append(now)
                self._px.append(pkt.ned_x)
                self._py.append(pkt.ned_y)
            h = pkt.heading_rad
            if self._hv:
                h = self._hv[-1] + math.remainder(h - self._hv[-1], 2 * math.pi)
            self._ht.append(now)
            self._hv.append(h)

    @staticmethod
    def _interp(ts, vs, t):
        i = bisect_left(ts, t)
        if i <= 0:
            return vs[0]
        if i >= len(ts):
            return vs[-1]
        k = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return vs[i - 1] + k * (vs[i] - vs[i - 1])

    def first(self) -> Optional[tuple[float, float]]:
        with self._lock:
            return (self._px[0], self._py[0]) if self._px else None

    def at(self, t: float) -> Optional[tuple[float, float, float]]:
        with self._lock:
            if not self._pt or t < self._pt[0] - 1.0 or t > self.last_rx + 0.5:
                return None
            return (self._interp(self._pt, self._px, t),
                    self._interp(self._pt, self._py, t),
                    self._interp(self._ht, self._hv, t))


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def body_to_local(x: float, y: float, psi: float, fwd: float, right: float):
    c, s = math.cos(psi), math.sin(psi)
    return x + fwd * c - right * s, y + fwd * s + right * c


def floor_ray_body(cx: float, cy: float, altitude: float):
    """Where the ray through pixel (cx, cy) meets the floor, drone frame
    (fwd, right), or None if it doesn't point down enough."""
    k = FIRMWARE_INTRINSICS
    d_cam = ((cx - k.cx) / k.fx, (cy - k.cy) / k.fy, 1.0)
    d = [sum(R_BODY_CAM[i][j] * d_cam[j] for j in range(3)) for i in range(3)]
    drop = altitude - CAM_POS_M[2]
    if d[2] < 0.05 or drop <= 0:
        return None
    s = drop / d[2]
    return CAM_POS_M[0] + s * d[0], CAM_POS_M[1] + s * d[1]


def fuse(rows: list[dict]) -> Optional[dict]:
    """Fuse one tag's used sightings: median outlier cut, 1/range^2 mean."""
    if not rows:
        return None
    xs = sorted(r["tag_x"] for r in rows)
    ys = sorted(r["tag_y"] for r in rows)
    mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
    keep = [r for r in rows if len(rows) < 3
            or math.hypot(r["tag_x"] - mx, r["tag_y"] - my) <= OUTLIER_M]
    w = [1.0 / max(r["range_m"], 0.2) ** 2 for r in keep]
    sw = sum(w)
    x = sum(wi * r["tag_x"] for wi, r in zip(w, keep)) / sw
    y = sum(wi * r["tag_y"] for wi, r in zip(w, keep)) / sw
    spread = math.sqrt(sum(wi * ((r["tag_x"] - x) ** 2 + (r["tag_y"] - y) ** 2)
                           for wi, r in zip(w, keep)) / sw)
    rays = [r for r in keep if r.get("ray_x") not in (None, "")]
    ray_x = sum(r["ray_x"] for r in rays) / len(rays) if rays else None
    ray_y = sum(r["ray_y"] for r in rays) / len(rays) if rays else None
    frames = {r["frame_ms"] for r in keep}
    return {"x": x, "y": y, "spread": spread, "n": len(keep), "n_all": len(rows),
            "frames": len(frames), "confirmed": len(frames) >= MIN_SIGHTINGS,
            "min_range": min(r["range_m"] for r in keep),
            "ray_x": ray_x, "ray_y": ray_y,
            "first_t": min(r["t_s"] for r in keep)}


def read_tag_csv(path: str) -> list[dict]:
    """Rows of a *_tags.csv with numbers parsed (for plotting / re-fusing)."""
    out = []
    p = Path(path)
    if not p.exists():
        return out
    with open(p, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                row = {k: (float(v) if v not in ("", None) and k != "reject" else v)
                       for k, v in r.items()}
                row["tag_id"] = int(row["tag_id"])
                row["used"] = int(row["used"])
                out.append(row)
            except (KeyError, ValueError):
                continue
    return out


def fuse_all(rows: list[dict]) -> dict[int, dict]:
    by_tag: dict[int, list[dict]] = {}
    for r in rows:
        if r["used"]:
            by_tag.setdefault(r["tag_id"], []).append(r)
    return {tid: f for tid, rs in sorted(by_tag.items()) if (f := fuse(rs))}


# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------

class TagLogger:
    """Listens on UDP 5008, places each detection, logs and fuses them.

    Usage (next to a FlightLogger, same comms node):
        poses = PoseHistory(drone_id)
        comms.on_telemetry(poses.feed)
        tags = TagLogger(flog.path, drone_id, poses, t0=flog.t0)
        tags.start()
        ...
        tags.close()      # flushes and writes the summary
    """

    def __init__(self, flight_log_path: str, drone_id: int, poses: PoseHistory,
                 t0: Optional[float] = None, altitude: float = ALTITUDE_M,
                 port: int = AT_DEBUG_PORT, verbose: bool = True,
                 on_place=None) -> None:
        stem = Path(flight_log_path).with_suffix("")
        self.status_path = Path(f"{stem}_detections.json")
        self.path = f"{stem}_tags.csv"
        self.summary_path = f"{stem}_tag_summary.csv"
        self.drone_id = drone_id
        self.poses = poses
        self.t0 = time.monotonic() if t0 is None else t0
        self.altitude = altitude
        self.port = port
        self.verbose = verbose
        self.on_place = on_place        # called (t_capture, packet, rows) per frame
        self.origin: Optional[tuple[float, float]] = None   # set by the mission;
        # otherwise the first telemetry position (the drone on the ground)
        self._offset: Optional[float] = None     # laptop - drone clock (s)
        self._last_frame: Optional[int] = None
        self._pending: list[tuple[float, object]] = []
        self._rows: list[dict] = []
        self._announced: set[int] = set()
        self._lock = threading.Lock()
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._file.flush()

    # -- lifecycle --------------------------------------------------------
    def start(self) -> bool:
        """Bind UDP 5008. Returns False (and logs nothing) if it's taken, e.g.
        by tag_stream_legacy.py --detail; the flight is unaffected."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("", self.port))
            s.settimeout(0.2)
        except OSError as exc:
            print(f"[tags] UDP {self.port} unavailable ({exc}); tags will NOT be logged. "
                  f"Close tag_stream_legacy.py / tag_debug.py and retry.")
            self._file.close()
            Path(self.path).unlink(missing_ok=True)     # don't leave an empty log
            return False
        self._sock = s
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="tag-logger", daemon=True)
        self._thread.start()
        print(f"[tags] listening for drone {self.drone_id} detections on UDP {self.port}; "
              f"logging to {self.path}")
        return True

    def _origin(self) -> tuple[float, float]:
        if self.origin is None:
            first = self.poses.first()
            return first if first else (0.0, 0.0)
        return self.origin

    def close(self) -> dict[int, dict]:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._sock:
            self._sock.close()
        with self._lock:
            self._process(flush=True)
            self._file.close()
        fused = fuse_all(self._rows)
        with open(self.summary_path, "w", newline="") as fh:
            w = csv.writer(fh)
            ox, oy = self._origin()
            w.writerow(["tag_id", "x_from_start_m", "y_from_start_m", "x_m", "y_m",
                        "spread_m", "sightings_used",
                        "sightings_all", "frames", "confirmed", "min_range_m",
                        "ray_x_m", "ray_y_m", "first_seen_t_s"])
            for tid, f in fused.items():
                w.writerow([tid, f"{f['x'] - ox:.3f}", f"{f['y'] - oy:.3f}",
                            f"{f['x']:.3f}", f"{f['y']:.3f}", f"{f['spread']:.3f}",
                            f["n"], f["n_all"], f["frames"], int(f["confirmed"]),
                            f"{f['min_range']:.2f}",
                            "" if f["ray_x"] is None else f"{f['ray_x']:.3f}",
                            "" if f["ray_y"] is None else f"{f['ray_y']:.3f}",
                            f"{f['first_t']:.2f}"])
        rejected = sum(1 for r in self._rows if not r["used"])
        print(f"[tags] {len(self._rows)} detections logged ({rejected} rejected), "
              f"{len(fused)} tag(s). Saved {self.path} and {self.summary_path}")
        return fused

    # -- receive / process -------------------------------------------------
    def _loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                data = None
            except OSError:
                break
            now = time.monotonic()
            with self._lock:
                if data:
                    pkt = parse_at_debug(data)
                    if pkt and pkt.drone_id == self.drone_id and pkt.frame_ms:
                        off = now - pkt.frame_ms / 1000.0
                        self._offset = off if self._offset is None else min(self._offset, off)
                        if pkt.frame_ms != self._last_frame:      # 10 Hz repeats
                            self._last_frame = pkt.frame_ms
                            self._publish_status(pkt)
                            if pkt.detections:
                                self._pending.append((now, pkt))
                self._process()

    def _publish_status(self, pkt) -> None:
        """Share metadata with the GUI; no images or laptop detector involved."""
        status = {"received_at": time.time(), "drone_id": pkt.drone_id,
                  "frame_ms": pkt.frame_ms, "proc_ms": pkt.proc_ms,
                  "raw_count": pkt.raw_count,
                  "detections": [vars(d) for d in pkt.detections]}
        temp = self.status_path.with_suffix(".tmp")
        try:
            temp.write_text(json.dumps(status))
            temp.replace(self.status_path)
        except OSError as exc:
            # A GUI status write failure must not stop detection CSV logging.
            print(f"[tags] onboard status unavailable: {exc}")

    def _process(self, flush: bool = False) -> None:
        now = time.monotonic()
        keep = []
        for rx, pkt in self._pending:
            if flush or now - rx >= PROCESS_DELAY_S:
                self._place(pkt)
            else:
                keep.append((rx, pkt))
        self._pending = keep
        self._file.flush()

    def _place(self, pkt) -> None:
        t_cap = pkt.frame_ms / 1000.0 + self._offset - pkt.proc_ms / 1000.0
        pose = self.poses.at(t_cap)
        placed = []
        for d in pkt.detections:
            if d.pose_err < 0:          # failed the firmware gate: likely noise
                continue
            fwd, right, down = camera_to_body((d.tx, d.ty, d.tz))
            rng = math.sqrt(d.tx ** 2 + d.ty ** 2 + d.tz ** 2)
            reject = ""
            if pose is None:
                reject = "no drone pose"
            elif REQUIRE_HAMMING0 and d.hamming != 0:
                reject = "hamming>0"
            elif d.pose_err > MAX_POSE_ERR:
                reject = "pose_err"
            elif rng > MAX_RANGE_M:
                reject = "too far"
            elif abs(down - self.altitude) > FLOOR_TOL_M:
                reject = "not at floor level"
            row = {"t_s": round(t_cap - self.t0, 3), "frame_ms": pkt.frame_ms,
                   "tag_id": d.id, "hamming": d.hamming, "margin": round(d.margin, 1),
                   "pose_err": round(d.pose_err, 4),
                   "fwd_m": round(fwd, 3), "right_m": round(right, 3),
                   "down_m": round(down, 3), "range_m": round(rng, 3),
                   "drone_x": "", "drone_y": "", "heading_rad": "",
                   "tag_x": "", "tag_y": "", "ray_x": "", "ray_y": "",
                   "used": int(not reject), "reject": reject}
            if pose is not None:
                x, y, psi = pose
                tx, ty = body_to_local(x, y, psi, fwd, right)
                row.update(drone_x=round(x, 3), drone_y=round(y, 3),
                           heading_rad=round(psi % (2 * math.pi), 4),
                           tag_x=round(tx, 3), tag_y=round(ty, 3))
                ray = floor_ray_body(d.cx, d.cy, self.altitude)
                if ray:
                    rx, ry = body_to_local(x, y, psi, *ray)
                    row.update(ray_x=round(rx, 3), ray_y=round(ry, 3))
            self._writer.writerow(row)
            self._rows.append(row)
            placed.append(row)
            if row["used"] and self.verbose:
                self._announce(d.id)
        if placed and self.on_place:
            try:                        # e.g. tag_snapshots; must never break logging
                self.on_place(t_cap, pkt, placed)
            except Exception as exc:
                print(f"[tags] snapshot callback failed: {exc}")

    def _announce(self, tid: int) -> None:
        f = fuse([r for r in self._rows if r["used"] and r["tag_id"] == tid])
        if f and f["confirmed"] and tid not in self._announced:
            self._announced.add(tid)
            ox, oy = self._origin()
            print(f"[tags] TAG {tid} confirmed at x={f['x'] - ox:+.2f} "
                  f"y={f['y'] - oy:+.2f} from the start ({f['frames']} sightings)")
