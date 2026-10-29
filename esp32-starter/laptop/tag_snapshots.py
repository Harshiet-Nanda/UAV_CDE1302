"""
Save a camera picture for every AprilTag detection during a flight.

The ESP32 has no storage of its own, but it streams JPEG frames to whichever
laptop sends it a keepalive (UDP 5009, see camera_stream.py). This keeps the
last few seconds of frames, and each time tag_logger.py places a detection it
picks the frame taken closest to that moment and saves it:

    logs/flight_<time>_snaps/
        12.34s_tag3.jpg        annotated: tag outline + ID + where the tag is
                               from the start (green = used, orange = rejected)
        raw/12.34s_tag3.jpg    the untouched JPEG from the drone
        best_tag3.jpg          the closest accepted view of each tag
        snapshots.csv          index: time, file, tags, frame timing

Matching: the detection's capture time comes from tag_logger; a frame's is
its arrival time minus the age the drone stamps on it. The drone detects on
its own frames, so the saved picture is the nearest streamed frame, normally
within ~50 ms (a few cm at flight speed); frame_dt_ms in the index says how
close it was.

Streaming adds ~60-80 KB/s of Wi-Fi traffic. The firmware streams from its
lowest-priority task on the Wi-Fi core and the README says it never affects
flight, but --no-snaps turns it off if you'd rather not. Only one program can
use UDP 5009: close camera_stream.py / tag_stream.py first.
"""

from __future__ import annotations

import csv
import math
import select
import shutil
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

from camera_stream import COMMAND_PORT, KEEPALIVE_S, FrameAssembler, drain_socket
from protocol import CAMERA_STREAM_PORT, build_camera_stream_command

BUFFER_S = 5.0          # keep this much video (tag_logger places tags 1.5 s late)
MATCH_S = 0.35          # give up if no frame was taken this close to the detection
SCALE = 2               # saved pictures are 640x480 so labels are readable


class SnapshotSaver:
    """Usage, next to a TagLogger:
        snaps = SnapshotSaver(flog.path, drone_id, comms, t0=flog.t0)
        snaps.start()
        tags = TagLogger(..., on_place=snaps.on_detection)
        ...
        tags.close(); snaps.close()
    """

    def __init__(self, flight_log_path: str, drone_id: int, comms,
                 t0: Optional[float] = None, fps: int = 10, quality: int = 60,
                 origin_fn=None) -> None:
        stem = Path(flight_log_path).with_suffix("")
        self.dir = Path(f"{stem}_snaps")
        self.drone_id = drone_id
        self.comms = comms
        self.t0 = time.monotonic() if t0 is None else t0
        self.keepalive = build_camera_stream_command(True, fps, quality)
        self.origin_fn = origin_fn or (lambda: (0.0, 0.0))
        self._frames: deque = deque()          # (capture time, frame_id, jpeg)
        self._lock = threading.Lock()
        self._running = False
        self._ip: Optional[str] = None
        self._rx: Optional[socket.socket] = None
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread: Optional[threading.Thread] = None
        self._index = None
        self._writer = None
        self._best: dict[int, tuple[float, Path]] = {}   # tag -> (range, file)
        self._n_saved = self._n_missed = self._n_frames = 0
        self._detector = None
        self._cv2 = self._np = None

    # -- lifecycle --------------------------------------------------------
    def start(self) -> bool:
        try:
            import cv2
            import numpy as np
            self._cv2, self._np = cv2, np
        except ImportError:
            print("[snaps] OpenCV missing; not saving pictures.")
            return False
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
            rx.setblocking(False)
        except OSError as exc:
            print(f"[snaps] UDP {CAMERA_STREAM_PORT} unavailable ({exc}); not saving "
                  f"pictures. Close camera_stream.py / tag_stream.py and retry.")
            return False
        try:                       # optional: outline tags on the saved pictures
            from apriltag_host import Detector
            self._detector = Detector()
        except Exception as exc:
            print(f"[snaps] tag outlines off ({exc}); pictures get centre markers only.")
        self._rx = rx
        (self.dir / "raw").mkdir(parents=True, exist_ok=True)
        self._index = open(self.dir / "snapshots.csv", "w", newline="")
        self._writer = csv.writer(self._index)
        self._writer.writerow(["t_s", "image", "raw_image", "frame_dt_ms", "tag_ids",
                               "used", "tags_from_start"])
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="snapshots", daemon=True)
        self._thread.start()
        print(f"[snaps] requesting the drone's camera; pictures go to {self.dir}")
        return True

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ip:
            try:
                self._tx.sendto(build_camera_stream_command(False), (self._ip, COMMAND_PORT))
            except OSError:
                pass
        if self._rx:
            self._rx.close()
        with self._lock:
            for tid, (_, path) in self._best.items():
                shutil.copyfile(path, self.dir / f"best_tag{tid}.jpg")
            self._index.close()
        if self._detector is not None:
            self._detector.close()
        print(f"[snaps] {self._n_saved} pictures saved to {self.dir} "
              f"({self._n_missed} detections had no frame close enough; "
              f"{self._n_frames} frames received)")

    # -- video ------------------------------------------------------------
    def _loop(self) -> None:
        asm = FrameAssembler()
        next_keepalive = 0.0
        while self._running:
            now = time.monotonic()
            if self._ip is None:
                self._ip = self.comms.known_drones().get(self.drone_id)
            if self._ip and now >= next_keepalive:
                next_keepalive = now + KEEPALIVE_S
                try:
                    self._tx.sendto(self.keepalive, (self._ip, COMMAND_PORT))
                except OSError:
                    pass
            if not self._ip:
                time.sleep(0.1)
                continue
            select.select([self._rx], [], [], 0.05)
            try:
                frame, _, _ = drain_socket(self._rx, asm, self._ip)
            except OSError:
                break
            asm.expire(time.monotonic())
            if frame is None:
                continue
            t_cap = time.monotonic() - frame.age_ms / 1000.0
            with self._lock:
                self._n_frames += 1
                self._frames.append((t_cap, frame.frame_id, frame.jpeg))
                while self._frames and t_cap - self._frames[0][0] > BUFFER_S:
                    self._frames.popleft()

    # -- detections (called by TagLogger) -----------------------------------
    def on_detection(self, t_cap: float, pkt, rows: list[dict]) -> None:
        if not self._running or not rows:
            return
        with self._lock:
            best = min(self._frames, key=lambda f: abs(f[0] - t_cap), default=None)
            if best is None or abs(best[0] - t_cap) > MATCH_S:
                self._n_missed += 1
                return
            frame_t, frame_id, jpeg = best
            t_s = t_cap - self.t0
            ids = sorted({r["tag_id"] for r in rows})
            name = f"{t_s:07.2f}s_tag{'-'.join(map(str, ids))}.jpg"
            (self.dir / "raw" / name).write_bytes(jpeg)
            path = self.dir / name
            self._annotate(jpeg, pkt, rows, t_s, (frame_t - t_cap) * 1000, path)
            ox, oy = self.origin_fn()
            where = ";".join(f"{r['tag_id']}:{r['tag_x'] - ox:+.2f},{r['tag_y'] - oy:+.2f}"
                             for r in rows if r["tag_x"] != "")
            self._writer.writerow([f"{t_s:.2f}", name, f"raw/{name}",
                                   f"{(frame_t - t_cap) * 1000:+.0f}",
                                   " ".join(map(str, ids)),
                                   " ".join(str(r["used"]) for r in rows), where])
            self._index.flush()
            self._n_saved += 1
            for r in rows:
                if r["used"] and r["range_m"] < self._best.get(r["tag_id"], (math.inf,))[0]:
                    self._best[r["tag_id"]] = (r["range_m"], path)

    def _annotate(self, jpeg, pkt, rows, t_s, dt_ms, path) -> None:
        cv2, np = self._cv2, self._np
        gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                          (gray.shape[1] * SCALE, gray.shape[0] * SCALE),
                          interpolation=cv2.INTER_NEAREST)
        font = cv2.FONT_HERSHEY_SIMPLEX

        def text(img, s, xy, colour, size=0.5):
            """Text on a solid dark box (readable on any background)."""
            (w, h), base = cv2.getTextSize(s, font, size, 1)
            x, y = xy
            x = max(0, min(x, img.shape[1] - w - 4))
            cv2.rectangle(img, (x - 2, y - h - 3), (x + w + 2, y + base), (20, 20, 20), -1)
            cv2.putText(img, s, (x, y), font, size, colour, 1, cv2.LINE_AA)

        ids = {r["tag_id"] for r in rows}
        if self._detector is not None:                     # outlines from the laptop
            try:
                for d in self._detector.detect(gray):
                    if d.id in ids:
                        pts = [(round(x * SCALE - 0.5), round(y * SCALE - 0.5))
                               for x, y in d.corners]
                        cv2.polylines(view, [np.array(pts, np.int32)], True,
                                      (0, 255, 0), 2, cv2.LINE_AA)
            except Exception:
                pass
        ox, oy = self.origin_fn()
        dets = {d.id: d for d in pkt.detections if d.pose_err >= 0}
        placed_boxes: list[tuple[int, int, int, int]] = []   # labels already drawn

        def free_spot(x0, y0, w, h):
            """Move a label down until it doesn't cover one already drawn."""
            for _ in range(20):
                if not any(x0 < bx1 and x0 + w > bx0 and y0 - 14 < by1 and y0 + h > by0
                           for bx0, by0, bx1, by1 in placed_boxes):
                    break
                y0 += h + 4
            placed_boxes.append((x0, y0 - 14, x0 + w, y0 + h))
            return x0, y0

        # accepted sightings first, so they keep the spot next to their tag
        for r in sorted(rows, key=lambda r: -r["used"]):    # drone's own detections
            d = dets.get(r["tag_id"])
            if d is None:
                continue
            c = (round(d.cx * SCALE), round(d.cy * SCALE))
            colour = (0, 220, 0) if r["used"] else (0, 140, 255)
            cv2.circle(view, c, 6, colour, 2, cv2.LINE_AA)
            # line 1: same as tag_stream.py (tag relative to the drone)
            # line 2: tag in the flight frame, measured from the start point
            line1 = (f"{r['tag_id']}  fwd {r['fwd_m']:.2f} right {r['right_m']:+.2f} "
                     f"down {r['down_m']:.2f} m")
            line2 = (f"T{r['tag_id']} at ({r['tag_x'] - ox:+.2f}, {r['tag_y'] - oy:+.2f}) m"
                     if r["tag_x"] != "" else f"T{r['tag_id']} position unknown")
            if not r["used"]:
                line2 += f"  REJECTED: {r['reject']}"
            x0, y0 = c[0] - 20, max(64, c[1] - 30)          # stay below the banner
            w = max(cv2.getTextSize(s_, font, 0.5, 1)[0][0] for s_ in (line1, line2))
            x0 = max(0, min(x0, view.shape[1] - w - 4))
            x0, y0 = free_spot(x0, y0, w, 18)
            text(view, line1, (x0, y0), colour)
            text(view, line2, (x0, y0 + 18), colour)
        # banner: where the drone was when this frame was taken
        cv2.rectangle(view, (0, 0), (view.shape[1], 26), (0, 0, 0), -1)
        pose = next((r for r in rows if r["drone_x"] != ""), None)
        if pose:
            banner = (f"drone at ({pose['drone_x'] - ox:+.2f}, {pose['drone_y'] - oy:+.2f}) m"
                      f"  heading {math.degrees(pose['heading_rad']):.0f} deg"
                      f"   [x fwd/north, y right/east, from start]")
        else:
            banner = "drone position unknown at this time"
        text(view, banner, (8, 18), (255, 255, 255), 0.45)
        text(view, f"t={t_s:.2f}s  frame {dt_ms:+.0f} ms from detection",
             (8, view.shape[0] - 10), (255, 255, 255), 0.45)
        cv2.imwrite(str(path), view, [cv2.IMWRITE_JPEG_QUALITY, 90])
