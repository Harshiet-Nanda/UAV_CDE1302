#!/usr/bin/env python3
"""
Fly a CSV trajectory on one drone, optionally turn around and fly a return
pass, with a telemetry log, live plot, AprilTag log and pictures.

    python laptop/send_trajectory_logged.py --drone-id 7 --takeoff --takeoff-wait 15 \
        --confirm --trajectory trajectory/coverage_traj.csv \
        --return-trajectory trajectory/coverage_traj_return.csv

Same as send_trajectory.py, plus:
  - every telemetry packet -> logs/flight_<time>.csv, live plot, PNG at the end
  - AprilTags the drone sees -> logs/flight_<time>_tags.csv / _tag_summary.csv,
    drawn on the plot (tag_logger.py; --no-tags turns it off)
  - a picture of every detection -> logs/flight_<time>_snaps/ (tag_snapshots.py;
    --no-snaps turns the camera stream off)
  - --return-trajectory: after the first pass the drone turns 180 degrees and
    flies the second CSV (written by coverage_min_snap.py), lined up with the
    first pass, then lands.
Needs flight_log.py, plot_trajectory.py, tag_logger.py, tag_snapshots.py in
the same folder.

The turn uses CMD_TURN: fixed position, 15 deg/s maximum yaw speed with
10 deg/s² acceleration/deceleration. Requires updated ESP32 firmware.
The return pass starts only after confirmed completion. --turn-test performs
only takeoff, turn and landing; --no-turn skips rotation.
"""

import argparse
import logging
import math
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from comms import CommsNode
from flight_log import FlightLogger
from protocol import (
    CMD_GOTO,
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_ARRIVED,
    NAV_TRAJ,
    TRAJ_MAX_PTS,
    CommandPacket,
    build_traj_packets,
    build_traj_start,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("send_trajectory")

UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue
PLOT_SCRIPT = Path(__file__).with_name("plot_trajectory.py")


GLIDE_SPEED = 0.10     # m/s, peak speed of the glide into pass 2
GLIDE_MAX_M = 1.2      # refuse to glide further than this (something went wrong)


def _detached_kwargs() -> dict:
    """Own process group, so Ctrl+C (which lands the drone) keeps the plot open."""
    if sys.platform == "win32":
        return {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
    return {"start_new_session": True}


def start_live_plot(log_path: str, ref_path: str) -> None:
    try:
        subprocess.Popen([sys.executable, str(PLOT_SCRIPT), "--file", log_path,
                          "--live", "--ref", ref_path], **_detached_kwargs())
    except Exception as exc:                  # never let plotting stop a flight
        log.warning("Could not start live plot: %s", exc)


def save_plot_png(log_path: str, ref_path: str) -> None:
    png = str(Path(log_path).with_suffix(".png"))
    try:
        subprocess.run([sys.executable, str(PLOT_SCRIPT), "--file", log_path,
                        "--ref", ref_path, "--save", png],
                       timeout=60, check=True, capture_output=True)
        log.info("Trajectory plot saved to %s", png)
    except Exception as exc:
        log.warning("Could not save trajectory PNG: %s", exc)


def load_trajectory(path: str, dt: float) -> np.ndarray:
    """Read t,x,y,z, resample every dt, return offsets from the first point."""
    data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
    data = data[~np.isnan(data).any(axis=1)]           # drops the header row
    if data.shape[0] < 2 or data.shape[1] < 4:
        raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
    t = data[:, 0]
    if np.any(np.diff(t) <= 0):
        raise SystemExit(f"{path}: time column must be strictly increasing")
    tq = np.arange(t[0], t[-1] + 1e-9, dt)
    pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
    return pts - pts[0]


def read_return_meta(path: str) -> tuple[np.ndarray, str]:
    """Pass-2 start relative to pass-1 start (NED m) and turn direction, from
    the '#' lines coverage_min_snap.py writes."""
    start, turn = None, "left"
    for line in Path(path).read_text().splitlines():
        if line.startswith("# pass2_start_from_pass1_start:"):
            x, y = line.split(":", 1)[1].split(",")
            start = np.array([float(x), float(y), 0.0])
        elif line.startswith("# turn:"):
            turn = line.split(":", 1)[1].strip()
        elif not line.startswith("#"):
            break
    if start is None:
        raise SystemExit(f"{path} has no '# pass2_start_from_pass1_start' line: "
                         f"make it with trajectory/coverage_min_snap.py")
    return start, turn


def glide(delta: np.ndarray, dt: float) -> np.ndarray:
    """Smooth rest-to-rest move from 0 to delta (quintic), peak GLIDE_SPEED."""
    d = float(np.linalg.norm(delta[:2]))
    T = max(3.0, 1.875 * d / GLIDE_SPEED)
    n = max(2, int(math.ceil(T / dt)))
    s = np.arange(n + 1) / n
    h = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return np.outer(h, delta)


def check_upload(pts: np.ndarray, dt_ms: int, max_speed: float, name: str) -> float:
    duration = (len(pts) - 1) * dt_ms / 1000
    vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
    log.info("%s: %d points, %.1f s, max speed %.3f m/s", name, len(pts), duration, vmax)
    if vmax > max_speed + 1e-6:
        raise SystemExit(f"{name}: max speed {vmax:.3f} m/s exceeds --max-speed {max_speed} m/s")
    if len(pts) > TRAJ_MAX_PTS:
        raise SystemExit(f"{name}: {len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt")
    return duration


def main() -> None:
    ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--trajectory", help="CSV with columns t,x,y,z (s, NED m)")
    ap.add_argument("--return-trajectory",
                    help="Second CSV from coverage_min_snap.py: turn 180 deg after the "
                         "first pass, then fly this")
    ap.add_argument("--turn-test", action="store_true",
                    help="Only take off, do the 180 deg turn, and land (try this first)")
    ap.add_argument("--no-turn", action="store_true",
                    help="Fly the return pass without turning round (camera keeps facing "
                         "the far wall; use if --turn-test shows the turn doesn't work)")
    ap.add_argument("--turn", choices=["left", "right"],
                    help="Turn direction (default: from the return CSV, or left)")
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
                    help="Fleet config (optional)")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Upload sample period (s); 0.05 = ESP nav loop rate")
    ap.add_argument("--max-speed", type=float, default=0.30,
                    help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
    ap.add_argument("--takeoff-wait", type=float, default=8.0,
                    help="Seconds from CMD_START to trajectory upload")
    ap.add_argument("--finish", choices=["land", "hold"], default="land",
                    help="Action after the trajectory")
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing TRAJ-<drone id> before sending any flight commands")
    ap.add_argument("--no-plot", action="store_true",
                    help="Don't open the live plot window (CSV log and PNG are still saved)")
    ap.add_argument("--no-tags", action="store_true", help="Don't log AprilTag detections")
    ap.add_argument("--no-snaps", action="store_true",
                    help="Don't stream the camera or save pictures of detections")
    ap.add_argument("--altitude", type=float, default=0.5,
                    help="Assumed flying height (m) for checking tag sightings")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()

    if args.turn_test:
        args.takeoff = True
    elif not args.trajectory:
        raise SystemExit("--trajectory is required (or use --turn-test)")

    dt_ms = round(args.dt * 1000)
    if not 1 <= dt_ms <= 1000:
        raise SystemExit("--dt must be 0.001-1.0 s")
    dt = dt_ms / 1000

    # --- load and check everything before anything flies ------------------
    pts1 = pts2 = start2 = None
    turn = args.turn or "left"
    if args.trajectory:
        pts1 = load_trajectory(args.trajectory, dt)
        dur1 = check_upload(pts1, dt_ms, args.max_speed, "pass 1")
    if args.return_trajectory and not args.turn_test:
        start2, turn_csv = read_return_meta(args.return_trajectory)
        turn = args.turn or turn_csv
        pts2 = load_trajectory(args.return_trajectory, dt)
        worst = np.vstack([glide(np.array([GLIDE_MAX_M, 0, 0]), dt), pts2[1:] + [GLIDE_MAX_M, 0, 0]])
        check_upload(worst, dt_ms, args.max_speed, "pass 2 (with longest glide)")

    latest = {}

    def on_telemetry(pkt, _ip):
        if pkt.drone_id == args.drone_id:
            latest["pkt"] = pkt
            latest["turn_snapshot"] = (pkt, time.monotonic())

    def wait_for(cond, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if "pkt" in latest and cond(latest["pkt"]):
                return True
            time.sleep(0.05)
        return False

    def send(cmd_type: int, name: str, gx: float = 0.0, gy: float = 0.0) -> None:
        if not comms.send_command(args.drone_id, CommandPacket(cmd_type, goal_x=gx, goal_y=gy)):
            raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
        log.info("%s sent", name)

    def play(pts: np.ndarray, name: str) -> None:
        """Upload, start (retrying if the drone didn't switch to TRAJ), wait to the end."""
        traj_id = random.randint(1, 255)
        packets = build_traj_packets(traj_id, pts)
        start = build_traj_start(traj_id, dt_ms)
        for attempt in range(1, 6):
            for p in packets:
                comms.send_raw(args.drone_id, p)
                time.sleep(UPLOAD_GAP_S)
            comms.send_raw(args.drone_id, start)
            if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
                break
            log.warning("%s attempt %d: trajectory not started (see ESP log)", name, attempt)
        else:
            raise RuntimeError(f"drone refused {name}")
        duration = (len(pts) - 1) * dt
        log.info("playing %s (%.1f s)", name, duration)
        end = time.monotonic() + duration + 10.0
        while latest["pkt"].nav_state == NAV_TRAJ and time.monotonic() < end:
            p = latest["pkt"]
            log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
            time.sleep(1.0)
        log.info("%s ended, nav state %s", name, latest["pkt"].nav_state_name)

    def turn_around(direction: str) -> None:
        from turn_in_place import turn_in_place

        def send_turn(command):
            if not comms.send_command(args.drone_id, command):
                raise RuntimeError("Turn not sent: drone IP unknown")

        log.info("Turning 180 degrees %s in place", direction)
        turn_in_place(direction, lambda: latest["turn_snapshot"], send_turn)
        log.info("Turn complete; position and heading held")

    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    comms.on_telemetry(on_telemetry)
    flog = FlightLogger(drone_id=args.drone_id, print_every_s=0)
    comms.on_telemetry(flog.feed)
    log.info("Logging telemetry to %s", flog.path)

    # Plan for the plot: pass 1, then pass 2 in pass 1's frame.
    ref_path = str(Path(flog.path).with_suffix("")) + "_plan.csv"
    with open(ref_path, "w") as fh:
        fh.write("t,x,y,z\n")
        rows = []
        if pts1 is not None:
            rows += [(i * dt, *p) for i, p in enumerate(pts1)]
        if pts2 is not None:
            t_off = (rows[-1][0] if rows else 0.0) + 10.0
            rows += [(t_off + i * dt, *(start2 + p)) for i, p in enumerate(pts2)]
        for r in rows or [(0.0, 0, 0, 0), (1.0, 0, 0, 0)]:
            fh.write(f"{r[0]:.3f},{r[1]:.4f},{r[2]:.4f},{r[3]:.4f}\n")

    # AprilTags + pictures (optional; never block the flight)
    tags = snaps = None
    if not args.no_tags:
        try:
            from tag_logger import PoseHistory, TagLogger
            poses = PoseHistory(args.drone_id)
            comms.on_telemetry(poses.feed)
            tags = TagLogger(flog.path, args.drone_id, poses, t0=flog.t0,
                             altitude=args.altitude)
            if not args.no_snaps:
                from tag_snapshots import SnapshotSaver
                snaps = SnapshotSaver(flog.path, args.drone_id, comms, t0=flog.t0,
                                      origin_fn=tags._origin)
                if snaps.start():
                    tags.on_place = snaps.on_detection
                else:
                    snaps = None
            if not tags.start():
                tags = None
        except Exception as exc:
            log.warning("Not logging tags: %s", exc)
            tags = None

    if not args.no_plot:
        start_live_plot(flog.path, ref_path)
    comms.start()
    flying = not args.takeoff   # without --takeoff the drone must already hover
    try:
        if not wait_for(lambda p: True, 30.0):
            raise RuntimeError(f"no telemetry from drone {args.drone_id}")
        p = latest["pkt"]
        if tags:
            tags.origin = (p.ned_x, p.ned_y)          # tag positions from the start point

        if args.confirm:
            what = ("a 180 deg turn test" if args.turn_test else args.trajectory
                    + (f" + turn + {args.return_trajectory}" if pts2 is not None else ""))
            answer = input(f"Type TRAJ-{args.drone_id} to fly {what}: ").strip()
            if answer != f"TRAJ-{args.drone_id}":
                log.info("confirmation did not match; cancelled")
                return

        if args.takeoff:
            send(CMD_START, "CMD_START")
            flying = True
            time.sleep(args.takeoff_wait)

        if args.turn_test:
            turn_around(turn)
        else:
            p = latest["pkt"]
            org1 = np.array([p.ned_x, p.ned_y, 0.0])   # pass 1 is flown from here
            play(pts1, "pass 1")
            if pts2 is not None:
                if args.no_turn:
                    send(CMD_HOLD, "CMD_HOLD (no turn)")
                    time.sleep(2.5)
                else:
                    turn_around(turn)
                p = latest["pkt"]
                delta = org1 + start2 - np.array([p.ned_x, p.ned_y, 0.0])
                delta[2] = 0.0
                if np.linalg.norm(delta) > GLIDE_MAX_M:
                    raise RuntimeError(f"pass 2 would start {np.linalg.norm(delta):.2f} m "
                                       f"away; not risking it")
                log.info("gliding %.2f m to the start of pass 2", np.linalg.norm(delta))
                up2 = np.vstack([glide(delta, dt), pts2[1:] + delta])
                check_upload(up2, dt_ms, args.max_speed, "pass 2")
                play(up2, "pass 2")

        if args.finish == "land":
            for _ in range(3):   # UDP: repeat LAND
                send(CMD_LAND, "CMD_LAND")
        else:
            send(CMD_HOLD, "CMD_HOLD")
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted — sending CMD_LAND")
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
        raise
    finally:
        if flying:
            time.sleep(5.0)    # keep logging the landing before closing the log
        comms.stop()
        flog.close()
        if tags:
            fused = tags.close()
            ox, oy = tags._origin()
            for tid, f in fused.items():
                log.info("  tag %2d  x=%+.2f  y=%+.2f  (+/-%.2f m, %d sightings, %s)",
                         tid, f["x"] - ox, f["y"] - oy, f["spread"], f["frames"],
                         "confirmed" if f["confirmed"] else "UNCONFIRMED")
        if snaps:
            snaps.close()
        save_plot_png(flog.path, ref_path)


if __name__ == "__main__":
    main()
