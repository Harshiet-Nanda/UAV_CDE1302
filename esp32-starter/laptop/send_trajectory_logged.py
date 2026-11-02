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

The turn. During a trajectory the ESP32 holds the heading it had when the
trajectory started, and this firmware has no "rotate in place" command: the
nose only turns while the drone flies at a GOTO goal (it points at the goal).
So the turn (default --turn-mode single) is done tactically:
  1. HOLD and wait TURN_SETTLE_S, so the drone has stopped sliding;
  2. a GOTO to a point TURN_GOAL_M away, aimed TURN_AIM_DEG round on the
     chosen side (just under 180, so PX4 swings that way); once the nose is
     TURN_SWITCH_DEG round, the goal moves to straight behind the start
     heading, so the nose settles exactly facing back;
  3. HOLD the moment the nose is within TURN_STOP_DEG of facing back, or the
     drone has moved TURN_MAX_MOVE_M, whichever comes first;
  4. if it is still more than TURN_RETRY_DEG off, one correction GOTO aimed
     straight back, cut off the same way.
"Behind" points back into the room, so the little movement there is goes
away from the far wall. The lead-in glide into pass 2 then removes it.
--turn-mode hops is the older method (several hops of TURN_STEP_DEG).
Try it on its own first:  --turn-test  (take off, turn, land). If it can't
turn reliably, fly the return pass with --no-turn.

Home. The spot the drone takes off from is "home" (0, 0). Every pass is
anchored to it, so drift doesn't shift the plan:
  - pass 1 starts with a slow lead-in back over home (cancels takeoff drift),
  - pass 2's lead-in goes to home + the offset in the return CSV,
  - after the last pass the drone flies back over home, then lands.
Each lead-in also brings the drone to --altitude (0.5 m) using the height in
telemetry. The lead-in is rebuilt from the drone's live position on every
start attempt, so drift while the ESP32 refuses a start doesn't carry over.
--no-recentre, --no-home and --no-height-fix switch these off.

CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
A trajectory is flown relative to where the drone hovers when it starts.
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
    NAV_IDLE,
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

# Single tactical turn (default)
TURN_SETTLE_S = 1.5    # HOLD this long before turning: stop the drone sliding
TURN_GOAL_M = 0.60     # GOTO goal this far away (must be > 0.25 m, the ESP's
                       # arrival radius, or it "arrives" before turning)
TURN_AIM_DEG = 170.0   # first aim this far round: < 180 so PX4 turns the chosen way
TURN_SWITCH_DEG = 100.0  # once this far round, aim straight back instead
TURN_STOP_DEG = 6.0    # HOLD once the nose is this close to facing back
TURN_MAX_MOVE_M = 0.40 # ...or once the drone has moved this far
TURN_TIMEOUT_S = 8.0   # ...or after this long
TURN_RETRY_DEG = 20.0  # still this far off afterwards: one correction GOTO
# Older multi-hop turn (--turn-mode hops)
TURN_STEP_DEG = 60.0   # each hop aims this far past where the nose points
TURN_HOP_M = 0.40      # GOTO this far ahead in the new direction (arrives at 0.25)
TURN_MAX_DRIFT_M = 1.0 # stop turning if the drone has wandered this far
TURN_MAX_HOPS = 15     # give up after this many hops
GLIDE_SPEED = 0.10     # m/s, peak speed of the lead-in glides
GLIDE_MAX_M = 1.2      # pass 2: refuse to glide further than this (something went wrong)
RECENTRE_MAX_M = 1.0   # pass 1: refuse if takeoff drifted further than this
HOME_SPEED = 0.20      # m/s, peak speed flying back to home after the last pass
HOME_MAX_M = 5.0       # refuse to fly home from further than this
Z_FIX_MAX_M = 0.30     # largest height correction a lead-in may make
START_ATTEMPTS = 20    # trajectory start retries (the ESP32 can refuse a start)


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


def glide(delta: np.ndarray, dt: float, speed: float = GLIDE_SPEED) -> np.ndarray:
    """Smooth rest-to-rest move from 0 to delta (quintic), peak `speed`."""
    d = float(np.linalg.norm(delta[:2]))
    T = max(3.0, 1.875 * d / speed)
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
    ap.add_argument("--turn-mode", choices=["single", "hops"], default="single",
                    help="single (default): stop, one GOTO behind, HOLD when facing "
                         "back. hops: the older several-hop turn")
    ap.add_argument("--turn", choices=["left", "right"],
                    help="Turn direction (default: from the return CSV, or left)")
    ap.add_argument("--config", default="none",
                    help="Fleet config with a start offset (default none: positions "
                         "are relative to where the drone starts)")
    ap.add_argument("--no-recentre", action="store_true",
                    help="Start pass 1 wherever the drone is after takeoff (no lead-in "
                         "back over the take-off spot)")
    ap.add_argument("--no-home", action="store_true",
                    help="Land where the last pass ends instead of flying back over the "
                         "take-off spot first")
    ap.add_argument("--no-height-fix", action="store_true",
                    help="Don't correct the height to --altitude at the start of each pass")
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
                    help="Flying height (m): each pass starts by moving to it, and tag "
                         "sightings are checked against it")
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
        lead = np.array([RECENTRE_MAX_M, 0, 0])
        check_upload(np.vstack([glide(lead, dt), pts1[1:] + lead]), dt_ms, args.max_speed,
                     "pass 1 (with longest lead-in)")
        check_upload(glide(np.array([HOME_MAX_M, 0, 0]), dt, HOME_SPEED), dt_ms,
                     args.max_speed, "return home (longest)")
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

    upload = {"id": random.randint(1, 255)}

    def lead_in(target_xy, max_m: float) -> np.ndarray:
        """(dx, dy, dz) from where the drone is right now to target_xy (None =
        stay put) at --altitude."""
        p = latest["pkt"]
        here = np.array([p.ned_x, p.ned_y])
        d = np.zeros(3)
        if target_xy is not None:
            d[:2] = np.asarray(target_xy, float) - here
        dist = float(np.hypot(d[0], d[1]))
        if dist > max_m:
            raise RuntimeError(f"the drone is {dist:.2f} m from where it should be "
                               f"(limit {max_m:.1f} m); not risking it")
        z = getattr(p, "z_m", None)
        if not args.no_height_fix:
            if z is not None and math.isfinite(z):
                d[2] = float(np.clip(-args.altitude - z, -Z_FIX_MAX_M, Z_FIX_MAX_M))
            elif not upload.get("warned_z"):
                upload["warned_z"] = True
                log.warning("no height in telemetry (needs the extended protocol.py "
                            "and firmware): height fix off")
        return d

    def leg(target_xy, pts, max_m: float, speed: float = GLIDE_SPEED):
        """Builder for one upload: a glide from the live position to target_xy
        (and the cruise height), then pts (offsets) flown from there."""
        def build() -> np.ndarray:
            d = lead_in(target_xy, max_m)
            out = glide(d, dt, speed)
            if pts is not None:
                out = np.vstack([out, pts[1:] + d])
            p = latest["pkt"]
            build.info = (float(np.hypot(d[0], d[1])), d[2])
            build.origin = np.array([p.ned_x, p.ned_y]) + d[:2]   # where pts start
            return out
        return build

    def play(build, name: str) -> None:
        """Upload, start (retrying if the drone didn't switch to TRAJ), wait to
        the end. The upload is rebuilt from the live position on every attempt,
        under a new id so no chunk of an older attempt can mix in."""
        for attempt in range(1, START_ATTEMPTS + 1):
            pts = build()
            try:
                check_upload(pts, dt_ms, args.max_speed, name)
            except SystemExit as exc:            # mid-flight: land, don't just exit
                raise RuntimeError(str(exc)) from None
            dist, dz = build.info
            log.info("%s: lead-in %.2f m, height change %+.2f m", name, dist, -dz)
            upload["id"] = upload["id"] % 255 + 1
            packets = build_traj_packets(upload["id"], pts)
            start = build_traj_start(upload["id"], dt_ms)
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

    def heading_err(q, target: float) -> float:
        return abs(math.remainder(q.heading_rad - target, 2 * math.pi))

    def turn_hop(theta: float, done, label: str, x0: float, y0: float) -> str:
        """One GOTO TURN_GOAL_M away at bearing theta: the ESP points the nose
        at it, so the nose swings toward theta. Returns (without stopping the
        drone) once done(q) is true, the drone is TURN_MAX_MOVE_M from (x0, y0)
        where the turn began, it arrives, or time runs out; returns why."""
        p = latest["pkt"]
        gx = p.ned_x + TURN_GOAL_M * math.cos(theta)
        gy = p.ned_y + TURN_GOAL_M * math.sin(theta)
        send(CMD_GOTO, label, gx, gy)
        t_sent = time.monotonic()
        resends, took = 0, False
        while time.monotonic() - t_sent < TURN_TIMEOUT_S:
            q = latest["pkt"]
            if q.nav_state not in (NAV_ARRIVED, NAV_IDLE):
                took = True                       # the ESP is flying at the goal
            elif not took and time.monotonic() - t_sent > 1.0 * (resends + 1) and resends < 3:
                send(CMD_GOTO, label + " (resend)", gx, gy)   # UDP: packet lost?
                resends += 1
            if done(q):
                return "done"
            if math.hypot(q.ned_x - x0, q.ned_y - y0) > TURN_MAX_MOVE_M:
                return f"moved {TURN_MAX_MOVE_M:.2f} m"
            if took and q.nav_state == NAV_ARRIVED:
                return "arrived"
            time.sleep(0.05)
        return "timeout"

    def hold(label: str) -> None:
        for _ in range(2):                        # UDP: make sure it stops
            send(CMD_HOLD, label)

    def turn_single(direction: str) -> None:
        """Stop, then swing the nose round with GOTOs behind the drone and HOLD
        the moment it faces back. See the module docstring."""
        hold("CMD_HOLD (before turn)")
        time.sleep(TURN_SETTLE_S)
        p0 = latest["pkt"]
        x0, y0, psi0 = p0.ned_x, p0.ned_y, p0.heading_rad
        target = psi0 + math.pi
        k_dir = 1.0 if direction == "right" else -1.0      # NED heading grows clockwise
        log.info("turning 180 deg to the %s (heading %.0f -> %.0f deg)", direction,
                 math.degrees(psi0) % 360, math.degrees(target) % 360)

        def facing_back(q) -> bool:
            return heading_err(q, target) < math.radians(TURN_STOP_DEG)

        def near_back(q) -> bool:
            return heading_err(q, target) < math.radians(180 - TURN_SWITCH_DEG)

        # Swing: aim TURN_AIM_DEG round from where the nose points (< 180, so PX4
        # turns the chosen way) until less than 180 - TURN_SWITCH_DEG is left.
        # Face back: aim straight back (unambiguous now) until it faces back.
        # No stop in between. A second round only if the first fell short.
        x_s, y_s = x0, y0
        for rnd in (1, 2):
            q = latest["pkt"]
            if heading_err(q, target) < math.radians(TURN_RETRY_DEG if rnd > 1 else 0.1):
                break
            if rnd > 1:
                log.info("turn: %.0f deg still to go, correcting",
                         math.degrees(heading_err(q, target)))
                x_s, y_s = q.ned_x, q.ned_y
            why = "done"
            if not near_back(q):
                why = turn_hop(q.heading_rad + k_dir * math.radians(TURN_AIM_DEG),
                               lambda q: near_back(q) or facing_back(q),
                               "turn GOTO (swing)", x_s, y_s)
            if why == "done" and not facing_back(latest["pkt"]):
                why = turn_hop(target, facing_back, "turn GOTO (face back)", x_s, y_s)
            hold("CMD_HOLD (stop turning)")
            time.sleep(TURN_SETTLE_S)
            log.info("turn: stopped (%s), %.0f deg off facing back", why,
                     math.degrees(heading_err(latest["pkt"], target)))
        q = latest["pkt"]
        err = math.degrees(heading_err(q, target))
        drift = math.hypot(q.ned_x - x0, q.ned_y - y0)
        (log.info if err < 25 else log.warning)(
            "turn done: heading %.0f deg (%.0f deg off a full 180), drifted %.2f m",
            math.degrees(q.heading_rad) % 360, err, drift)

    def turn_around(direction: str) -> None:
        if args.turn_mode == "single":
            turn_single(direction)
        else:
            turn_hops(direction)

    def turn_hops(direction: str) -> None:
        """Turn 180 deg with short GOTO hops, then HOLD (older method).

        The firmware flies at a GOTO goal while turning to face it and stops
        turning when it arrives (0.25 m), so each hop only turns the nose part
        of the way. Each hop therefore aims TURN_STEP_DEG past where the nose
        points now (never past the target), until it faces back."""
        p0 = latest["pkt"]
        x0, y0, psi0 = p0.ned_x, p0.ned_y, p0.heading_rad
        target = psi0 + math.pi
        k_dir = 1.0 if direction == "right" else -1.0      # NED heading grows clockwise
        log.info("turning 180 deg to the %s (heading %.0f -> %.0f deg)", direction,
                 math.degrees(psi0) % 360, math.degrees(target) % 360)
        turned = 0.0                                        # progress so far (rad)
        for k in range(1, TURN_MAX_HOPS + 1):
            p = latest["pkt"]
            turned = k_dir * math.remainder(p.heading_rad - psi0, 2 * math.pi)
            if turned < -0.5:                               # wrapped past 180
                turned += 2 * math.pi
            if math.pi - turned < math.radians(20):
                break
            theta = psi0 + k_dir * min(turned + math.radians(TURN_STEP_DEG), math.pi)
            gx = p.ned_x + TURN_HOP_M * math.cos(theta)
            gy = p.ned_y + TURN_HOP_M * math.sin(theta)
            send(CMD_GOTO, f"turn hop {k} (nose {math.degrees(turned):.0f} deg round)", gx, gy)
            if not wait_for(lambda q: q.nav_state != NAV_ARRIVED, 1.5):
                send(CMD_GOTO, "turn hop (resend)", gx, gy)
                wait_for(lambda q: q.nav_state != NAV_ARRIVED, 1.5)
            wait_for(lambda q: q.nav_state == NAV_ARRIVED
                     or abs(math.remainder(q.heading_rad - theta, 2 * math.pi)) < math.radians(10),
                     6.0)
            q = latest["pkt"]
            drift = math.hypot(q.ned_x - x0, q.ned_y - y0)
            if drift > TURN_MAX_DRIFT_M:
                log.warning("turn: drifted %.2f m, stopping the turn here", drift)
                break
        send(CMD_HOLD, "CMD_HOLD (end of turn)")
        time.sleep(2.5)                     # settle; let a fresh 1 Hz position arrive
        q = latest["pkt"]
        err = math.degrees(abs(math.remainder(q.heading_rad - target, 2 * math.pi)))
        drift = math.hypot(q.ned_x - x0, q.ned_y - y0)
        (log.info if err < 25 else log.warning)(
            "turn done: heading %.0f deg (%.0f deg off a full 180), drifted %.2f m",
            math.degrees(q.heading_rad) % 360, err, drift)

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

        if args.confirm:
            what = ("a 180 deg turn test" if args.turn_test else args.trajectory
                    + (f" + turn + {args.return_trajectory}" if pts2 is not None else ""))
            answer = input(f"Type TRAJ-{args.drone_id} to fly {what}: ").strip()
            if answer != f"TRAJ-{args.drone_id}":
                log.info("confirmation did not match; cancelled")
                return

        p = latest["pkt"]
        home = np.array([p.ned_x, p.ned_y])           # take-off spot = (0, 0)
        if tags:
            tags.origin = (p.ned_x, p.ned_y)
        log.info("home (take-off spot): (%.2f, %.2f)", home[0], home[1])

        if args.takeoff:
            send(CMD_START, "CMD_START")
            flying = True
            time.sleep(args.takeoff_wait)

        if args.turn_test:
            turn_around(turn)
        else:
            # Pass 1 from home (or from wherever takeoff left the drone)
            leg1 = (leg(None, pts1, 0.0) if args.no_recentre
                    else leg(home, pts1, RECENTRE_MAX_M))
            play(leg1, "pass 1")
            org1 = leg1.origin                        # where pass 1 really started
            if pts2 is not None:
                if args.no_turn:
                    send(CMD_HOLD, "CMD_HOLD (no turn)")
                    time.sleep(2.5)
                else:
                    turn_around(turn)
                # Pass 2 lined up with pass 1: its start is home + the CSV offset
                play(leg(org1 + start2[:2], pts2, GLIDE_MAX_M), "pass 2")
            if args.finish == "land" and not args.no_home:
                play(leg(home, None, HOME_MAX_M, HOME_SPEED), "return home")

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