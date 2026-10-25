#!/usr/bin/env python3
"""
Snake ("lawnmower") coverage of a rectangular room as one minimum-snap
trajectory, for laptop/send_trajectory.py.

    python trajectory/coverage_min_snap.py              (uses SETTINGS below)
    python trajectory/coverage_min_snap.py --length 4 --width 3

The room is split into lanes as wide as what the downward camera sees.
By default (--lanes across) the drone starts at the middle of the back wall
(pass that wall as --width), flies out to one side, stops, flies back across
the whole first lane, then snakes side to side away from the wall and finishes
at the far wall. --start corner starts in a back corner (no stop needed).
--lanes along flies lanes the length of the room and returns to the start. The corners are rounded (U-turns become
semicircles), then a minimum-snap polynomial is fitted through points along
that path, the same 7th-order, C3, equality-constrained QP as
ellipse_min_snap_demo.m, including its trick of constraining the velocity
direction at each waypoint so the curve doesn't cut corners or bulge.

Room coordinates used below (what you measure with a tape):
    f = forward: along --length, the way the drone's nose points
    r = right:   along --width, toward the drone's right
    Back wall = the wall behind the drone at the start (length 0).
    The CSV is relative to the start point, so x/y can be negative.

Outputs (default next to this file):
    coverage_traj.csv   t,x,y,z (s, NED m), offsets from the start point, 100 Hz
    coverage_traj.png   top view: room, path, camera coverage, speed

Assumes the camera points straight down from the drone centre, with the
image's wide (320 px) axis across the drone (left-right). The default field
of view comes from the firmware's QVGA intrinsics in laptop/apriltag_host.py.
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla


# ===========================================================================
# SETTINGS: edit these, then run  python trajectory/coverage_min_snap.py
# (any of them can still be overridden on the command line, e.g. --length 6)
# ===========================================================================
ROOM_LENGTH_M = 5.0      # start wall -> far wall, the way the drone's nose points
ROOM_WIDTH_M  = 4.0      # along the start wall (side wall to side wall)
MARGIN_M      = 0.7      # closest the path may get to any wall
ALTITUDE_M    = 0.5      # flying height used for the camera footprint
LANES         = "across" # "across": side to side, ends at far wall
                         # "along":  lengthwise, returns to the start
START         = "centre" # "centre" of the start wall, or "corner"
SWEEP         = "right"  # "left" mirrors the pattern
HEADING_DEG   = 0.0      # drone heading (QGC) when its nose points along the room
MAX_SPEED_MS  = 0.25     # peak speed; send_trajectory.py refuses > 0.30
OVERLAP       = 0.15     # overlap between neighbouring lanes (0-0.9)
TAG_SIZE_M    = 0.12     # tag edge length
# ===========================================================================

HERE = Path(__file__).resolve().parent

# QVGA (320x240) intrinsics from laptop/apriltag_host.py FIRMWARE_INTRINSICS
FX, FY, IMG_W, IMG_H = 163.5047216, 153.22210511, 320, 240
DEFAULT_HFOV = math.degrees(2 * math.atan(IMG_W / 2 / FX))   # ~88.8 deg, across
DEFAULT_VFOV = math.degrees(2 * math.atan(IMG_H / 2 / FY))   # ~76.1 deg, along

TRAJ_MAX_PTS = 2400        # NAV_TRAJ_MAX_PTS in main/nav_task.h
UPLOAD_DT = 0.05           # send_trajectory.py default --dt
# main/nav_task.c collision_avoid() is disabled (returns false first thing), so
# nothing onboard stops the drone at a wall: --margin is the only protection.


# ---------------------------------------------------------------------------
# Geometry: straight lines and arcs in room coordinates (f, r)
# ---------------------------------------------------------------------------

@dataclass
class Line:
    p0: np.ndarray
    p1: np.ndarray

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    def point(self, s: float) -> tuple[np.ndarray, np.ndarray]:
        u = (self.p1 - self.p0) / self.length
        return self.p0 + u * s, u

    def distance(self, q: np.ndarray) -> np.ndarray:
        d = self.p1 - self.p0
        k = np.clip(((q - self.p0) @ d) / (d @ d), 0.0, 1.0)
        return np.linalg.norm(q - (self.p0 + k[:, None] * d), axis=1)


@dataclass
class Arc:
    c: np.ndarray       # centre
    rad: float
    a0: float           # start angle
    sweep: float        # signed, + = counter-clockwise in (f, r) axes

    @property
    def length(self) -> float:
        return abs(self.sweep) * self.rad

    def point(self, s: float) -> tuple[np.ndarray, np.ndarray]:
        a = self.a0 + math.copysign(s / self.rad, self.sweep)
        p = self.c + self.rad * np.array([math.cos(a), math.sin(a)])
        u = math.copysign(1.0, self.sweep) * np.array([-math.sin(a), math.cos(a)])
        return p, u

    def distance(self, q: np.ndarray) -> np.ndarray:
        v = q - self.c
        ang = np.arctan2(v[:, 1], v[:, 0])
        rel = np.mod((ang - self.a0) * math.copysign(1.0, self.sweep), 2 * math.pi)
        on_arc = rel <= abs(self.sweep)
        d_arc = np.abs(np.linalg.norm(v, axis=1) - self.rad)
        e0 = self.point(0.0)[0]
        e1 = self.point(self.length)[0]
        d_end = np.minimum(np.linalg.norm(q - e0, axis=1), np.linalg.norm(q - e1, axis=1))
        return np.where(on_arc, d_arc, d_end)


def fillet_polyline(corners: list[np.ndarray], radius: float) -> list:
    """Straight polyline -> lines joined by tangent arcs of `radius`."""
    pieces: list = []
    start = corners[0]
    for i in range(1, len(corners) - 1):
        p_prev, p, p_next = corners[i - 1], corners[i], corners[i + 1]
        a = (p - p_prev) / np.linalg.norm(p - p_prev)
        b = (p_next - p) / np.linalg.norm(p_next - p)
        cross = a[0] * b[1] - a[1] * b[0]
        theta = math.acos(float(np.clip(a @ b, -1.0, 1.0)))     # deflection
        if theta < 1e-6:
            continue
        if theta > math.pi - 1e-3:          # doubles back: stop here, no arc
            if np.linalg.norm(p - start) > 1e-9:
                pieces.append(Line(start, p))
            start = p
            continue
        tl = radius * math.tan(theta / 2)
        s_pt, e_pt = p - a * tl, p + b * tl
        left = np.array([-a[1], a[0]])
        c = s_pt + left * radius * math.copysign(1.0, cross)
        a0 = math.atan2(s_pt[1] - c[1], s_pt[0] - c[0])
        if np.linalg.norm(s_pt - start) > 1e-9:
            pieces.append(Line(start, s_pt))
        pieces.append(Arc(c, radius, a0, math.copysign(theta, cross)))
        start = e_pt
    if np.linalg.norm(corners[-1] - start) > 1e-9:
        pieces.append(Line(start, corners[-1]))
    return pieces


# ---------------------------------------------------------------------------
# Coverage plan
# ---------------------------------------------------------------------------

@dataclass
class Plan:
    corners: list[np.ndarray]
    lanes_r: np.ndarray
    f0: float
    f1: float
    swath: float          # usable footprint across the lane (m)
    footlen: float        # usable footprint along the lane (m)
    spacing: float        # actual lane spacing (m)


def footprint(alt, hfov, vfov, tag) -> tuple[float, float]:
    """Usable camera footprint (across the drone, along the drone) in metres:
    the area in which a whole tag of edge `tag` is in view."""
    swath = 2 * alt * math.tan(math.radians(hfov) / 2) - tag
    footlen = 2 * alt * math.tan(math.radians(vfov) / 2) - tag
    if swath <= 0 or footlen <= 0:
        raise SystemExit("Footprint too small for the tag at this altitude.")
    return swath, footlen


def plan_across(L, W, alt, hfov, vfov, tag, overlap, margin, start="centre") -> Plan:
    """Lanes run side to side (across --width), stepping away from the back
    wall; the path ends at the far wall. The nose stays pointing into the
    room, so lanes are spaced by the camera's shorter, along-the-drone view."""
    swath, footlen = footprint(alt, hfov, vfov, tag)
    step = footlen * (1.0 - overlap)
    r_in = max(margin, swath / 2)          # lane ends: distance from side walls
    f_in = max(margin, footlen / 2)        # first/last lane: distance from end walls
    r0, r1 = r_in, W - r_in
    f0, f1 = f_in, L - f_in
    if f1 - f0 < 0 or r1 - r0 <= 0:
        raise SystemExit("Room too small for this --margin.")
    if f1 - f0 < 0.5 * step:              # one lane in the middle sees it all
        lanes = np.array([(f0 + f1) / 2])
    else:
        n = int(math.ceil((f1 - f0) / step - 1e-9)) + 1
        lanes = np.linspace(f0, f1, n)
    spacing = float(lanes[1] - lanes[0]) if len(lanes) > 1 else 0.0
    if len(lanes) > 1 and r1 - r0 < spacing + 1e-6:
        raise SystemExit(f"Lanes ({r1 - r0:.2f} m) are shorter than the turn diameter "
                         f"({spacing:.2f} m). Lower --margin or use --lanes along.")

    corners = []
    if start == "centre":
        # From the middle of the back wall: out to one side, then back across
        # the whole first lane (a stop-and-reverse), so lane 1 is fully seen.
        corners = [np.array([lanes[0], W / 2]), np.array([lanes[0], r0])]
        ends = (r0, r1)
    else:
        ends = (r0, r1)
    for i, f in enumerate(lanes):
        a, b = ends if i % 2 == 0 else ends[::-1]
        if not corners or np.linalg.norm(corners[-1] - np.array([f, a])) > 1e-9:
            corners.append(np.array([f, a]))
        corners.append(np.array([f, b]))
    return Plan(corners, lanes, f0, f1, swath, footlen, spacing)


def plan_lanes(L, W, alt, hfov, vfov, tag, overlap, margin, start="centre") -> Plan:
    swath, footlen = footprint(alt, hfov, vfov, tag)
    step = swath * (1.0 - overlap)

    r_in = max(margin, swath / 2)          # lane centre-line distance from side walls
    f_in = max(margin, footlen / 2)        # turn line distance from end walls
    r0, r_end = r_in, W - r_in
    f0, f1 = f_in, L - f_in
    if f1 - f0 <= 0 or W < 2 * margin:
        raise SystemExit("Room too small for this --margin.")
    if r_end <= r0:
        lanes = np.array([W / 2])
    else:
        n = int(math.ceil((r_end - r0) / step - 1e-9)) + 1
        if start == "centre" and n % 2:
            n += 1           # even: the snake ends back on the start wall
        lanes = np.linspace(r0, r_end, n)
    spacing = float(lanes[1] - lanes[0]) if len(lanes) > 1 else 0.0
    if len(lanes) > 1 and f1 - f0 < spacing + 1e-6:
        raise SystemExit(f"Lanes ({f1 - f0:.2f} m) are shorter than the turn diameter "
                         f"({spacing:.2f} m). Swap --length and --width or lower --margin.")

    if start == "centre" and len(lanes) > 1:
        c = W / 2
        corners = [np.array([f0, c])]
        for i, r in enumerate(lanes):
            a, b = (f0, f1) if i % 2 == 0 else (f1, f0)
            corners += [np.array([a, r]), np.array([b, r])]
        corners.append(np.array([f0, c]))            # back along the start wall
        return Plan(corners, lanes, f0, f1, swath, footlen, spacing)

    corners = []
    for i, r in enumerate(lanes):
        a, b = (f0, f1) if i % 2 == 0 else (f1, f0)
        corners += [np.array([a, r]), np.array([b, r])]
    # Return to the start without cutting through the room or reversing.
    if len(lanes) % 2 == 0:                      # ended on the back line
        corners.append(np.array([f0, lanes[0]]))
    elif len(lanes) > 1:                         # ended on the front line
        corners.append(np.array([f1, lanes[0]]))
        corners.append(np.array([f0, lanes[0]]))
    else:                                        # one lane: there and back
        corners.append(np.array([f0, lanes[0]]))
    return Plan(corners, lanes, f0, f1, swath, footlen, spacing)


# ---------------------------------------------------------------------------
# Waypoints with tangent directions
# ---------------------------------------------------------------------------

@dataclass
class Waypoints:
    p: np.ndarray        # (n+1, 2) positions
    u: np.ndarray        # (n+1, 2) unit tangents (zeros where the drone stops)
    stop: np.ndarray     # (n+1,) bool: velocity forced to zero
    seg_len: np.ndarray  # (n,) path length of each segment
    seg_turn: np.ndarray # (n,) bool: segment lies on an arc


def build_waypoints(pieces, line_step, arc_step_rad) -> Waypoints:
    P, U, seg_len, seg_turn = [], [], [], []
    for k, pc in enumerate(pieces):
        if isinstance(pc, Line):
            n = max(1, int(math.ceil(pc.length / line_step - 1e-9)))
        else:
            n = max(2, int(math.ceil(abs(pc.sweep) / arc_step_rad - 1e-9)))
        for j in range(0 if k == 0 else 1, n + 1):
            p, u = pc.point(pc.length * j / n)
            P.append(p)
            U.append(u)
        seg_len += [pc.length / n] * n
        seg_turn += [isinstance(pc, Arc)] * n
    P, U = np.array(P), np.array(U)
    stop = np.zeros(len(P), bool)
    stop[0] = stop[-1] = True
    # A reversal (path doubles back) needs a stop, e.g. the one-lane case.
    for i in range(1, len(P) - 1):
        a = P[i] - P[i - 1]
        b = P[i + 1] - P[i]
        if a @ b < -0.9 * np.linalg.norm(a) * np.linalg.norm(b):
            stop[i] = True
    U[stop] = 0.0
    return Waypoints(P, U, stop, np.array(seg_len), np.array(seg_turn))


# ---------------------------------------------------------------------------
# Minimum-snap QP (same formulation as ellipse_min_snap_demo.m)
# ---------------------------------------------------------------------------

N_ORDER = 7
N_COEF = N_ORDER + 1


def basis(tau, deriv, T):
    tau = np.atleast_1d(np.asarray(tau, float))
    M = np.zeros((tau.size, N_COEF))
    for p in range(deriv, N_ORDER + 1):
        M[:, p] = math.factorial(p) / math.factorial(p - deriv) * tau ** (p - deriv) / T ** deriv
    return M


def solve_min_snap(wp: Waypoints, T: np.ndarray) -> np.ndarray:
    """Coefficients (n_seg*8, 2) in normalised time tau = t/T per segment."""
    n_seg = len(T)
    nv = n_seg * N_COEF                       # variables per axis
    rows, cols, vals, rhs = [], [], [], []
    r = 0

    def add(axis, seg, row_vec, b=0.0, row=None):
        base = axis * nv + seg * N_COEF
        for j, v in enumerate(row_vec):
            if v != 0.0:
                rows.append(row)
                cols.append(base + j)
                vals.append(v)

    def new_row(b):
        nonlocal r
        rhs.append(b)
        r += 1
        return r - 1

    for ax in range(2):
        # start / end: position fixed, vel/acc/jerk zero
        for d in range(4):
            i = new_row(wp.p[0, ax] if d == 0 else 0.0)
            add(ax, 0, basis(0, d, T[0])[0], row=i)
            i = new_row(wp.p[-1, ax] if d == 0 else 0.0)
            add(ax, n_seg - 1, basis(1, d, T[-1])[0], row=i)
        for s in range(n_seg - 1):
            i = new_row(wp.p[s + 1, ax])
            add(ax, s, basis(1, 0, T[s])[0], row=i)
            i = new_row(wp.p[s + 1, ax])
            add(ax, s + 1, basis(0, 0, T[s + 1])[0], row=i)
            for d in range(1, 4):
                i = new_row(0.0)
                add(ax, s, basis(1, d, T[s])[0], row=i)
                add(ax, s + 1, -basis(0, d, T[s + 1])[0], row=i)
            if wp.stop[s + 1]:
                i = new_row(0.0)
                add(ax, s, basis(1, 1, T[s])[0], row=i)

    # Velocity parallel to the path tangent at internal waypoints:
    # u_r * v_f - u_f * v_r = 0  (the ellipse demo's normal . v = 0)
    for s in range(n_seg - 1):
        if wp.stop[s + 1]:
            continue
        uf, ur = wp.u[s + 1]
        vrow = basis(1, 1, T[s])[0]
        i = new_row(0.0)
        add(0, s, ur * vrow, row=i)
        add(1, s, -uf * vrow, row=i)

    A = sp.csr_matrix((vals, (rows, cols)), shape=(r, 2 * nv))

    blocks = []
    for s in range(n_seg):
        q = np.zeros((N_COEF, N_COEF))
        for i in range(4, N_COEF):
            for j in range(4, N_COEF):
                q[i, j] = (math.factorial(i) / math.factorial(i - 4)
                           * math.factorial(j) / math.factorial(j - 4)
                           / (i + j - 7) / T[s] ** 7)
        blocks.append(q)
    Q1 = sp.block_diag([sp.coo_matrix(b) for b in blocks])
    Q = sp.block_diag([Q1, Q1], format="csc")

    KKT = sp.bmat([[Q, A.T], [A, None]], format="csc")
    sol = spla.spsolve(KKT, np.concatenate([np.zeros(2 * nv), rhs]))
    if not np.all(np.isfinite(sol)):
        raise SystemExit("Minimum-snap solve failed (singular system).")
    return np.column_stack([sol[:nv], sol[nv:2 * nv]])


def sample(coeff, T, t):
    edges = np.concatenate([[0.0], np.cumsum(T)])
    seg = np.clip(np.searchsorted(edges, t, side="right") - 1, 0, len(T) - 1)
    out = [np.zeros((t.size, 2)) for _ in range(3)]
    for s in np.unique(seg):
        m = seg == s
        tau = np.clip((t[m] - edges[s]) / T[s], 0.0, 1.0)
        C = coeff[s * N_COEF:(s + 1) * N_COEF]
        for d in range(3):
            out[d][m] = basis(tau, d, T[s]) @ C
    return out   # position, velocity, acceleration


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _shown(path: Path) -> str:
    """Path relative to the current folder when possible (for the printed command)."""
    try:
        return Path(os.path.relpath(path)).as_posix()
    except ValueError:          # different drive on Windows
        return path.as_posix()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--length", type=float, default=ROOM_LENGTH_M,
                    help="Room size along the drone's nose direction (m)")
    ap.add_argument("--width", type=float, default=ROOM_WIDTH_M, help="Room size across (m)")
    ap.add_argument("--margin", type=float, default=MARGIN_M,
                    help="Minimum distance from the path to any wall (m). Must cover takeoff "
                         "drift (~0.35 m seen in logs) plus the drone's own radius; onboard "
                         "collision avoidance is disabled in this firmware.")
    ap.add_argument("--altitude", type=float, default=ALTITUDE_M,
                    help="Flying height for the footprint (m). The firmware hovers at "
                         "CRUISE_ALT_M = 0.5 m; changing it needs a reflash.")
    ap.add_argument("--hfov", type=float, default=DEFAULT_HFOV,
                    help=f"Camera FOV across the drone (deg, default {DEFAULT_HFOV:.1f})")
    ap.add_argument("--vfov", type=float, default=DEFAULT_VFOV,
                    help=f"Camera FOV along the drone (deg, default {DEFAULT_VFOV:.1f})")
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M,
                    help="Tag edge (m). Footprint is shrunk by this so a tag is fully in view.")
    ap.add_argument("--overlap", type=float, default=OVERLAP,
                    help="Fraction of overlap between neighbouring lanes (0-0.9)")
    ap.add_argument("--lanes", choices=["across", "along"], default=LANES,
                    help="across (default): side-to-side lanes stepping away from the back "
                         "wall, ending at the far wall. along: lanes run the length of the "
                         "room and the path returns to the start.")
    ap.add_argument("--start", choices=["centre", "corner"], default=START,
                    help="Start at the middle of the back wall (default) or in a back corner")
    ap.add_argument("--sweep", "--first-turn", dest="sweep", choices=["right", "left"],
                    default=SWEEP,
                    help="'left' mirrors the pattern left-right (check the PNG)")
    ap.add_argument("--heading", type=float, default=HEADING_DEG,
                    help="Drone heading in degrees when its nose points along --length "
                         "(read it in QGC). Rotates the path into NED.")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED_MS,
                    help="Peak speed (m/s); send_trajectory.py refuses > 0.30")
    ap.add_argument("--turn-speed", type=float, default=0.6,
                    help="Speed in turns relative to straight lanes (0.2-1)")
    ap.add_argument("--line-step", type=float, default=0.5,
                    help="Max waypoint spacing along straight lanes (m)")
    ap.add_argument("--out", default=str(HERE / "coverage_traj.csv"))
    ap.add_argument("--no-png", action="store_true")
    a = ap.parse_args()

    if a.max_speed > 0.30 + 1e-9:
        raise SystemExit("--max-speed above 0.30 m/s will be refused by send_trajectory.py")
    if not 0 <= a.overlap < 0.9:
        raise SystemExit("--overlap must be in [0, 0.9)")

    L, W = a.length, a.width
    planner = plan_across if a.lanes == "across" else plan_lanes
    plan = planner(L, W, a.altitude, a.hfov, a.vfov, a.tag_size, a.overlap, a.margin, a.start)
    radius = plan.spacing / 2
    pieces = fillet_polyline(plan.corners, radius)
    wp = build_waypoints(pieces, a.line_step, math.radians(45))

    # Time allocation: length / nominal speed, slower in turns and next to
    # stops (start, end, reversals). A few rounds then rebalance segment times
    # so no stretch is much faster than its nominal speed, which stops one
    # short burst from slowing the whole flight. Solved at mean 1 s for
    # conditioning, then scaled uniformly to hit --max-speed; a uniform time
    # scale leaves the minimum-snap shape unchanged.
    v_nom = np.where(wp.seg_turn, a.turn_speed, 1.0)
    shape_T = wp.seg_len / v_nom
    next_to_stop = wp.stop[:-1] | wp.stop[1:]
    shape_T[next_to_stop] *= 2.0
    best = None
    for _ in range(7):
        shape_T *= len(shape_T) / shape_T.sum()
        coeff = solve_min_snap(wp, shape_T)
        edges = np.concatenate([[0.0], np.cumsum(shape_T)])
        ts = np.concatenate([np.linspace(edges[i], edges[i + 1], 25)
                             for i in range(len(shape_T))])
        p_s, v_s, _ = sample(coeff, shape_T, ts)
        sp_s = np.linalg.norm(v_s, axis=1).reshape(len(shape_T), 25)
        dur = shape_T.sum() * sp_s.max() / a.max_speed
        dev_s = np.min(np.vstack([pc.distance(p_s) for pc in pieces]), axis=0).max()
        # keep the fastest allocation that still follows the planned path
        key = (dev_s > 0.02, dur if dev_s <= 0.02 else dev_s)
        if best is None or key < best[0]:
            best = (key, shape_T.copy())
        ratio = sp_s.max(axis=1) / v_nom
        shape_T = shape_T * np.clip(ratio / np.median(ratio), 0.7, 1.5) ** 0.5

    shape_T = best[1]
    coeff = solve_min_snap(wp, shape_T)
    t_shape = np.linspace(0, shape_T.sum(), 20000)
    _, v_shape, _ = sample(coeff, shape_T, t_shape)
    scale = np.linalg.norm(v_shape, axis=1).max() / a.max_speed
    T = shape_T * scale
    duration = float(T.sum())

    t = np.round(np.arange(0.0, duration + 1e-9, 0.01), 2)
    pos, vel, acc = sample(coeff, T, t)
    speed = np.linalg.norm(vel, axis=1)

    # --- checks -------------------------------------------------------------
    q = pos
    dev = np.min(np.vstack([pc.distance(q) for pc in pieces]), axis=0)
    wall = np.min(np.column_stack([q[:, 0], L - q[:, 0], q[:, 1], W - q[:, 1]]), axis=1)
    edges = np.concatenate([[0.0], np.cumsum(T)])
    v_join = np.array([sample(coeff, T, np.array([edges[i]]))[1][0]
                       for i in range(1, len(T))])
    backward = [i + 1 for i, v in enumerate(v_join)
                if not wp.stop[i + 1] and v @ wp.u[i + 1] < 0]

    # camera coverage: area of the room where a whole tag is in view at some time
    g = 0.02
    nf, nr = int(math.ceil(L / g)), int(math.ceil(W / g))
    seen = np.zeros((nf, nr), bool)
    hf, hr = plan.footlen / 2, plan.swath / 2
    for pf, pr in pos[::5]:
        i0, i1 = max(0, int((pf - hf) / g)), min(nf, int(math.ceil((pf + hf) / g)))
        j0, j1 = max(0, int((pr - hr) / g)), min(nr, int(math.ceil((pr + hr) / g)))
        seen[i0:i1, j0:j1] = True
    coverage = seen.mean() * 100

    # --- write CSV in NED, relative to the start point ------------------------
    sign = 1.0 if a.sweep == "right" else -1.0
    df = pos[:, 0] - pos[0, 0]
    dr = (pos[:, 1] - pos[0, 1]) * sign
    psi = math.radians(a.heading)
    x = df * math.cos(psi) - dr * math.sin(psi)
    y = df * math.sin(psi) + dr * math.cos(psi)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as fh:
        fh.write("t,x,y,z\n")
        for row in zip(t, x, y):
            fh.write(f"{row[0]:.2f},{row[1]:.6f},{row[2]:.6f},0\n")

    # same resampling and speed check as send_trajectory.py
    tq = np.arange(0.0, duration + 1e-9, UPLOAD_DT)
    up = np.column_stack([np.interp(tq, t, x), np.interp(tq, t, y)])
    v_up = np.linalg.norm(np.diff(up, axis=0), axis=1).max() / UPLOAD_DT
    n_up = len(tq)
    dt_needed = UPLOAD_DT if n_up <= TRAJ_MAX_PTS else \
        math.ceil(duration / (TRAJ_MAX_PTS - 1) / 0.05) * 0.05

    side = "left" if a.sweep == "right" else "right"
    start = plan.corners[0]
    print(f"Room {L:.2f} m (along nose) x {W:.2f} m, altitude {a.altitude:.2f} m")
    print(f"Usable camera footprint {plan.footlen:.2f} m along x {plan.swath:.2f} m across "
          f"(tag {a.tag_size:.2f} m fully in view)")
    print(f"{len(plan.lanes_r)} lane{'s' if len(plan.lanes_r) != 1 else ''}, spacing {plan.spacing:.2f} m, "
          f"turn radius {radius:.2f} m, {len(wp.p)} waypoints")
    print(f"Path {sum(pc.length for pc in pieces):.1f} m, duration {duration:.1f} s, "
          f"peak speed {speed.max():.3f} m/s, peak accel {np.linalg.norm(acc, axis=1).max():.3f} m/s^2")
    print(f"Max deviation from planned path {dev.max() * 100:.1f} cm; "
          f"closest approach to a wall {wall.min():.2f} m")
    print(f"Camera coverage of the room: {coverage:.0f} %")
    if backward:
        print(f"WARNING: velocity reverses at waypoints {backward}; try --turn-speed or --line-step")
    if wall.min() < 0.6:
        print("WARNING: path comes within 0.6 m of a wall. Takeoff drift alone has been "
              "~0.35 m and nothing onboard stops a wall strike. Increase --margin.")
    print(f"Upload at --dt {UPLOAD_DT}: {n_up} points (limit {TRAJ_MAX_PTS}), "
          f"max speed {v_up:.3f} m/s")
    if n_up > TRAJ_MAX_PTS:
        print(f"  Too many points: use --dt {dt_needed:.2f} with send_trajectory.py")
    other = "right" if side == "left" else "left"
    where = ("in the MIDDLE of the back wall" if abs(start[1] - W / 2) < 1e-6
             else f"{start[1]:.2f} m from the {side} wall")
    print(f"\nPLACE THE DRONE {where}, {start[0]:.2f} m out from the back wall "
          f"({start[1]:.2f} m to the {side} wall, {W - start[1]:.2f} m to the {other}), "
          f"nose pointing along the room (heading {a.heading:.0f} deg).")
    e = pos[-1]
    if np.linalg.norm(e - pos[0]) > 1e-6:
        e_side = side if e[1] < W / 2 - 1e-6 else (other if e[1] > W / 2 + 1e-6 else "middle")
        print(f"FINISHES {L - e[0]:.2f} m from the far wall, "
              f"{min(e[1], W - e[1]):.2f} m from the {e_side} wall "
              f"(CSV x={x[-1]:+.2f}, y={y[-1]:+.2f} m from the start).")
    if a.lanes == "along" and W > L:
        print("Note: the start wall (--width) is the longer side; lanes still run along "
              "--length, so there will be more turns.")
    print(f"Wrote {out}")
    print(f"\npython laptop/send_trajectory.py --drone-id <ID> --takeoff --takeoff-wait 15 "
          f"--confirm --trajectory {_shown(out)}"
          + (f" --dt {dt_needed:.2f}" if n_up > TRAJ_MAX_PTS else ""))

    if not a.no_png:
        draw(out.with_suffix(".png"), a, plan, pieces, wp, pos, speed, t, seen, g,
             coverage, duration, sign, side)
    return 0


def draw(png, a, plan, pieces, wp, pos, speed, t, seen, g, coverage, duration, sign, side):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Rectangle

    L, W = a.length, a.width
    # Draw from above with the nose up. With --sweep left the room is
    # mirrored so the picture matches what you see standing behind the drone.
    def X(r):  # room r -> plot x
        return r if sign > 0 else W - r

    fig, (ax, axs) = plt.subplots(1, 2, figsize=(13, 7),
                                  gridspec_kw={"width_ratios": [1.5, 1]})
    ax.imshow(seen, origin="lower", extent=[0, W, 0, L] if sign > 0 else [W, 0, 0, L],
              cmap="Greens", alpha=0.35, vmin=0, vmax=1.6, aspect="equal")
    ax.add_patch(Rectangle((0, 0), W, L, fill=False, lw=2.5, color="k"))
    ax.add_patch(Rectangle((a.margin, a.margin), W - 2 * a.margin, L - 2 * a.margin,
                           fill=False, lw=1, ls=":", color="tab:red"))
    pts = np.column_stack([X(pos[:, 1]), pos[:, 0]]).reshape(-1, 1, 2)
    lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap="viridis", lw=2.2)
    lc.set_array(speed[:-1])
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02).set_label("speed (m/s)")
    ax.plot(X(wp.p[:, 1]), wp.p[:, 0], ".", color="0.35", ms=4, label="min-snap waypoints")
    s = plan.corners[0]
    e = pos[-1]
    returns = np.linalg.norm(e - pos[0]) < 1e-6
    ax.plot(X(s[1]), s[0], "o", color="tab:red", ms=11,
            label="start / finish" if returns else "start")
    if not returns:
        ax.plot(X(e[1]), e[0], "s", color="k", ms=10, label="finish")
    ax.annotate("", xy=(X(s[1]), s[0] + 0.45), xytext=(X(s[1]), s[0]),
                arrowprops=dict(arrowstyle="-|>", color="tab:red", lw=2.5))
    centred = abs(s[1] - W / 2) < 1e-6
    ax.text(X(s[1]), s[0] - 0.1,
            (f"DRONE HERE, nose up: middle of back wall, {s[0]:.2f} m out" if centred else
             f"DRONE HERE, nose up: {s[0]:.2f} m from back wall, {s[1]:.2f} m from {side} wall"),
            color="tab:red", fontsize=8, fontweight="bold", va="top",
            ha="center" if centred else ("left" if sign > 0 else "right"))
    # one example footprint
    mid = len(pos) // 3
    fp = Rectangle((X(pos[mid, 1]) - plan.swath / 2, pos[mid, 0] - plan.footlen / 2),
                   plan.swath, plan.footlen, fill=False, ec="tab:blue", lw=1.2, ls="--")
    ax.add_patch(fp)
    ax.plot([], [], ls="--", color="tab:blue", label="camera footprint (tag fully in view)")
    ax.plot([], [], ls=":", color="tab:red", label=f"--margin {a.margin:.2f} m")
    ax.set_xlim(-0.2, W + 0.2)
    ax.set_ylim(-0.2, L + 0.2)
    ax.set_aspect("equal")
    ax.set_xlabel(f"across the room (m)   [{side} wall at 0]" if sign > 0
                  else f"across the room (m)   [{side} wall at {W:g}]")
    ax.set_ylabel("along the room, nose direction (m)   [back wall at 0]")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8)
    ax.set_title(f"{L:g} m x {W:g} m room, top view, {len(plan.lanes_r)} lanes\n"
                 f"green = seen by camera ({coverage:.0f} % of floor)")

    axs.plot(t, speed, color="tab:blue")
    axs.axhline(a.max_speed, color="0.6", ls="--", lw=1)
    axs.set_xlabel("time (s)")
    axs.set_ylabel("speed (m/s)")
    axs.set_title(f"{duration:.0f} s, peak {speed.max():.2f} m/s")
    axs.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    print(f"Wrote {png}")


if __name__ == "__main__":
    raise SystemExit(main())