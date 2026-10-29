#!/usr/bin/env python3
"""
Bench check for tag logging + pictures. No flying: props OFF, drone powered,
laptop on the drone Wi-Fi.

    python laptop/snap_check.py --drone-id 7

Runs the exact same code the flight scripts use (TagLogger + SnapshotSaver)
for --seconds (default 30). Hold a 12 cm tag ~0.5-1 m in front of the camera
while it runs. At the end it prints, in plain words, whether
    1. telemetry arrives          (UDP 5005)
    2. tag detections arrive      (UDP 5008)
    3. camera video arrives       (UDP 5009)
    4. pictures were saved
and, for anything that failed, the most likely reason.

Files go to logs/flight_<time>_* like a real flight, including
first_frame.jpg in the _snaps folder (the first picture the camera sent).
Close camera_stream.py, tag_stream.py and every other drone script first.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from comms import CommsNode                         # noqa: E402
from flight_log import FlightLogger                 # noqa: E402
from tag_logger import PoseHistory, TagLogger       # noqa: E402
from tag_snapshots import SnapshotSaver             # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--telem-port", type=int, default=5005)
    a = ap.parse_args()

    flog = FlightLogger(drone_id=a.drone_id, print_every_s=1e9)
    comms = CommsNode(listen_port=a.telem_port, config_path=None)
    comms.on_telemetry(flog.feed)
    poses = PoseHistory(a.drone_id)
    comms.on_telemetry(poses.feed)
    tags = TagLogger(flog.path, a.drone_id, poses, t0=flog.t0, verbose=True)
    snaps = SnapshotSaver(flog.path, a.drone_id, comms, t0=flog.t0,
                          origin_fn=tags._origin)
    snaps_ok = snaps.start()
    if snaps_ok:
        tags.on_place = snaps.on_detection
    tags_ok = tags.start()
    comms.start()

    print(f"\nChecking for {a.seconds:.0f} s. Hold a tag in front of the camera now. "
          "Ctrl+C stops early.\n")
    t_end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < t_end - 0.05:
            time.sleep(min(5.0, t_end - time.monotonic()))
            ip = comms.known_drones().get(a.drone_id)
            print(f"  {a.seconds - max(0.0, t_end - time.monotonic()):4.0f} s | telemetry: "
                  f"{'yes from ' + ip if ip else 'NO'} | detection packets: "
                  f"{tags._n_frames if tags_ok else 'port busy'} | tag in view: "
                  f"{tags._n_seen if tags_ok else '-'} | video frames: "
                  f"{snaps._n_frames if snaps_ok else 'port busy'} | pictures: "
                  f"{snaps._n_saved if snaps_ok else '-'}")
    except KeyboardInterrupt:
        pass
    finally:
        comms.stop()
        flog.close()

    ip = comms.known_drones().get(a.drone_id)
    print("\n================ RESULT ================")
    ok = True
    if ip:
        print(f"[OK]   telemetry from drone {a.drone_id} at {ip}")
    else:
        ok = False
        others = comms.known_drones()
        print(f"[FAIL] no telemetry from drone {a.drone_id}"
              + (f" (heard drones {sorted(others)}: wrong --drone-id?)" if others else
                 " (laptop on the drone Wi-Fi? drone powered? another script open?)"))
    if not tags_ok:
        ok = False
        print("[FAIL] detections: UDP 5008 is in use by another program (see above)")
    elif tags._n_frames == 0:
        ok = False
        print(f"[FAIL] detections: {tags.diagnosis()}")
    elif tags._n_seen == 0:
        ok = False
        print(f"[FAIL] detections arrive ({tags._n_frames} frames checked) but no tag seen: "
              f"{tags.diagnosis()}")
    else:
        print(f"[OK]   detections: tag in view in {tags._n_seen} of {tags._n_frames} frames")
    if not snaps_ok:
        ok = False
        print("[FAIL] video: UDP 5009 is in use by another program (see above)")
    elif snaps._n_frames == 0:
        ok = False
        print(f"[FAIL] video: {snaps.diagnosis()}")
    else:
        print(f"[OK]   video: {snaps._n_frames} frames "
              f"(~{snaps._n_frames / a.seconds:.0f} fps), see {snaps.dir / 'first_frame.jpg'}")
    tags.close() if tags_ok else None
    if snaps_ok:
        snaps.close()
        if snaps._n_saved:
            print(f"[OK]   pictures: {snaps._n_saved} saved in {snaps.dir}")
        elif snaps._n_frames and tags_ok and tags._n_seen:
            ok = False
            print(f"[FAIL] pictures: {snaps.diagnosis()}")
    print("All good: flights will log tags and save pictures." if ok else
          "Fix the first FAIL above, then run this again.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())