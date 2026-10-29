"""
Telemetry logger: records every telemetry packet to a CSV and (optionally)
prints a throttled position line to the console.

Attach it to any script that already uses CommsNode — two lines:

    from flight_log import FlightLogger
    flog = FlightLogger(drone_id=args.drone_id)     # writes logs/flight_<time>.csv
    comms.on_telemetry(flog.feed)                   # before comms.start()
    ...
    flog.close()                                    # in the finally: block

Then watch it live in a second terminal:

    python3 laptop/plot_trajectory.py --file logs/flight_<time>.csv --live

CSV columns: t_s, drone_id, x_m, y_m, heading_rad, nav_state, is_stuck
  t_s   seconds since the logger was created (laptop clock)
  x_m   NED north in the map frame (PX4 local + setup.yaml start offset)
  y_m   NED east  in the map frame
"""

import csv
import math
import threading
import time
from pathlib import Path
from typing import Optional

from protocol import TelemetryPacket


class FlightLogger:
    def __init__(self, path: Optional[str] = None, drone_id: Optional[int] = None,
                 print_every_s: float = 0.5) -> None:
        """
        path:          CSV path; default logs/flight_<YYYYmmdd_HHMMSS>.csv
        drone_id:      only log this drone (None = log every drone heard)
        print_every_s: console print interval per drone (0 = never print)
        """
        if path is None:
            Path("logs").mkdir(exist_ok=True)
            path = time.strftime("logs/flight_%Y%m%d_%H%M%S.csv")
        self.path = path
        self._drone_id = drone_id
        self._print_every_s = print_every_s
        self._t0 = time.monotonic()
        self.t0 = self._t0          # time.monotonic() of t_s = 0 (shared with tag_logger)
        self._last_print: dict[int, float] = {}
        self._lock = threading.Lock()
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["t_s", "drone_id", "x_m", "y_m",
                               "heading_rad", "nav_state", "is_stuck"])
        self._file.flush()
        print(f"[flight_log] logging telemetry to {path}")

    def feed(self, pkt: TelemetryPacket, _source_ip: str = "") -> None:
        """CommsNode telemetry callback."""
        if self._drone_id is not None and pkt.drone_id != self._drone_id:
            return
        t = time.monotonic() - self._t0
        with self._lock:
            if self._file.closed:
                return
            self._writer.writerow([f"{t:.3f}", pkt.drone_id,
                                   f"{pkt.ned_x:.4f}", f"{pkt.ned_y:.4f}",
                                   f"{pkt.heading_rad:.4f}", pkt.nav_state_name,
                                   int(pkt.is_stuck)])
            self._file.flush()   # so plot_trajectory.py --live sees it at once

        if self._print_every_s > 0:
            last = self._last_print.get(pkt.drone_id, -math.inf)
            if t - last >= self._print_every_s:
                self._last_print[pkt.drone_id] = t
                print(f"[t={t:7.2f}s] drone {pkt.drone_id}: "
                      f"x={pkt.ned_x:+6.2f} m  y={pkt.ned_y:+6.2f} m  "
                      f"hdg={math.degrees(pkt.heading_rad):+6.1f}°  "
                      f"{pkt.nav_state_name}")

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()
                print(f"[flight_log] saved {self.path}")
