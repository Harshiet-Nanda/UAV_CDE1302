"""Dashboard integration with legacy telemetry and onboard detection packets."""
import csv
import math
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from protocol import parse_telemetry
from flight_log import FlightLogger
from plot_trajectory import LogTail, TrajectoryPlot


def packet():
    return struct.pack('<BBfffBbf32sBH', 1, 22, 1., 2., math.pi/2,
                       2, -1, 0., bytes(32), 0, 65535)


def test_legacy_csv_to_dashboard(tmp_path):
    log = FlightLogger(str(tmp_path/'flight.csv'), print_every_s=0)
    log.feed(parse_telemetry(packet()))
    prefix = packet()
    log.feed(parse_telemetry(prefix))
    log.close()
    tail = LogTail(log.path, 22)
    assert tail.poll() and not tail.poll()
    plot = TrajectoryPlot('Test dashboard')
    plot.update(tail)
    assert len(plot.height_line.get_ydata()) == 0
    assert len(plot.ax_log.tables) == 0
    assert 'FLYING' in plot.event_text.get_text()
    assert plot.hdg_line.get_xdata()[1] > plot.hdg_line.get_xdata()[0]
    plot.fig.canvas.draw()
    plt.close(plot.fig)


def test_legacy_csv_and_partial_write(tmp_path):
    path = tmp_path/'legacy.csv'
    path.write_text('t_s,drone_id,x_m,y_m,heading_rad,nav_state,is_stuck\n0,22,3,4,0,IDLE,0\n1,22,')
    tail = LogTail(str(path), 22)
    assert tail.poll() and len(tail.t) == 1
    with path.open('a') as f:
        f.write('4,5,0,FLYING,0\n')
    assert tail.poll() and tail.x == [0., 1.]
    plot = TrajectoryPlot('Legacy')
    plot.update(tail)
    assert 'unavailable' in plot.height_note.get_text()
    plt.close(plot.fig)


def test_onboard_results_reach_gui_without_host_detector(tmp_path, monkeypatch):
    import json
    import socket
    import time
    import apriltag_host
    from tag_logger import PoseHistory, TagLogger

    def forbidden(*args, **kwargs):
        raise AssertionError("Laptop detection must never run")
    monkeypatch.setattr(apriltag_host, 'Detector', forbidden)
    poses = PoseHistory(7)
    telemetry = bytearray(packet())
    telemetry[1] = 7
    poses.feed(parse_telemetry(bytes(telemetry)))
    logger = TagLogger(str(tmp_path/'flight.csv'), 7, poses, port=0, verbose=False)
    assert logger.start()
    plot = TrajectoryPlot('Onboard detection')
    plot.live = True
    plot.detections_path = logger.status_path
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ('127.0.0.1', logger._sock.getsockname()[1])

    def send_and_wait(frame, count):
        data = struct.pack('<BBIHbBB', 4, 7, frame, 120, -1, count, count)
        if count:
            data += struct.pack('<bB7f', 5, 0, 100., 160., 120., .1, .2, .5, .01)
        tx.sendto(data, address)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                status = json.loads(logger.status_path.read_text())
                if status['frame_ms'] == frame:
                    return status
            except (OSError, ValueError):
                pass
            time.sleep(.01)
        pytest.fail('Onboard packet did not reach GUI status')

    try:
        status = send_and_wait(1000, 1)
        assert status['detections'][0]['id'] == 5
        plot.update_detections()
        assert 'T5: (+0.10, +0.20, +0.50)' in plot.detection_note.get_text()
        assert 'LIVE' in plot.detection_note.get_text()
        send_and_wait(1200, 0)
        plot.update_detections()
        assert 'No AprilTags' in plot.detection_note.get_text()
        assert 'T5:' not in plot.detection_note.get_text()
        # Delay the recorded result to check that old sightings aren't called live.
        status = json.loads(logger.status_path.read_text())
        status['received_at'] = time.time() - 5
        logger.status_path.write_text(json.dumps(status))
        plot.update_detections()
        assert 'STALE' in plot.detection_note.get_text()
        assert not list(tmp_path.glob('*_snaps'))
        assert not logger.status_path.with_suffix('.tmp').exists()
        plot.fig.canvas.draw()
        plot.fig.savefig(tmp_path/'onboard-dashboard.png')
    finally:
        tx.close()
        logger.close()
        plt.close(plot.fig)
    with open(logger.path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1 and rows[0]["tag_id"] == "5"
    assert rows[0]["used"] == "1"
    assert rows[0]["tag_x"] and rows[0]["tag_y"]


def test_gui_and_mission_launchers_have_no_video_receiver():
    # Catch reintroducing the implicit video/detection startup path.
    import ast
    root = Path(__file__).resolve().parents[1]
    for filename in ('send_trajectory_logged.py', 'waypoint_logged.py', 'plot_trajectory.py'):
        tree = ast.parse((root/filename).read_text())
        imports = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        assert not imports.intersection({'tag_snapshots', 'camera_stream', 'tag_stream'})
        assert not any(isinstance(node, ast.Name) and node.id in {'Detector', 'SnapshotSaver'}
                       for node in ast.walk(tree))


def test_missing_onboard_status_is_explicit(tmp_path):
    plot = TrajectoryPlot('Missing')
    plot.detections_path = tmp_path/'missing.json'
    plot.update_detections()
    assert 'unavailable' in plot.detection_note.get_text()
    assert 'no laptop detection' in plot.detection_note.get_text()
    plt.close(plot.fig)
