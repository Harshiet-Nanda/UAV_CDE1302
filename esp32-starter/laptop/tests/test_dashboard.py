"""Dashboard integration: legacy/new packets, CSV tailing and real tag overlays."""
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


def packet(ages=(0, 0, 0, 0)):
    prefix = struct.pack('<BBfffBbf32sBH', 1, 22, 1., 2., math.pi/2,
                         2, -1, 0., bytes(32), 0, 65535)
    suffix = struct.pack('<4s11f4H3B', b'GUI1', -.5, .1, .2, .3, .4, .5,
                         .6, .7, .8, 12., 3., *ages, 1, 6, 0)
    return prefix, suffix


def test_extension_fresh_stale_and_legacy():
    prefix, suffix = packet()
    fresh = parse_telemetry(prefix + suffix)
    assert fresh.z_m == -.5 and fresh.current_a == 3.
    assert fresh.px4_armed == 1 and fresh.px4_link_ok
    legacy = parse_telemetry(prefix)
    assert legacy.z_m is None and legacy.current_a is None
    _, stale = packet((2000, 65535, 2500, 3000))
    old = parse_telemetry(prefix + stale)
    assert old.z_m is None and old.roll_rad is None and old.current_a is None
    assert old.px4_armed is None and old.px4_link_ok is False
    assert parse_telemetry(prefix + suffix[:20]).z_m is None


def test_csv_to_height_and_heading(tmp_path):
    log = FlightLogger(str(tmp_path/'flight.csv'), print_every_s=0)
    log.feed(parse_telemetry(b''.join(packet())))
    prefix, _ = packet()
    log.feed(parse_telemetry(prefix))
    log.close()
    tail = LogTail(log.path, 22)
    assert tail.poll() and not tail.poll()
    plot = TrajectoryPlot('Test dashboard')
    plot.update(tail)
    assert plot.height_line.get_ydata()[0] == .5
    assert np.isnan(plot.height_line.get_ydata()[1])
    assert plot.hdg_line.get_xdata()[1] > plot.hdg_line.get_xdata()[0]
    assert plot.telemetry_tables['imu'][1, 1].get_text().get_text() == 'unavailable'
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


def test_real_scanner_publishes_and_clears_overlay(tmp_path):
    import cv2
    from apriltag_host import Detector
    from tag_snapshots import SnapshotSaver
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV aruco unavailable')
    saver = SnapshotSaver.__new__(SnapshotSaver)
    saver.dir, saver._cv2, saver._np = tmp_path, cv2, np
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    gray = np.full((240, 320), 255, np.uint8)
    gray[60:180, 100:220] = cv2.aruco.generateImageMarker(dictionary, 5, 120)
    with Detector() as detector:
        saver._detector = detector
        saver._publish_preview(cv2.imencode('.jpg', gray)[1].tobytes())
        image = cv2.imread(str(tmp_path/'preview.jpg'))
        # Real detector draws a green outline around the recognised marker.
        assert ((image[110:130, 195:205, 1].astype(int) - image[110:130, 195:205, 2]) > 80).any()
        plot = TrajectoryPlot('Camera')
        plot.preview_path = str(tmp_path/'preview.jpg')
        plot.update_camera()
        assert plot.camera_image.get_array().shape == (480, 640, 3)
        saver._publish_preview(cv2.imencode('.jpg', np.full_like(gray, 255))[1].tobytes())
        cleared = cv2.imread(str(tmp_path/'preview.jpg'))
        assert cleared[120, 200, 0] > 240
        assert not (tmp_path/'preview.tmp.jpg').exists()
        plt.close(plot.fig)
