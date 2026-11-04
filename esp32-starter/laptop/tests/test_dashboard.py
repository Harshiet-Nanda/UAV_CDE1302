"""Dashboard integration using original kaiwei telemetry and camera protocol."""
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
