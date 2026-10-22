#!/usr/bin/env python3
"""Display the camera pose in each AprilTag marker frame.

Run: python laptop/tag_stream_unprocessed.py --esp-ip 192.168.1.222
Add --no-display for terminal output only. Uses the existing detector's tag
size, intrinsics and Hamming/margin gate. No body transform or smoothing.
The detector returns a rotation matrix; rvec is its Rodrigues axis-angle
representation of the inverse rotation (camera frame to marker frame),
not Euler angles. tvec is the camera centre relative to the marker centre.
"""

import argparse
import select
import socket
import time

import cv2
import numpy as np

from apriltag_host import Detector, passes_gate
from camera_stream import COMMAND_PORT, KEEPALIVE_S, FrameAssembler, drain_socket
from protocol import CAMERA_STREAM_PORT, build_camera_stream_command

GREEN = (0, 255, 0)     # OpenCV colours are BGR
BLACK = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def to_view(x, y, scale):
    """AprilTag pixel (x, y) -> pixel in the scaled window.

    AprilTag puts (0, 0) at the corner of the top-left pixel, OpenCV at its
    centre, hence the - 0.5.
    """
    return round(x * scale - 0.5), round(y * scale - 0.5)


def pose_vectors(det):
    """Return camera-in-marker translation and Rodrigues rotation vector."""
    if det.t is None or det.R is None:
        return None
    rotation = np.asarray(det.R, dtype=np.float64).reshape(3, 3)
    # Detector: p_camera = R @ p_marker + t. Invert the rigid transform.
    inverse_rotation = rotation.T
    translation = -inverse_rotation @ np.asarray(det.t, dtype=np.float64)
    rvec, _ = cv2.Rodrigues(inverse_rotation)
    return translation, rvec.reshape(3)


def vector_text(name, values, unit):
    return f"{name} [{', '.join(f'{v:+.4f}' for v in values)}] {unit}"


def draw_tag(view, det, scale):
    pts = [to_view(x, y, scale) for x, y in det.corners]
    cv2.polylines(view, [np.array(pts, np.int32)], True, GREEN, 2, cv2.LINE_AA)
    x, y = pts[0]
    cv2.putText(view, str(det.id), (max(0, x), max(15, y)),
                FONT, 0.5, GREEN, 2, cv2.LINE_AA)


def draw_vectors(view, lines):
    """Use a separate panel so vector labels cannot obscure each other."""
    width = max(view.shape[1], 620)
    panel = np.zeros((max(1, len(lines)) * 24 + 12, width, 3), np.uint8)
    for row, line in enumerate(lines):
        cv2.putText(panel, line, (8, 24 * row + 23), FONT, 0.5,
                    GREEN, 1, cv2.LINE_AA)
    image = np.zeros((view.shape[0], width, 3), np.uint8)
    image[:, :view.shape[1]] = view
    return np.vstack((image, panel))


def main():
    parser = argparse.ArgumentParser(description="Camera pose in the AprilTag marker frame.")
    parser.add_argument("--esp-ip", required=True,
                        help="drone IP, 192.168.1.(200 + drone id)")
    parser.add_argument("--fps", type=int, default=0,
                        help="1-15 (default 0 = firmware's 10)")
    parser.add_argument("--quality", type=int, default=0,
                        help="JPEG quality 10-90 (default 0 = firmware's 60)")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="window size multiplier (default 2)")
    parser.add_argument("--no-display", action="store_true",
                        help="Print vectors in the terminal without opening a window")
    args = parser.parse_args()
    if not np.isfinite(args.scale) or args.scale <= 0:
        parser.error("--scale must be finite and positive")
    try:
        start_cmd = build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    esp_ip = socket.gethostbyname(args.esp_ip)  # packets carry numeric IPs

    # Frames arrive on one UDP port; start/stop commands go out on another.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)  # 1 MB
    rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drone = (esp_ip, COMMAND_PORT)

    detector = Detector()           # compiles the drone's C code on first run
    assembler = FrameAssembler()    # joins UDP pieces back into whole JPEGs
    window = "AprilTags: camera in marker frame"
    if not args.no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"Asking {esp_ip} for video. Ctrl+C quits; in the window q/Esc quits, s saves.")
    print("Each ID defines its own marker frame. tvec: camera position (m); rvec: camera-to-marker axis-angle (rad).")

    next_keepalive = 0.0
    view = None
    try:
        while True:
            # The drone only streams while it keeps hearing from us.
            if time.monotonic() >= next_keepalive:
                next_keepalive = time.monotonic() + KEEPALIVE_S
                try:
                    tx.sendto(start_cmd, drone)
                except OSError as exc:      # e.g. not on the drone Wi-Fi
                    print(f"cannot reach drone: {exc}")

            select.select([rx], [], [], 0.01)   # wait up to 10 ms for data
            frame, _, _ = drain_socket(rx, assembler, esp_ip)
            assembler.expire(time.monotonic())  # forget frames missing a piece

            if frame is not None:
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8),
                                    cv2.IMREAD_GRAYSCALE)   # (240, 320) uint8
                if gray is not None:
                    tags = [d for d in detector.detect(gray) if passes_gate(d)]
                    lines = []
                    for det in tags:
                        vectors = pose_vectors(det)
                        if vectors is None:
                            continue
                        tvec, rvec = vectors
                        lines.extend((f"ID {det.id} camera: " + vector_text("tvec", tvec, "m"),
                                      "       " + vector_text("rvec", rvec, "rad")))
                        print(f"ID {det.id} camera-in-marker | {vector_text('tvec', tvec, 'm')} | "
                              f"{vector_text('rvec', rvec, 'rad')}", flush=True)
                    if not args.no_display:
                        h, w = gray.shape
                        size = (max(1, round(w * args.scale)), max(1, round(h * args.scale)))
                        view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                          size, interpolation=cv2.INTER_NEAREST)
                        for det in tags:
                            draw_tag(view, det, args.scale)
                        view = draw_vectors(view, lines or ["No accepted tags"])
                        cv2.imshow(window, view)

            key = (cv2.waitKey(1) & 0xFF) if not args.no_display else -1
            if key in (ord("q"), 27):           # 27 = Esc
                break
            if key == ord("s") and view is not None:
                path = time.strftime("tag_stream_unprocessed_%Y%m%d_%H%M%S.png")
                cv2.imwrite(path, view)
                print(f"saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tx.sendto(build_camera_stream_command(False), drone)  # stop stream
        except OSError:
            pass
        detector.close()
        rx.close()
        tx.close()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
