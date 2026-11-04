"""Position-holding firmware turn; no movement-command fallback."""
import math
import time

from protocol import CMD_HOLD, CMD_TURN, NAV_TURN, NAV_TURN_DONE, NAV_TURN_FAILED


def run_turn(get_sample, send, target, direction, *, clock=time.monotonic,
             sleep=time.sleep, timeout=50.0):
    """get_sample -> (packet, monotonic receive time). send accepts cmd,name,x,y.

    Require fresh TURN acknowledgement, then one second of fresh TURN_DONE
    packets near the fixed target. Old firmware must fail before a return pass.
    """
    if direction not in ("left", "right") or not math.isfinite(target):
        raise ValueError("invalid turn target/direction")
    packet, received = get_sample()
    if clock()-received > 0.5:
        raise RuntimeError("fresh telemetry required before turn")
    x0, y0 = packet.ned_x, packet.ned_y
    target = math.remainder(target, 2*math.pi)
    started = clock()
    acknowledged = False
    last_send = -math.inf
    settled = None
    samples = 0
    last_received = received
    try:
        while clock()-started < timeout:
            now = clock()
            packet, received = get_sample()
            if now-received > 0.5:
                raise RuntimeError("telemetry lost during turn")
            if not all(math.isfinite(v) for v in
                       (packet.ned_x, packet.ned_y, packet.heading_rad)):
                raise RuntimeError("invalid telemetry during turn")
            if math.hypot(packet.ned_x-x0, packet.ned_y-y0) > 0.5:
                raise RuntimeError("turn drift exceeded 0.5 m")
            if received > last_received:
                last_received = received
                if packet.nav_state == NAV_TURN_FAILED:
                    raise RuntimeError("firmware turn failed; reverse pass cancelled")
                if packet.nav_state == NAV_TURN:
                    acknowledged = True
                if acknowledged and packet.nav_state not in (NAV_TURN, NAV_TURN_DONE):
                    raise RuntimeError("turn interrupted; reverse pass cancelled")
                error = abs(math.remainder(target-packet.heading_rad, 2*math.pi))
                if acknowledged and packet.nav_state == NAV_TURN_DONE and error <= math.radians(5):
                    if settled is None:
                        settled = now
                    samples += 1
                    if now-settled >= 1.0 and samples >= 5:
                        return target
                else:
                    settled = None
                    samples = 0
            if not acknowledged:
                if now-started > 3.0:
                    raise RuntimeError("no TURN acknowledgement: install updated ESP32 firmware")
                if now-last_send >= 0.75:
                    send(CMD_TURN, "CMD_TURN", target, 1.0 if direction == "right" else -1.0)
                    last_send = now
            sleep(0.05)
        raise RuntimeError("turn did not settle; reverse pass cancelled")
    except BaseException:
        send(CMD_HOLD, "CMD_HOLD (turn aborted)", 0.0, 0.0)
        raise
