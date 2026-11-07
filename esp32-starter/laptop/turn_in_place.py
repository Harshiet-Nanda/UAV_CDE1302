"""Dedicated fixed-position turn; requires matching ESP32 CMD_TURN firmware."""
import math
import time
from protocol import CMD_TURN, NAV_TURNING, NAV_TURN_FAILED, NAV_ARRIVED, CommandPacket

TURN_SPEED = math.radians(15)
STALE_TIMEOUT = 3.0  # same default as waypoint_logged.py
TURN_TIMEOUT = 30.0


def turn_in_place(direction, snapshot, send, *, clock=time.monotonic, sleep=time.sleep):
    """snapshot() returns an atomic (packet, local reception time) pair."""
    def fresh():
        p, received = snapshot()
        if clock() - received > STALE_TIMEOUT:
            raise ConnectionError("Turn telemetry is stale (over 3 s)")
        return p, received

    p, _ = fresh()
    target = math.remainder(p.heading_rad + math.pi, 2*math.pi)
    speed = TURN_SPEED if direction == "right" else -TURN_SPEED
    command = CommandPacket(CMD_TURN, goal_x=target, goal_y=speed)
    started = clock()
    send(command)
    last_sent = started
    acknowledged = False
    while clock()-started < TURN_TIMEOUT + 2.0:
        p, received = fresh()
        if received > started:
            if p.nav_state == NAV_TURN_FAILED:
                raise RuntimeError("ESP32 turn failed")
            if p.nav_state == NAV_TURNING:
                acknowledged = True
            elif acknowledged:
                if p.nav_state == NAV_ARRIVED:
                    error = abs(math.remainder(p.heading_rad-target, 2*math.pi))
                    if error <= math.radians(5):
                        return
                    raise RuntimeError("Turn ended outside heading tolerance")
                raise RuntimeError("Turn interrupted before completion")
        if not acknowledged:
            if clock()-started >= 3.0:
                raise RuntimeError("No TURNING acknowledgement; matching firmware required")
            if clock()-last_sent >= 0.5:
                send(command)  # identical retry: firmware must not restart
                last_sent = clock()
        sleep(0.05)
    raise TimeoutError("Turn did not complete within its deadline")
