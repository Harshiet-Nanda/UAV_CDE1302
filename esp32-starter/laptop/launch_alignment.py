"""Wait for firmware-verified launch XY; immediate abort LAND stays separate."""
import time

from protocol import (CMD_HOME_LAND, NAV_TAKEOFF_ALIGN, NAV_FLIGHT_READY,
                      NAV_HOME_ALIGN, NAV_HOME_READY, NAV_ALIGN_FAILED)


def wait_launch_alignment(get_sample, *, landing=False, send=None, min_wait=0.0,
                          clock=time.monotonic, sleep=time.sleep, timeout=55.0):
    """Fresh alignment acknowledgement then completion; returns last packet.

    Called immediately after START, or sends HOME_LAND itself for normal finish.
    min_wait replaces the old blind post-START sleep while still watching state.
    Receive gaps do not abort this wait; only newly received samples may advance
    alignment. The overall timeout and explicit firmware failures still apply.
    """
    active = NAV_HOME_ALIGN if landing else NAV_TAKEOFF_ALIGN
    done = NAV_HOME_READY if landing else NAV_FLIGHT_READY
    started = clock()
    last_received = get_sample()[1]
    acknowledged = False
    last_send = -float("inf")
    while clock()-started < timeout:
        now = clock()
        packet, received = get_sample()
        if received > last_received:
            last_received = received
            if packet.nav_state == NAV_ALIGN_FAILED:
                raise RuntimeError("takeoff/home alignment correction failed (ALIGN_FAILED); mission blocked. "
                                   "Read ESP32 serial TAKEOFF/ALIGN logs for altitude timeout, "
                                   "telemetry ages, XY/Z/speed limits or CMD_HOLD. "
                                   "Increasing XY radius does not change the altitude gate.")
            if packet.nav_state == active:
                acknowledged = True
            if acknowledged and packet.nav_state == done and now-started >= min_wait:
                return packet
            if acknowledged and packet.nav_state not in (active, done):
                raise RuntimeError("launch XY alignment interrupted")
        if landing and not acknowledged and now-last_send >= 0.75:
            if send is None:
                raise ValueError("normal landing requires command sender")
            send(CMD_HOME_LAND, "CMD_HOME_LAND (verify launch XY)")
            last_send = now
        if landing and not acknowledged and now-started > 3.0:
            raise RuntimeError("no HOME_ALIGN acknowledgement: updated firmware required")
        sleep(0.05)
    raise RuntimeError("launch XY alignment timed out; refusing to continue")
