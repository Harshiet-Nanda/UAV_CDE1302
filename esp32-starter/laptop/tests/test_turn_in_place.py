import math
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from protocol import (CMD_TURN, NAV_TURNING, NAV_TURN_FAILED, NAV_ARRIVED,
                      NAV_IDLE, CommandPacket, build_command)
from turn_in_place import turn_in_place


class Simulation:
    def __init__(self, state):
        self.now = 10.0
        self.state = state
        self.sent = []

    def snapshot(self):
        state, heading, age = self.state(self.now-10)
        return SimpleNamespace(nav_state=state, heading_rad=heading), self.now-age

    def run(self, direction="right"):
        turn_in_place(direction, self.snapshot, self.sent.append,
                      clock=lambda: self.now, sleep=self.advance)

    def advance(self, dt):
        self.now += dt


def test_packet_layout():
    raw = build_command(CommandPacket(CMD_TURN, goal_x=math.pi, goal_y=-math.radians(15)))
    assert len(raw) == 22
    pkt, cmd, yaw, speed, *_ = struct.unpack("<BBff12b", raw)
    assert (pkt, cmd) == (2, 10)
    assert yaw == pytest.approx(math.pi)
    assert speed == pytest.approx(-math.radians(15))


@pytest.mark.parametrize("direction,sign", [("left", -1), ("right", 1)])
def test_acknowledgement_before_arrival(direction, sign):
    sim = Simulation(lambda t: (NAV_IDLE, 0, 0) if t < .2 else
                     (NAV_TURNING, 0, 0) if t < 15 else (NAV_ARRIVED, math.pi, 0))
    sim.run(direction)
    assert sim.now >= 25
    assert sim.sent[0].goal_y == pytest.approx(sign*math.radians(15))


@pytest.mark.parametrize("kind", ["stale", "failed", "interrupted", "timeout", "wrong_heading", "no_ack"])
def test_failures_do_not_continue(kind):
    def state(t):
        if kind == "no_ack": return NAV_ARRIVED, math.pi, 0
        if t < .2: return NAV_IDLE, 0, 0
        if t < .5: return NAV_TURNING, 0, 0
        if kind == "stale": return NAV_TURNING, 0, 4
        if kind == "failed": return NAV_TURN_FAILED, 0, 0
        if kind == "interrupted": return NAV_IDLE, 0, 0
        if kind == "wrong_heading": return NAV_ARRIVED, 0, 0
        return NAV_TURNING, 0, 0
    sim = Simulation(state)
    with pytest.raises((RuntimeError, ConnectionError, TimeoutError)):
        sim.run()
    assert all(c == sim.sent[0] for c in sim.sent)


def test_real_firmware_profile(tmp_path):
    source = tmp_path / "turn.c"
    source.write_text(r'''#include "turn_profile.h"
#include <assert.h>
int main(void) {
    for (int dir=-1; dir<=1; dir+=2) {
        turn_profile_t p;
        float start=dir>0 ? 170*TURN_PI/180 : -170*TURN_PI/180;
        assert(turn_profile_init(&p, 1, 2, -.5, start, turn_wrap(start+TURN_PI), dir*TURN_MAX_SPEED));
        float prev=start, velocity=0, yaw=start;
        for (int i=1; i<=290; i++) {
            float t=i*.05f;
            int result=turn_profile_tick(&p, t, yaw, true, &yaw);
            float v=turn_wrap(yaw-prev)/.05f;
            assert(dir*v >= -.0001f);
            assert(fabsf(v) <= TURN_MAX_SPEED+.0001f);
            assert(fabsf(v-velocity)/.05f <= TURN_ACCEL+.002f);
            assert(p.x==1 && p.y==2 && p.z==-.5f);
            assert(result>=0);
            prev=yaw; velocity=v;
        }
        assert(turn_profile_tick(&p, 16, p.target, true, &yaw)==1);
        assert(turn_profile_tick(&p, 17, p.target+0.2f, true, &yaw)==0);
        assert(turn_profile_tick(&p, 18, p.target, true, &yaw)==0);
        assert(turn_profile_tick(&p, 19.1f, p.target, true, &yaw)==1);
        assert(turn_profile_tick(&p, 20, p.target, false, &yaw)==-1);
        assert(turn_profile_tick(&p, 30, p.target, true, &yaw)==-1);
    }
    turn_profile_t p;
    assert(!turn_profile_init(&p,0,0,0,0,NAN,1));
    assert(!turn_profile_init(&p,0,0,0,0,1,0));
    assert(turn_profile_init(&p,0,0,0,0,.01f,TURN_MAX_SPEED*2));
    assert(p.speed==TURN_MAX_SPEED);
    return 0;
}
''')
    main = Path(__file__).resolve().parents[2] / "main"
    binary = tmp_path / "turn"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(main),
                    str(source), "-lm", "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_firmware_turn_lifecycle(tmp_path):
    """Execute the actual firmware entry/tick/cancel functions with fake PX4/clock."""
    import re
    main = Path(__file__).resolve().parents[2] / "main"
    nav = (main / "nav_task.c").read_text()
    def function(name):
        start = re.search(r"^(?:static )?(?:void|bool) " + name + r"\(", nav, re.M).start()
        brace = nav.index("{", start)
        depth = 1
        end = brace + 1
        while depth:
            depth += (nav[end] == "{") - (nav[end] == "}")
            end += 1
        return nav[start:end]
    enum = re.search(r"typedef enum \{.*?\} nav_state_t;", (main / "nav_task.h").read_text(), re.S).group()
    internal = re.search(r"typedef struct \{.*?\} nav_internal_t;", nav, re.S).group()
    harness = r'''#include <assert.h>
#include <stdint.h>
#include "turn_profile.h"
#define portMAX_DELAY 0
#define PX4_MAIN_MODE_OFFBOARD 6
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
static int s_mutex, locked, writes, holds;
static int64_t now;
static bool fresh=true;
typedef struct { float x,y,z,heading; bool armed; int custom_main_mode; } drone_state_t;
static drone_state_t state={1,2,-.5,0,true,6};
static float out_x,out_y,out_z,out_yaw;
static void xSemaphoreTake(int m, int wait) { (void)m; (void)wait; assert(!locked); locked=1; }
static void xSemaphoreGive(int m) { (void)m; assert(locked); locked=0; }
static drone_state_t mavlink_get_state(void) { return state; }
static bool mavlink_position_valid(void) { return fresh; }
static int64_t esp_timer_get_time(void) { return now; }
static void mavlink_set_hold(void) { assert(locked); holds++; }
static void mavlink_set_position_ned(float x,float y,float z,float yaw) {
    assert(locked); writes++; out_x=x;out_y=y;out_z=z;out_yaw=yaw;
}
'''
    harness += enum + internal + r'''
static nav_internal_t s_nav;
static struct {nav_state_t state;} s_status;
static turn_profile_t s_turn;
static int64_t s_turn_t0_us;
static bool s_turn_command_valid;
static float s_turn_command_yaw,s_turn_command_speed;
'''
    # status heading field is used by the real tick function
    harness = harness.replace("struct {nav_state_t state;}", "struct {nav_state_t state; float heading_error_rad;}")
    for name in ("turn_tick_locked", "nav_turn_start", "nav_cancel", "nav_set_goal_map"):
        harness += function(name) + "\n"
    harness += r'''
static void tick(void) {xSemaphoreTake(0,0);turn_tick_locked();xSemaphoreGive(0);}
int main(void) {
    nav_turn_start(TURN_PI, TURN_MAX_SPEED);
    assert(s_nav.state==NAV_TURNING && out_x==1 && out_y==2 && out_z==-.5f);
    now=1000000;tick();
    float first=out_yaw;
    /* Retrying must keep the original capture/time even if the drone drifts. */
    state.x=1.2;state.y=2.2;state.z=-.6;
    nav_turn_start(TURN_PI, TURN_MAX_SPEED);
    assert(s_turn_t0_us==0 && s_turn.x==1);
    tick();assert(out_yaw==first && out_x==1 && out_y==2 && out_z==-.5f);
    state.heading=TURN_PI;now=15000000;tick();now=16100000;tick();
    assert(s_nav.state==NAV_ARRIVED);
    int n=writes;nav_turn_start(TURN_PI,TURN_MAX_SPEED);assert(writes==n);
    nav_cancel();
    nav_turn_start(0, -TURN_MAX_SPEED);assert(s_nav.state==NAV_TURNING);
    fresh=false;tick();assert(s_nav.state==NAV_TURN_FAILED && holds>0);fresh=true;
    nav_cancel();nav_turn_start(0, -TURN_MAX_SPEED);
    state.armed=false;tick();assert(s_nav.state==NAV_TURN_FAILED);state.armed=true;
    nav_cancel();nav_turn_start(0, -TURN_MAX_SPEED);
    state.custom_main_mode=3;tick();assert(s_nav.state==NAV_TURN_FAILED);state.custom_main_mode=6;
    nav_cancel();nav_turn_start(0, -TURN_MAX_SPEED);
    now+=30000000;tick();assert(s_nav.state==NAV_TURN_FAILED);
    nav_cancel();nav_turn_start(0, -TURN_MAX_SPEED);nav_cancel();
    n=writes;tick();assert(writes==n && s_nav.state==NAV_IDLE);
    nav_turn_start(0,-TURN_MAX_SPEED);nav_set_goal_map(3,4,-.5);
    n=writes;tick();assert(writes==n && s_nav.state==NAV_ROTATING);
    nav_cancel();fresh=false;nav_turn_start(0,TURN_MAX_SPEED);
    assert(s_nav.state==NAV_TURN_FAILED);
    return 0;
}
'''
    source = tmp_path / "lifecycle.c"
    source.write_text(harness)
    binary = tmp_path / "lifecycle"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(main),
                    str(source), "-lm", "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
