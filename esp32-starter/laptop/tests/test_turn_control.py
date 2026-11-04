import ctypes
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from protocol import (CMD_HOLD, CMD_TURN, CommandPacket, build_command,
                      NAV_TURN, NAV_TURN_DONE, NAV_TURN_FAILED)
from turn_control import run_turn


class Flight:
    def __init__(self, scenario):
        self.t = 0.0
        self.scenario = scenario
        self.sent = []

    def sample(self):
        state, heading, x, received = self.scenario(self.t)
        return SimpleNamespace(nav_state=state, heading_rad=heading,
                               ned_x=x, ned_y=0.0), received

    def sleep(self, dt):
        self.t += dt

    def run(self, timeout=10):
        return run_turn(self.sample, lambda *args: self.sent.append(args), math.pi,
                        "left", clock=lambda: self.t, sleep=self.sleep, timeout=timeout)


def test_turn_wire_format():
    import struct
    raw = build_command(CommandPacket(CMD_TURN, goal_x=math.pi, goal_y=-1))
    kind, cmd, yaw, direction = struct.unpack_from("<BBff", raw)
    assert (kind, cmd, direction, len(raw)) == (2, 10, -1, 22)
    assert yaw == pytest.approx(math.pi)


def test_overshoot_must_be_corrected_before_reverse_pass():
    def scenario(t):
        # Misreported DONE while 20 degrees past target must not release gate.
        if t < 0.1: return 3, 0.0, 0.0, t
        if t < 0.5: return NAV_TURN, 1.0, 0.0, t
        if t < 3.0: return NAV_TURN_DONE, math.pi+math.radians(20), 0.0, t
        return NAV_TURN_DONE, -math.pi, 0.0, t
    f = Flight(scenario)
    assert f.run() == pytest.approx(math.pi)
    assert f.t >= 4.0
    assert f.sent[0][0] == CMD_TURN
    assert all(cmd[0] != CMD_HOLD for cmd in f.sent)


def test_alignment_must_be_continuous():
    def scenario(t):
        state = NAV_TURN if t < 0.5 or 1.0 < t < 1.5 else NAV_TURN_DONE
        return state, math.pi, 0.0, t
    f = Flight(scenario)
    f.run()
    assert f.t >= 2.5


@pytest.mark.parametrize("state", [3, NAV_TURN_DONE])
def test_old_firmware_or_stale_done_cannot_acknowledge(state):
    f = Flight(lambda t: (state, math.pi, 0.0, t))
    with pytest.raises(RuntimeError, match="acknowledgement"):
        f.run()
    assert f.sent[-1][0] == CMD_HOLD


@pytest.mark.parametrize("problem, message", [
    ("stale", "telemetry lost"), ("drift", "drift"),
    ("failed", "firmware turn failed"), ("interrupted", "interrupted"),
    ("nan", "invalid telemetry"), ("timeout", "did not settle")])
def test_failures_block_reverse_pass(problem, message):
    def scenario(t):
        state, heading, x, received = NAV_TURN, 0.0, 0.0, t
        if t > 0.5:
            if problem == "stale": received = 0.5
            elif problem == "drift": x = 0.6
            elif problem == "failed": state = NAV_TURN_FAILED
            elif problem == "interrupted": state = 0
            elif problem == "nan": heading = float("nan")
        return state, heading, x, received
    f = Flight(scenario)
    with pytest.raises(RuntimeError, match=message): f.run(timeout=3)
    assert f.sent[-1][0] == CMD_HOLD


def test_firmware_profile_limits_and_correction_gate(tmp_path):
    main = Path(__file__).resolve().parents[2] / "main"
    source = tmp_path / "profile.c"
    source.write_text('#include "turn_profile.h"\n'
                      'float yaw(float s,float d,float t){return turn_heading(s,d,t);}\n'
                      'float duration(float d){return turn_duration(d);}\n'
                      'int aligned(float h,float t,float r){return turn_aligned(h,t,r);}\n')
    libpath = tmp_path / "profile.so"
    subprocess.run(["cc", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror",
                    "-I", str(main), str(source), "-lm", "-o", str(libpath)], check=True)
    lib = ctypes.CDLL(str(libpath))
    lib.yaw.argtypes = [ctypes.c_float]*3
    lib.yaw.restype = ctypes.c_float
    lib.duration.argtypes = [ctypes.c_float]
    lib.duration.restype = ctypes.c_float
    lib.aligned.argtypes = [ctypes.c_float]*3
    for delta in (math.pi, -math.pi, 0.1, -0.1):
        duration = lib.duration(delta)
        dt = 0.05
        angles = [lib.yaw(6.1, delta, k*dt) for k in range(math.ceil(duration/dt)+2)]
        rates = [math.remainder(b-a, 2*math.pi)/dt for a,b in zip(angles, angles[1:])]
        assert max(abs(r) for r in rates) <= math.radians(20)+0.001
        accels = [(b-a)/dt for a,b in zip(rates, rates[1:])]
        assert max(abs(a) for a in accels) <= math.radians(10)+0.003
        assert all(r*delta >= -0.0001 for r in rates)
        assert abs(math.remainder(angles[-1]-(6.1+delta), 2*math.pi)) < 1e-5
    assert not lib.aligned(math.pi+0.2, math.pi, 0.0)  # overshoot
    assert not lib.aligned(math.pi, math.pi, 0.3)      # passing through target
    assert lib.aligned(-math.pi+0.01, math.pi, 0.0)   # wraparound



def test_firmware_turn_state_machine_with_mocked_vehicle(tmp_path):
    """Execute the actual turn functions against mocked telemetry/setpoint IO."""
    main = Path(__file__).resolve().parents[2] / "main"
    source = (main / "nav_task.c").read_text()

    def function(signature):
        start = source.index(signature)
        opening = source.index("{", start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        return source[start:end]

    state_start = source.index("static struct {", source.index("s_traj_t0_us"))
    state_end = source.index("} s_turn;", state_start) + len("} s_turn;")
    harness = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "turn_profile.h"
enum {NAV_IDLE, NAV_ROTATING, NAV_FLYING, NAV_ARRIVED, NAV_STUCK,
      NAV_TRAJ, NAV_TURN, NAV_TURN_DONE, NAV_TURN_FAILED};
#define PX4_MAIN_MODE_OFFBOARD 6
#define portMAX_DELAY 0
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
struct {int state; bool has_goal; float goal_z;} s_nav;
struct {int state; float heading_error_rad, dist_to_goal;} s_status;
static int s_mutex;
typedef struct {
 float x,y,z,vx,vy,vz,heading;
 bool armed;
 int custom_main_mode;
 uint32_t last_att_ms,last_hb_ms;
} drone_state_t;
static drone_state_t vehicle;
static int64_t now_us;
static float output_yaw, output_x, output_y, output_z;
static bool position_valid=true;
static int64_t esp_timer_get_time(void) {return now_us;}
static drone_state_t mavlink_get_state(void) {return vehicle;}
static bool mavlink_position_valid(void) {return position_valid;}
static void xSemaphoreTake(int m,int t) {(void)m;(void)t;}
static void xSemaphoreGive(int m) {(void)m;}
static void mavlink_set_position_ned(float x,float y,float z,float yaw)
{output_x=x;output_y=y;output_z=z;output_yaw=yaw;}
"""
    harness += source[state_start:state_end]
    harness += function("static void turn_tick(void)")
    harness += function("void nav_turn_start(float target_yaw, float direction)")
    harness += r"""
static void reset(void) {
 memset(&s_nav,0,sizeof(s_nav));
 memset(&vehicle,0,sizeof(vehicle));
 now_us=1000000; position_valid=true;
 vehicle.x=2;vehicle.y=3;vehicle.z=-0.5f;
 vehicle.armed=true;vehicle.custom_main_mode=6;
 vehicle.last_att_ms=vehicle.last_hb_ms=now_us/1000;
}
static void tick(float heading) {
 now_us+=50000;
 vehicle.last_att_ms=vehicle.last_hb_ms=now_us/1000;
 vehicle.heading=heading;
 turn_tick();
 assert(output_x==2 && output_y==3 && output_z==-0.5f);
}
int main(void) {
 for(int direction=-1;direction<=1;direction+=2) {
  reset();nav_turn_start(TURN_PI,direction);
  assert(s_nav.state==NAV_TURN);
  int64_t requested=s_turn.requested_us;
  for(int i=0;i<430;i++) {
   nav_turn_start(TURN_PI,direction); // UDP duplicates cannot restart profile
   tick(output_yaw);
   assert(s_turn.requested_us==requested);
  }
  assert(s_nav.state==NAV_TURN_DONE);
  assert(fabsf(turn_wrap(output_yaw-TURN_PI))<1e-5f);
  tick(TURN_PI+0.25f); // overshoot revokes completion
  assert(s_nav.state==NAV_TURN);
  for(int i=0;i<30;i++)tick(TURN_PI+0.25f);
  assert(s_nav.state==NAV_TURN); // waiting alone cannot accept heading error
  for(int i=0;i<30;i++)tick(TURN_PI);
  assert(s_nav.state==NAV_TURN_DONE);
 }
 reset();nav_turn_start(TURN_PI,1);
 for(int i=0;i<910;i++)tick(0); // heading never follows: fail, not DONE
 assert(s_nav.state==NAV_TURN_FAILED);
 reset();nav_turn_start(TURN_PI,1);vehicle.x+=0.6f;tick(0);
 assert(s_nav.state==NAV_TURN_FAILED);
 reset();nav_turn_start(TURN_PI,1);now_us+=600000;turn_tick();
 assert(s_nav.state==NAV_TURN_FAILED); // stale attitude is not success
 reset();nav_turn_start(NAN,1);assert(s_nav.state==NAV_IDLE);
 reset();vehicle.armed=false;nav_turn_start(TURN_PI,1);assert(s_nav.state==NAV_IDLE);
 reset();nav_turn_start(TURN_PI,1);s_nav.state=NAV_IDLE;
 output_yaw=0.123f;turn_tick();assert(output_yaw==0.123f); // cancellation wins
 return 0;
}
"""
    cfile = tmp_path / "turn_state.c"
    cfile.write_text(harness)
    executable = tmp_path / "turn_state"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(main), str(cfile), "-lm", "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
