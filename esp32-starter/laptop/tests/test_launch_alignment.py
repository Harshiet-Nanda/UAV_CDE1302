import math
import struct
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from launch_alignment import wait_launch_alignment
from protocol import (CMD_HOME_LAND, CommandPacket, build_command, NAV_TAKEOFF_ALIGN,
                      NAV_FLIGHT_READY, NAV_HOME_ALIGN, NAV_HOME_READY, NAV_ALIGN_FAILED)


class Flight:
    def __init__(self, scenario):
        self.t = 0.0
        self.scenario = scenario
        self.sent = []

    def sample(self):
        state, received = self.scenario(self.t)
        return SimpleNamespace(nav_state=state), received

    def sleep(self, dt):
        self.t += dt

    def wait(self, **kwargs):
        return wait_launch_alignment(self.sample, clock=lambda: self.t,
                                     sleep=self.sleep,
                                     send=lambda *x: self.sent.append(x), **kwargs)


def test_normal_landing_wire_format():
    raw = build_command(CommandPacket(CMD_HOME_LAND))
    assert struct.unpack_from('<BB', raw) == (2, 11)
    assert len(raw) == 22


@pytest.mark.parametrize('landing', [False, True])
def test_fresh_alignment_ack_and_completion_required(landing):
    active, done = (NAV_HOME_ALIGN, NAV_HOME_READY) if landing else (NAV_TAKEOFF_ALIGN, NAV_FLIGHT_READY)
    f = Flight(lambda t: (active if t < 2 else done, t))
    assert f.wait(landing=landing).nav_state == done
    assert f.t >= 2
    assert bool(f.sent) == landing


def test_minimum_takeoff_delay_keeps_watching_telemetry():
    f = Flight(lambda t: (NAV_TAKEOFF_ALIGN if t < 1 else NAV_FLIGHT_READY, t))
    f.wait(min_wait=3)
    assert f.t >= 3


@pytest.mark.parametrize('problem,match', [('stale','telemetry lost'),
    ('failed','correction failed'), ('interrupted','interrupted'), ('timeout','timed out')])
def test_takeoff_cannot_start_path_on_failure(problem, match):
    def scenario(t):
        if t < 0.5: return NAV_TAKEOFF_ALIGN,t
        return {'stale':(NAV_TAKEOFF_ALIGN,0.5), 'failed':(NAV_ALIGN_FAILED,t),
                'interrupted':(0,t), 'timeout':(NAV_TAKEOFF_ALIGN,t)}[problem]
    f=Flight(scenario)
    with pytest.raises(RuntimeError,match=match): f.wait(timeout=3)


def test_old_firmware_or_old_ready_state_cannot_complete_landing():
    f=Flight(lambda t:(NAV_HOME_READY,t))
    with pytest.raises(RuntimeError,match='acknowledgement'): f.wait(landing=True)


def test_xy_profile_and_actual_recovery_with_mocked_vehicle(tmp_path):
    main=Path(__file__).resolve().parents[2]/'main'
    source=(main/'main.c').read_text()
    def function(signature):
        start=source.index(signature); brace=source.index('{',start)
        depth=1; end=brace+1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        return source[start:end]
    harness=r"""
#include <stdint.h>
#include <assert.h>
#include "xy_recovery.h"
#define PX4_MAIN_MODE_OFFBOARD 6
#define pdMS_TO_TICKS(x) (x)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
typedef int nav_state_t;
enum {NAV_TAKEOFF_ALIGN=9,NAV_FLIGHT_READY,NAV_HOME_ALIGN,NAV_HOME_READY,NAV_ALIGN_FAILED};
typedef struct {
 float x,y,z,vx,vy,vz,heading;
 bool armed;
 int custom_main_mode;
 uint32_t last_pos_ms,last_att_ms,last_hb_ms;
} drone_state_t;
static drone_state_t vehicle;
static int64_t now_us;
static bool immediate_land,hold,connected=true,valid=true,follow=true,freeze_pos=false;
static int status;
static int mode;
static float cmd_x,cmd_y,cmd_z,previous_x,previous_y,max_rate;
static bool wifi_land_requested(void){return immediate_land;}
static void wifi_clear_land_request(void){immediate_land=false;}
static void wifi_clear_home_land_request(void){}
static void wifi_clear_hold_request(void){hold=false;}
static bool wifi_hold_requested(void){return hold;}
static bool wifi_is_connected(void){return connected;}
static drone_state_t mavlink_get_state(void){return vehicle;}
static bool mavlink_position_valid(void){return valid;}
static int64_t esp_timer_get_time(void){return now_us;}
static void nav_cancel(void){}
static void nav_set_mission_status(int s,float error){(void)error;status=s;}
static void mavlink_set_position_ned(float x,float y,float z,float yaw){
 assert(fabsf(z+0.5f)<1e-6f);assert(yaw==0.3f);
 float speed=hypotf(x-previous_x,y-previous_y)/0.05f;
 if(speed>max_rate)max_rate=speed;
 previous_x=x;previous_y=y;cmd_x=x;cmd_y=y;cmd_z=z;
}
static void vTaskDelay(int ms){
 now_us+=(int64_t)ms*1000;
 if(follow){vehicle.x=cmd_x;vehicle.y=cmd_y;vehicle.z=cmd_z;}
 if(!freeze_pos)vehicle.last_pos_ms=now_us/1000;
 vehicle.last_att_ms=vehicle.last_hb_ms=now_us/1000;
 if(mode==1 && now_us>1500000)immediate_land=true;
 if(mode==2 && now_us>1500000)hold=true;
 if(mode==3 && now_us>1500000)vehicle.custom_main_mode=3;
 if(mode==4 && now_us>1500000)connected=false;
}
"""
    harness+=function('static bool land_requested(void)')
    harness+=function('static bool recovery_telemetry_valid(drone_state_t st)')
    harness+='typedef enum { XY_DONE, XY_FAILED, XY_LAND_NOW, XY_RELEASED } xy_result_t;\n'
    harness+=function('static xy_result_t recover_launch_xy(')
    harness+=r"""
static void reset(void){
 now_us=1000000;mode=0;immediate_land=hold=false;
 connected=valid=follow=true;freeze_pos=false;max_rate=0;
 vehicle=(drone_state_t){.x=3.4f,.y=-1.8f,.z=-0.5f,.heading=0.3f,
    .armed=true,.custom_main_mode=6,.last_pos_ms=1000,.last_att_ms=1000,.last_hb_ms=1000};
 previous_x=cmd_x=vehicle.x;previous_y=cmd_y=vehicle.y;cmd_z=vehicle.z;
}
int main(void){
 reset();assert(recover_launch_xy(3,-2,-0.5f,NAV_TAKEOFF_ALIGN)==XY_DONE);
 assert(status==NAV_TAKEOFF_ALIGN);assert(hypotf(vehicle.x-3,vehicle.y+2)<=0.1f);
 assert(max_rate<=0.1001f);assert(now_us>1000000+1000000);
 reset();assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_DONE);
 assert(status==NAV_HOME_ALIGN);assert(max_rate<=0.1001f);
 reset();follow=false;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();vehicle.vx=0.2f;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();freeze_pos=true;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();vehicle.x=5;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();valid=false;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();vehicle.heading=NAN;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();mode=1;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_LAND_NOW);
 reset();mode=2;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 reset();mode=3;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_RELEASED);
 reset();mode=4;assert(recover_launch_xy(3,-2,-0.5f,NAV_HOME_ALIGN)==XY_FAILED);
 // Within position tolerance but still moving is not settled.
 assert(!xy_recovery_settled(3,-2,-0.5f,0.2f,0,0,3,-2,-0.5f));
 assert(!xy_recovery_settled(3,-2,-0.1f,0,0,0,3,-2,-0.5f));
 assert(!xy_recovery_settled(NAN,-2,-0.5f,0,0,0,3,-2,-0.5f));
 return 0;
}
"""
    cfile=tmp_path/'xy_recovery.c'; cfile.write_text(harness)
    binary=tmp_path/'xy_recovery'
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(main),
                    str(cfile),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)



def test_actual_mission_captures_before_arm_and_gates_normal_landing(tmp_path):
    main=Path(__file__).resolve().parents[2]/'main'
    source=(main/'main.c').read_text()
    mission=source[source.index('#define CRUISE_ALT_M'):source.index('void app_main(void)')]
    harness=r"""
#include <stdint.h>
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "xy_recovery.h"
#define PX4_MAIN_MODE_OFFBOARD 6
#define pdMS_TO_TICKS(x) (x)
#define TOF_ENABLED 0
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
typedef int nav_state_t;
enum {NAV_IDLE=0,NAV_TAKEOFF_ALIGN=9,NAV_FLIGHT_READY,NAV_HOME_ALIGN,
      NAV_HOME_READY,NAV_ALIGN_FAILED};
enum {MISSION_BUSY,MISSION_READY,MISSION_FLYING};
typedef struct {
 float x,y,z,vx,vy,vz,heading;
 bool armed;
 int custom_main_mode;
 uint32_t last_pos_ms,last_att_ms,last_hb_ms;
} drone_state_t;
static drone_state_t vehicle;
static int64_t now_us;
static int state,phase,scenario,ready_count,land_calls;
static bool start_request,land_request,home_request,hold_request;
static bool injected,first_climb,flight_ready,home_ready,failed,home_active;
static float cmd_x,cmd_y,cmd_z,home_x,home_y;
static jmp_buf finished;
static int64_t esp_timer_get_time(void){return now_us;}
static drone_state_t mavlink_get_state(void){return vehicle;}
static bool mavlink_position_valid(void){return true;}
static bool wifi_is_connected(void){return true;}
static bool wifi_land_requested(void){return land_request;}
static void wifi_clear_land_request(void){land_request=false;}
static void wifi_clear_home_land_request(void){home_request=false;}
static bool wifi_home_land_requested(void){return home_request;}
static bool wifi_hold_requested(void){return hold_request;}
static void wifi_clear_hold_request(void){hold_request=false;}
static bool wifi_start_requested(void){return start_request;}
static void wifi_clear_start_request(void){start_request=false;}
static void nav_cancel(void){state=NAV_IDLE;}
static void mavlink_set_hold(void){cmd_x=vehicle.x;cmd_y=vehicle.y;cmd_z=vehicle.z;}
static void nav_set_mission_status(int status,float error){
 (void)error;state=status;
 if(status==NAV_FLIGHT_READY){
  assert(hypotf(vehicle.x-home_x,vehicle.y-home_y)<=0.1f);flight_ready=true;
 }
 if(status==NAV_HOME_ALIGN)home_active=true;
 if(status==NAV_HOME_READY){
  assert(hypotf(vehicle.x-home_x,vehicle.y-home_y)<=0.1f);home_ready=true;
 }
 if(status==NAV_ALIGN_FAILED){failed=true;land_request=true;}
}
static struct {int state;} nav_get_status(void);
/* C anonymous return structs are distinct; define the status type explicitly. */
"""
    harness=harness.replace('static struct {int state;} nav_get_status(void);',
        'typedef struct {int state;} nav_status_t;\nstatic nav_status_t nav_get_status(void){return (nav_status_t){state};}')
    harness+=r"""
static void wifi_set_mission_phase(int value){
 phase=value;
 if(value==MISSION_READY){if(++ready_count>1)longjmp(finished,1);}
 if(value==MISSION_FLYING){
  assert(flight_ready && first_climb);
  vehicle.x=home_x+0.4f;vehicle.y=home_y+0.1f;
  if(scenario==1)land_request=true;else home_request=true;
 }
}
static void mavlink_set_offboard_mode(void){vehicle.custom_main_mode=6;}
static void mavlink_arm(bool arm){vehicle.armed=arm;if(arm)vehicle.x+=0.2f;}
static void mavlink_set_position_ned(float x,float y,float z,float yaw){
 (void)yaw;
 if(!first_climb && vehicle.armed && z<0){
  // Actual position has shifted during arming: target must use PRE-arm XY.
  assert(x==home_x && y==home_y);first_climb=true;
 }
 cmd_x=x;cmd_y=y;cmd_z=z;
}
static void mavlink_send_land_command(void){
 land_calls++;
 if(scenario==0)assert(home_ready && state==NAV_HOME_READY);
 else assert(!home_ready);
 vehicle.armed=false;vehicle.custom_main_mode=4;
}
static void vTaskDelay(int ms){
 now_us+=(int64_t)ms*1000;
 if(phase==MISSION_READY)start_request=true;
 if(vehicle.armed){
  if(!(scenario==2 || (scenario==3 && flight_ready))){vehicle.x=cmd_x;vehicle.y=cmd_y;}
  vehicle.z=cmd_z;
  if(first_climb && !injected){vehicle.x=home_x+0.3f;vehicle.y=home_y+0.2f;injected=true;}
  if(scenario==4 && home_active)land_request=true;
 }
 vehicle.last_pos_ms=vehicle.last_att_ms=vehicle.last_hb_ms=now_us/1000;
 assert(now_us<150000000);
}
"""
    harness+=mission
    harness+=r"""
int main(void){
 for(scenario=0;scenario<5;scenario++){
  now_us=1000000;state=phase=ready_count=land_calls=0;
  start_request=land_request=home_request=hold_request=false;
  injected=first_climb=flight_ready=home_ready=failed=home_active=false;
  home_x=3.0f+scenario;home_y=-2.0f-scenario;
  vehicle=(drone_state_t){.x=home_x,.y=home_y,.heading=0.3f,
   .last_pos_ms=1000,.last_att_ms=1000,.last_hb_ms=1000};
  if(!setjmp(finished))mission_task(0);
  assert(first_climb && land_calls==1);
  if(scenario==0)assert(flight_ready && home_ready && !failed);
  if(scenario==1)assert(flight_ready && !home_active && !home_ready && !failed);
  if(scenario==2)assert(!flight_ready && failed);
  if(scenario==3)assert(flight_ready && failed && !home_ready);
  if(scenario==4)assert(flight_ready && home_active && !home_ready && !failed);
 }
 return 0;
}
"""
    cfile=tmp_path/'mission.c';cfile.write_text(harness)
    binary=tmp_path/'mission'
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                    '-Wno-unused-parameter','-Wno-unused-variable',
                    '-I',str(main),str(cfile),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
