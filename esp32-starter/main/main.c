#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "wifi_task.h"
#include "odom.h"
#include "xy_recovery.h"
#include "esp_timer.h"

#include <math.h>

static const char *TAG = "mission";

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define CRUISE_ALT_M        0.5f    /* target altitude above takeoff (m)        */
#define ARM_TIMEOUT_S       10      /* give up on OFFBOARD + arm (s)            */
#define TAKEOFF_TIMEOUT_S   10      /* max seconds to wait for altitude (s)     */
#define LAND_TIMEOUT_S      20      /* warn if still armed after landing (s)    */
#define ALT_TOLERANCE_M     0.15f   /* altitude band considered "at altitude"   */

/* Consume a pending CMD_LAND. */
static bool land_requested(void)
{
    if (!wifi_land_requested()) return false;
    wifi_clear_land_request();
    wifi_clear_home_land_request();
    wifi_clear_hold_request();
    ESP_LOGI(TAG, "CMD_LAND received from laptop");
    return true;
}

/* Fresh local position/attitude and heartbeat are required for recovery. */
static bool recovery_telemetry_valid(drone_state_t st)
{
    uint32_t ms=(uint32_t)(esp_timer_get_time()/1000);
    return mavlink_position_valid() && (uint32_t)(ms-st.last_att_ms)<500
        && (uint32_t)(ms-st.last_hb_ms)<1500
        && isfinite(st.x) && isfinite(st.y) && isfinite(st.z)
        && isfinite(st.heading) && isfinite(st.vx) && isfinite(st.vy) && isfinite(st.vz);
}

/* One snapshot per second, plus failure snapshots: keep UART logging bounded. */
static void log_alignment_snapshot(const char *stage, drone_state_t st,
                                    float home_x, float home_y, float target_z)
{
    uint32_t ms=(uint32_t)(esp_timer_get_time()/1000);
    float error=hypotf(st.x-home_x,st.y-home_y);
    float speed=sqrtf(st.vx*st.vx+st.vy*st.vy+st.vz*st.vz);
    ESP_LOGI(TAG,"ALIGN %s: armed=%d mode=%u wifi=%d pos_valid=%d "
             "age_ms(pos/att/hb)=%lu/%lu/%lu XY=(%.3f,%.3f) "
             "home=(%.3f,%.3f) xy_error=%.3f z=%.3f target_z=%.3f "
             "z_error=%.3f speed=%.3f",
             stage,st.armed,(unsigned)st.custom_main_mode,wifi_is_connected(),
             mavlink_position_valid(),(unsigned long)(ms-st.last_pos_ms),
             (unsigned long)(ms-st.last_att_ms),(unsigned long)(ms-st.last_hb_ms),
             st.x,st.y,home_x,home_y,error,st.z,target_z,fabsf(st.z-target_z),speed);
    /* Also compile with test log macros that discard arguments. */
    (void)stage; (void)ms; (void)error; (void)speed; (void)target_z;
}

typedef enum { XY_DONE, XY_FAILED, XY_LAND_NOW, XY_RELEASED } xy_result_t;

/* Captured launch coordinates are immutable for this flight. Recovery uses a
 * slow interpolated position target, then waits for measured position/speed. */
static xy_result_t recover_launch_xy(float home_x, float home_y, float hold_z,
                                      nav_state_t status)
{
    drone_state_t start=mavlink_get_state();
    float distance=hypotf(start.x-home_x,start.y-home_y);
    nav_cancel();
    nav_set_mission_status(status,distance);
    ESP_LOGI(TAG, "XY check: actual(%.3f,%.3f), launch(%.3f,%.3f), error=%.3f m",
             start.x,start.y,home_x,home_y,distance);
    log_alignment_snapshot("recovery entry",start,home_x,home_y,hold_z);
    if (!recovery_telemetry_valid(start) || !isfinite(hold_z)
        || distance>XY_RECOVERY_MAX_M) {
        ESP_LOGE(TAG,"ALIGN failed at recovery entry: telemetry_valid=%d target_finite=%d distance=%.3f max=%.3f",
                 recovery_telemetry_valid(start),isfinite(hold_z),distance,XY_RECOVERY_MAX_M);
        return XY_FAILED;
    }
    float duration=xy_recovery_duration(distance);
    int64_t begin=esp_timer_get_time(), settled=0, next_log=begin;
    uint32_t last_pos_ms=start.last_pos_ms;
    while ((esp_timer_get_time()-begin)*1e-6f<XY_RECOVERY_TIMEOUT_S) {
        if (land_requested()) return XY_LAND_NOW;
        if (wifi_hold_requested()) {
            ESP_LOGE(TAG,"ALIGN failed: CMD_HOLD interrupted recovery");
            return XY_FAILED;
        }
        drone_state_t st=mavlink_get_state();
        if (!st.armed || st.custom_main_mode!=PX4_MAIN_MODE_OFFBOARD) {
            ESP_LOGW(TAG,"ALIGN released: disarmed or left OFFBOARD");
            log_alignment_snapshot("released",st,home_x,home_y,hold_z);
            return XY_RELEASED;
        }
        const char *failure=NULL;
        if (!wifi_is_connected()) failure="Wi-Fi disconnected";
        else if (!recovery_telemetry_valid(st)) failure="stale or non-finite PX4 telemetry";
        else if (hypotf(st.x-home_x,st.y-home_y)>XY_RECOVERY_MAX_M) failure="XY error exceeds maximum recovery distance";
        else if (fabsf(st.z-hold_z)>0.3f) failure="altitude error exceeds 0.30 m during recovery";
        if (failure) {
            ESP_LOGE(TAG,"ALIGN failed: %s",failure);
            log_alignment_snapshot("failed",st,home_x,home_y,hold_z);
            return XY_FAILED;
        }
        int64_t now=esp_timer_get_time();
        float elapsed=(now-begin)*1e-6f;
        float h=xy_recovery_fraction(elapsed,duration);
        mavlink_set_position_ned(start.x+h*(home_x-start.x),
                                 start.y+h*(home_y-start.y),hold_z,start.heading);
        float error=hypotf(st.x-home_x,st.y-home_y);
        nav_set_mission_status(status,error);
        float radius_m = status == NAV_TAKEOFF_ALIGN
            ? TAKEOFF_ALIGNMENT_TOL_M : XY_RECOVERY_TOL_M;
        bool aligned=elapsed>=duration && xy_recovery_settled_with_radius(st.x,st.y,st.z,
                        st.vx,st.vy,st.vz,home_x,home_y,hold_z,radius_m);
        if (now>=next_log) {
            log_alignment_snapshot("recovering",st,home_x,home_y,hold_z);
            ESP_LOGI(TAG,"ALIGN settling: radius=%.2f profile_done=%d within_xy=%d within_z=%d slow=%d settled_ms=%ld",
                     radius_m,elapsed>=duration,error<=radius_m,
                     fabsf(st.z-hold_z)<=0.15f,
                     sqrtf(st.vx*st.vx+st.vy*st.vy+st.vz*st.vz)<=XY_RECOVERY_SETTLE_M_S,
                     (long)(settled ? (now-settled)/1000 : 0));
            next_log=now+1000000;
        }
        if (!aligned) settled=0;
        else if (st.last_pos_ms!=last_pos_ms) {
            if (!settled) settled=now;
            if (now-settled>=XY_RECOVERY_SETTLE_US) {
                ESP_LOGI(TAG,"Launch XY restored and settled: error=%.3f m",error);
                return XY_DONE;
            }
        }
        last_pos_ms=st.last_pos_ms;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGE(TAG,"ALIGN failed: recovery timed out after %.1f s; check XY, Z, speed and fresh samples",
             XY_RECOVERY_TIMEOUT_S);
    log_alignment_snapshot("timeout",mavlink_get_state(),home_x,home_y,hold_z);
    return XY_FAILED;
}

/* Do not silently continue the mission or claim a home landing on failure.
 * Immediate LAND and RC takeover remain available while recovery is stopped. */
static void hold_failed_recovery(void)
{
    nav_cancel();
    mavlink_set_hold();
    nav_set_mission_status(NAV_ALIGN_FAILED,0.0f);
    ESP_LOGE(TAG,"Takeoff/alignment failed: holding; see preceding TAKEOFF/ALIGN reason; immediate LAND or pilot action required");
    while (mavlink_get_state().armed
           && mavlink_get_state().custom_main_mode==PX4_MAIN_MODE_OFFBOARD) {
        if (land_requested()) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------------------------------------------------------------------
 * Mission task: START → OFFBOARD → arm → takeoff → laptop control → LAND,
 * repeat.  Owns the setpoint until a GOTO/trajectory; nav_cancel() returns it.
 * --------------------------------------------------------------------------- */
static void mission_task(void *arg)
{
    float target_z   = -(CRUISE_ALT_M);   /* NED: negative = above ground */
    float takeoff_x  = 0.0f;
    float takeoff_y  = 0.0f;
    float takeoff_heading = 0.0f;

    for (;;) {

    /* ------------------------------------------------------------------ */
    /* Phase 1: Wait for valid telemetry                                   */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for telemetry...");
    while (1) {
        drone_state_t st = mavlink_get_state();
        if (mavlink_position_valid() && st.last_hb_ms != 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Telemetry valid");

    /* ------------------------------------------------------------------ */
    /* Phase 2: Pre-stream hold setpoint                                   */
    /* PX4 requires setpoints to already be streaming before it will       */
    /* accept an OFFBOARD mode switch.  Stream for 2 s.                    */
    /* ------------------------------------------------------------------ */
    mavlink_set_hold();
    nav_cancel();                 /* drop stale goal + commands */
    wifi_clear_start_request();
    wifi_clear_land_request();
    wifi_clear_home_land_request();
    wifi_clear_hold_request();
    wifi_set_mission_phase(MISSION_READY);
    ESP_LOGI(TAG, "Pre-streaming hold setpoint for 2 s...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ------------------------------------------------------------------ */
    /* Phase 2b: Wait for CMD_START (wifi_task switches to BUSY on it)     */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for CMD_START from laptop...");
    while (!wifi_start_requested()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    wifi_clear_start_request();
    ESP_LOGI(TAG, "CMD_START received");

    /* Pre-arm: refuse to arm until all ToF sensors are up. */
#if TOF_ENABLED
    {
        ESP_LOGI(TAG, "Checking ToF sensors...");
        int tof_ok = tof_sensors_ok_count();
        while (tof_ok < TOF_SENSOR_COUNT) {
            ESP_LOGE(TAG, "TOF CHECK FAILED: %d / %d sensors OK — refusing to arm",
                     tof_ok, TOF_SENSOR_COUNT);
            vTaskDelay(pdMS_TO_TICKS(2000));
            tof_ok = tof_sensors_ok_count();
        }
        ESP_LOGI(TAG, "All %d ToF sensors OK — proceeding to arm", tof_ok);
    }
#else
    ESP_LOGW(TAG, "ToF disabled (TOF_ENABLED=0) — skipping pre-arm sensor check");
#endif

    /* ------------------------------------------------------------------ */
    /* Phase 3: OFFBOARD, then arm — retry every 500 ms.                   */
    /* CMD_LAND or ARM_TIMEOUT_S aborts and returns to waiting.            */
    /* ------------------------------------------------------------------ */
    drone_state_t launch=mavlink_get_state();
    if (!recovery_telemetry_valid(launch) || launch.armed) {
        ESP_LOGW(TAG,"START refused: need fresh telemetry and disarmed launch reference");
        continue;
    }
    takeoff_x=launch.x;
    takeoff_y=launch.y;
    takeoff_heading=launch.heading;
    ESP_LOGI(TAG,"Saved pre-arm launch XY=(%.3f,%.3f)",takeoff_x,takeoff_y);

    const char *abort_why = NULL;
    int tries = ARM_TIMEOUT_S * 2;

    ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
    while (mavlink_get_state().custom_main_mode != PX4_MAIN_MODE_OFFBOARD) {
        if (land_requested())  { abort_why = "CMD_LAND"; break; }
        if (tries-- == 0)      { abort_why = "OFFBOARD refused (check QGC)"; break; }
        mavlink_set_offboard_mode();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!abort_why) {
        ESP_LOGI(TAG, "OFFBOARD mode confirmed");
        ESP_LOGI(TAG, "Arming...");
    }
    while (!abort_why && !mavlink_get_state().armed) {
        if (land_requested())  { abort_why = "CMD_LAND"; break; }
        if (tries-- == 0)      { abort_why = "arming refused (check QGC)"; break; }
        mavlink_arm(true);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (abort_why) {
        mavlink_arm(false);   /* in case the arm just went through */
        ESP_LOGW(TAG, "Takeoff aborted: %s", abort_why);
        continue;
    }
    ESP_LOGI(TAG, "Armed confirmed");

    /* Phase 4: vertical takeoff at the immutable pre-arm launch XY. */
    nav_set_mission_status(NAV_TAKEOFF_ALIGN,0.0f);
    mavlink_set_position_ned(takeoff_x,takeoff_y,target_z,takeoff_heading);
    ESP_LOGI(TAG,"Taking off: launch XY=(%.3f,%.3f), NED z=%.2f",
             takeoff_x,takeoff_y,target_z);
    bool land_now=false, altitude_reached=false;
    ESP_LOGI(TAG,"TAKEOFF altitude gate: NED target_z=%.3f tolerance=%.3f timeout=%d s; XY radius=%.2f m applies AFTER altitude gate",
             target_z,ALT_TOLERANCE_M,TAKEOFF_TIMEOUT_S,TAKEOFF_ALIGNMENT_TOL_M);
    log_alignment_snapshot("takeoff begin",mavlink_get_state(),takeoff_x,takeoff_y,target_z);
    for (int i=0;i<TAKEOFF_TIMEOUT_S*10;i++) {
        drone_state_t st=mavlink_get_state();
        if (land_requested()) { land_now=true; break; }
        if (wifi_hold_requested()) {
            ESP_LOGE(TAG,"TAKEOFF interrupted: CMD_HOLD");
            break;
        }
        if (!st.armed || st.custom_main_mode!=PX4_MAIN_MODE_OFFBOARD) {
            ESP_LOGW(TAG,"TAKEOFF released: disarmed or left OFFBOARD");
            break;
        }
        if (i%10==0)
            log_alignment_snapshot("waiting for altitude",st,takeoff_x,takeoff_y,target_z);
        if (recovery_telemetry_valid(st) && fabsf(st.z-target_z)<ALT_TOLERANCE_M) {
            ESP_LOGI(TAG,"TAKEOFF altitude gate passed; starting XY recovery");
            altitude_reached=true; break;
        }
        if (i==TAKEOFF_TIMEOUT_S*10-1) {
            ESP_LOGE(TAG,"TAKEOFF failed: altitude gate timed out (requires altitude AND fresh finite telemetry)");
            log_alignment_snapshot("altitude timeout",st,takeoff_x,takeoff_y,target_z);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    xy_result_t takeoff_result=XY_RELEASED;
    if (!land_now && altitude_reached)
        takeoff_result=recover_launch_xy(takeoff_x,takeoff_y,target_z,NAV_TAKEOFF_ALIGN);
    else if (!land_now && mavlink_get_state().armed
             && mavlink_get_state().custom_main_mode==PX4_MAIN_MODE_OFFBOARD)
        takeoff_result=XY_FAILED;
    if (takeoff_result==XY_LAND_NOW) land_now=true;
    if (takeoff_result==XY_FAILED) hold_failed_recovery();

    /* Phase 5: accept laptop movement only once launch XY/altitude settle. */
    bool normal_home_land=false;
    if (!land_now && takeoff_result==XY_DONE) {
        nav_set_mission_status(NAV_FLIGHT_READY,0.0f);
        wifi_set_mission_phase(MISSION_FLYING);
        ESP_LOGI(TAG,"Takeoff XY verified — ready for laptop mission");
        while (1) {
            if (land_requested()) { land_now=true; break; }
            if (wifi_home_land_requested()) { normal_home_land=true; break; }
            if (!mavlink_get_state().armed
                || mavlink_get_state().custom_main_mode!=PX4_MAIN_MODE_OFFBOARD) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    wifi_set_mission_phase(MISSION_BUSY);
    wifi_clear_home_land_request();
    if (normal_home_land && !land_now) {
        float landing_z=mavlink_get_state().z; /* correct XY at current altitude */
        xy_result_t result=recover_launch_xy(takeoff_x,takeoff_y,landing_z,NAV_HOME_ALIGN);
        if (result==XY_DONE) nav_set_mission_status(NAV_HOME_READY,0.0f);
        else if (result==XY_FAILED) hold_failed_recovery();
    }

    /* ------------------------------------------------------------------ */
    /* Phase 6: Land, wait for disarm.  LAND is (re)sent every 2 s only    */
    /* while in OFFBOARD, so an RC takeover is never overridden.           */
    /* ------------------------------------------------------------------ */
    if (nav_get_status().state != NAV_HOME_READY) nav_cancel();
    for (int i = 0; mavlink_get_state().armed; i++) {
        bool offboard = mavlink_get_state().custom_main_mode == PX4_MAIN_MODE_OFFBOARD;
        if (i % 20 == 0 && offboard) {
            mavlink_send_land_command();
            if (i == 0) ESP_LOGI(TAG, "Land command sent");
        }
        if (i == 0 && !offboard) ESP_LOGW(TAG, "Not in OFFBOARD — RC has control, LAND not sent");
        if (i == LAND_TIMEOUT_S * 10) {
            ESP_LOGW(TAG, "Still armed after %d s — land with RC", LAND_TIMEOUT_S);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Disarmed — mission complete");

    ESP_LOGI(TAG, "Mission loop complete — waiting for next CMD_START");
    }
}

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all tasks
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    /* NVS must be init before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Init all modules before spawning tasks */
    mavlink_task_init();
    tof_task_init();    /* always: creates the scan mutex the tof_get_*() accessors take */
    nav_task_init();
    at_detect_init();
    odom_init();
    wifi_task_init();   /* blocks until IP obtained */

    /* Core 0: hardware-facing tasks + WiFi telemetry */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
#if TOF_ENABLED
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );
#endif
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", WIFI_TASK_STACK,
        NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE
    );

    /* Core 1: navigator + AprilTag + mission
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        at_detect_task, "apriltag", AT_TASK_STACK,
        NULL, AT_TASK_PRIORITY, NULL, AT_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 6144,
        NULL, 2, NULL, 1
    );

    ESP_LOGI(TAG, "All tasks spawned");
}
