#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "vfh.h"

/* ---------------------------------------------------------------------------
 * FreeRTOS placement — Core 1, Priority 3 (below MAVLink/ToF on Core 0)
 * --------------------------------------------------------------------------- */
#define NAV_TASK_CORE       1
#define NAV_TASK_PRIORITY   4
#define NAV_TASK_STACK      8192

/* ---------------------------------------------------------------------------
 * Tuning constants — adjust to vehicle
 *
 *  NAV_YAW_TOL_RAD     Heading error below which the drone is considered
 *                      "aligned" and switches from rotating to flying forward.
 *                      At 0.10 rad (~6°) the camera is well within frame.
 *
 *  NAV_STUCK_MIN_FREE  Min free VFH bins before exiting STUCK on retry.
 *                      2 avoids retrying on a single-bin noise artefact.
 * --------------------------------------------------------------------------- */
#define NAV_CRUISE_SPEED_MS     0.10f    /* forward cruise speed (m/s)            */
#define NAV_YAW_TOL_RAD         0.1f
#define NAV_ARRIVE_RADIUS_M     0.25f   /* XY goal-reached radius (m)            */
#define NAV_STUCK_HOLD_MS       3500    /* hold duration before declaring stuck   */
#define NAV_TRAJ_MAX_PTS        2400    /* uploaded trajectory length (2 min at 20 Hz) */

/* ---------------------------------------------------------------------------
 * Navigator states
 * --------------------------------------------------------------------------- */
typedef enum {
    NAV_IDLE     = 0,   /* no goal — task is dormant, does not touch setpoints  */
    NAV_ROTATING,       /* spinning in place to face VFH-selected direction      */
    NAV_FLYING,         /* aligned — flying forward at cruise speed              */
    NAV_ARRIVED,        /* within arrival radius — goal reached, hold commanded  */
    NAV_STUCK,          /* VFH fully blocked — holding, awaiting laptop rescue   */
    NAV_TRAJ,           /* playing an uploaded trajectory                        */
    NAV_TURN,           /* holding position, turning/correcting yaw              */
    NAV_TURN_DONE,      /* aligned and settled; fixed target yaw remains held    */
    NAV_TURN_FAILED,    /* turn failed; position hold, no trajectory start       */
    NAV_TAKEOFF_ALIGN,  /* mission owns setpoint: recover pre-arm XY             */
    NAV_FLIGHT_READY,   /* takeoff altitude and launch XY settled                */
    NAV_HOME_ALIGN,     /* mission owns setpoint: align before normal landing    */
    NAV_HOME_READY,     /* launch XY settled; normal landing requested           */
    NAV_ALIGN_FAILED,   /* hold, waiting for immediate LAND or pilot action      */
} nav_state_t;

/* ---------------------------------------------------------------------------
 * Status snapshot — returned by nav_get_status()
 * --------------------------------------------------------------------------- */
typedef struct {
    nav_state_t state;
    float       goal_x;              /* current goal map-frame north (m)         */
    float       goal_y;              /* current goal map-frame east  (m)         */
    float       goal_z;              /* current goal NED down  (m, negative AGL) */
    float       dist_to_goal;        /* horizontal distance remaining (m)        */
    float       heading_error_rad;   /* signed error toward goal, body frame     */
    float       vfh_steering_rad;    /* last VFH steering output, body frame     */
    int         free_bins;           /* free VFH bins last cycle                 */
    uint32_t    stuck_count;         /* lifetime stuck events                    */
    bool        vfh_blocked[VFH_BINS]; /* per-bin blocked state (grown + forbidden) */
} nav_status_t;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

/* Call once before spawning the task. */
void nav_task_init(void);

/* FreeRTOS task entry — pin to Core 1, Priority 3.
 *   xTaskCreatePinnedToCore(nav_task, "nav", NAV_TASK_STACK,
 *                           NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE); */
void nav_task(void *arg);

/* ---------------------------------------------------------------------------
 * Control API  (thread-safe — safe to call from any task)
 * --------------------------------------------------------------------------- */

/* Set a new goal in MAP frame (NED metres).
 *   map_x, map_y: horizontal position in map frame.
 *   z: NED down (negative = above ground, e.g. -1.5 = 1.5 m AGL).
 * Converted to odom each tick with the fixed start offset. */
void nav_set_goal_map(float map_x, float map_y, float z);

/* Cancel goal/trajectory → NAV_IDLE + position hold.  No-op if none active. */
void nav_cancel(void);

/* Absolute NED yaw in radians; direction is +1 clockwise, -1 anticlockwise.
 * Duplicate requests for the same active/completed target do not restart it. */
void nav_turn_start(float target_yaw, float direction);

/* Store points [offset, offset+n) of trajectory upload `id` (`total` points).
 * xyz: n × (x, y, z) float32, NED offsets from the first point (m).
 * A new id or total starts a new upload. Ignored while a trajectory plays. */
void nav_traj_put(uint8_t id, uint16_t total, uint16_t offset,
                  const void *xyz, int n);

/* Play upload `id` from the current position, one point every dt_ms,
 * holding the current heading. Refused unless the upload is complete and the
 * drone is armed in OFFBOARD. Each upload plays once. Ends in NAV_ARRIVED. */
void nav_traj_start(uint8_t id, uint16_t dt_ms);

/* Thread-safe status snapshot. */
nav_status_t nav_get_status(void);

/* Mission-owned status only; caller must cancel navigation first and gate
 * laptop movement commands with MISSION_BUSY. */
void nav_set_mission_status(nav_state_t state, float xy_error);
