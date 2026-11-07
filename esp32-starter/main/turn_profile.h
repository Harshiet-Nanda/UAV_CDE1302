#pragma once
/* Pure turn controller, also exercised by the host tests. Units: radians/seconds. */
#include <math.h>
#include <stdbool.h>
#define TURN_PI 3.14159265358979323846f
#define TURN_MAX_SPEED (15.0f * TURN_PI / 180.0f)
#define TURN_ACCEL (10.0f * TURN_PI / 180.0f)
#define TURN_TOL (5.0f * TURN_PI / 180.0f)
#define TURN_TIMEOUT 30.0f

typedef struct {
    float x, y, z, start_yaw, target, speed, direction, distance;
    float ramp, cruise, duration, settled_since;
} turn_profile_t;

static inline float turn_wrap(float yaw) { return remainderf(yaw, 2.0f * TURN_PI); }

static inline bool turn_profile_init(turn_profile_t *p, float x, float y, float z,
                                     float yaw, float target, float signed_speed)
{
    if (!isfinite(target) || !isfinite(signed_speed) || signed_speed == 0.0f
        || !isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(yaw)) return false;
    *p = (turn_profile_t){.x=x, .y=y, .z=z, .start_yaw=yaw,
        .target=turn_wrap(target), .speed=fminf(fabsf(signed_speed), TURN_MAX_SPEED),
        .direction=signed_speed > 0 ? 1.0f : -1.0f, .settled_since=-1.0f};
    p->distance = fmodf(p->direction * (p->target - yaw), 2.0f * TURN_PI);
    if (p->distance < 0) p->distance += 2.0f * TURN_PI;
    /* A target already within tolerance needs settling, not a full revolution. */
    if (fabsf(turn_wrap(p->target-yaw)) <= TURN_TOL) p->distance = 0;
    p->ramp = fminf(p->speed / TURN_ACCEL, sqrtf(p->distance / TURN_ACCEL));
    p->cruise = p->ramp > 0 ? fmaxf(0, p->distance/(TURN_ACCEL*p->ramp)-p->ramp) : 0;
    p->duration = 2*p->ramp + p->cruise;
    return true;
}

/* 0=turning, 1=settled, -1=failed. Absolute elapsed time avoids timing drift. */
static inline int turn_profile_tick(turn_profile_t *p, float elapsed, float measured_yaw,
                                    bool ready, float *yaw)
{
    if (!ready || !isfinite(measured_yaw) || elapsed >= TURN_TIMEOUT) return -1;
    float t=fmaxf(0, elapsed), a=TURN_ACCEL, r=p->ramp;
    float travel;
    if (t < r) travel=0.5f*a*t*t;
    else if (t < r+p->cruise) travel=0.5f*a*r*r+a*r*(t-r);
    else if (t < p->duration) {
        float left=p->duration-t;
        travel=p->distance-0.5f*a*left*left;
    } else travel=p->distance;
    *yaw=t >= p->duration ? p->target : turn_wrap(p->start_yaw+p->direction*travel);
    if (t >= p->duration && fabsf(turn_wrap(measured_yaw-p->target)) <= TURN_TOL) {
        if (p->settled_since < 0) p->settled_since=t;
        if (t-p->settled_since >= 1.0f) return 1;
    } else p->settled_since=-1;
    return 0;
}
