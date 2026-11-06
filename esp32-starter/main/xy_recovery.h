#pragma once
#include <math.h>
#include <stdbool.h>

/* Initial recovery limits; verify on the actual vehicle. Not a route planner. */
#define XY_RECOVERY_TOL_M        0.10f
#define TAKEOFF_ALIGNMENT_TOL_M  0.20f
#define XY_RECOVERY_SPEED_M_S    0.10f
#define XY_RECOVERY_ACCEL_M_S2   0.05f
#define XY_RECOVERY_SETTLE_M_S   0.10f
#define XY_RECOVERY_MAX_M        1.0f
#define XY_RECOVERY_TIMEOUT_S    30.0f
#define XY_RECOVERY_SETTLE_US    1000000

static inline float xy_recovery_duration(float distance)
{
    return fmaxf(1.0f, fmaxf(1.875f*distance/XY_RECOVERY_SPEED_M_S,
                 sqrtf(5.773503f*distance/XY_RECOVERY_ACCEL_M_S2)));
}

static inline float xy_recovery_fraction(float elapsed, float duration)
{
    double u=fmin(1.0, fmax(0.0, (double)elapsed/duration));
    return (float)(u*u*u*(10.0+u*(-15.0+6.0*u)));
}

static inline bool xy_recovery_settled_with_radius(float x, float y, float z,
                                      float vx, float vy, float vz,
                                      float home_x, float home_y, float hold_z, float radius_m)
{
    return isfinite(x) && isfinite(y) && isfinite(z)
        && isfinite(vx) && isfinite(vy) && isfinite(vz)
        && hypotf(x-home_x,y-home_y) <= radius_m
        && fabsf(z-hold_z) <= 0.15f
        && sqrtf(vx*vx+vy*vy+vz*vz) <= XY_RECOVERY_SETTLE_M_S;
}

/* Home landing keeps the original, tighter alignment radius. */
static inline bool xy_recovery_settled(float x, float y, float z,
                                      float vx, float vy, float vz,
                                      float home_x, float home_y, float hold_z)
{
    return xy_recovery_settled_with_radius(x,y,z,vx,vy,vz,home_x,home_y,hold_z,
                                           XY_RECOVERY_TOL_M);
}
