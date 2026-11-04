#pragma once
#include <math.h>
#include <stdbool.h>

/* Quintic yaw profile: zero speed/acceleration at each end. Maximum first
 * derivative is 1.875; maximum second derivative is 10/sqrt(3). */
#define TURN_PI 3.14159265358979323846f
#define TURN_RATE_RAD_S (20.0f * TURN_PI / 180.0f)
#define TURN_ACCEL_RAD_S2 (10.0f * TURN_PI / 180.0f)
#define TURN_TOL_RAD (5.0f * TURN_PI / 180.0f)
#define TURN_SETTLE_RATE_RAD_S (5.0f * TURN_PI / 180.0f)
#define TURN_TIMEOUT_S 45.0f

static inline float turn_wrap(float a)
{
    return atan2f(sinf(a), cosf(a));
}

static inline float turn_duration(float delta)
{
    return fmaxf(1.0f, fmaxf(1.875f * fabsf(delta) / TURN_RATE_RAD_S,
                 sqrtf(5.773503f * fabsf(delta) / TURN_ACCEL_RAD_S2)));
}

static inline float turn_heading(float start, float delta, float elapsed)
{
    double u = fmin(1.0, fmax(0.0, (double)elapsed / turn_duration(delta)));
    double h = u*u*u * (10.0 + u * (-15.0 + 6.0*u));
    return turn_wrap((float)(start + delta * h));
}

/* Completion requires actual heading AND low measured rotation speed. */
static inline bool turn_aligned(float heading, float target, float rate)
{
    return isfinite(heading) && isfinite(rate)
        && fabsf(turn_wrap(target - heading)) <= TURN_TOL_RAD
        && fabsf(rate) <= TURN_SETTLE_RATE_RAD_S;
}
