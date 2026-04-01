/**
 * @file  pid.c
 * @brief Discrete PID controller implementation.
 */

#include "pid.h"

/* ── Private clamp helper ────────────────────────────────────────────────── */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->setpoint  = 0.0f;
    pid->integral  = 0.0f;
    pid->prev_error= 0.0f;
    pid->out_min   = out_min;
    pid->out_max   = out_max;
    /* Clamp integrator to the same range as the output by default          */
    pid->int_min   = out_min;
    pid->int_max   = out_max;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_t *pid, float measurement)
{
    float error      = pid->setpoint - measurement;
    pid->integral   += error;
    pid->integral    = clampf(pid->integral, pid->int_min, pid->int_max);
    float derivative = error - pid->prev_error;
    pid->prev_error  = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    return clampf(output, pid->out_min, pid->out_max);
}
