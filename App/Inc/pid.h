/**
 * @file  pid.h
 * @brief Generic discrete PID controller.
 *
 * Used by the maze solver to maintain a target wall-following distance,
 * trimming the left/right wheel speeds to keep the car centred.
 *
 * Output formula (position form):
 *   u(k) = Kp·e(k) + Ki·∑e + Kd·(e(k) − e(k−1))
 *
 * The output is clamped to [out_min, out_max] and integral wind-up is
 * prevented by clamping the accumulated integral term independently.
 */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp;           /**< Proportional gain                             */
    float ki;           /**< Integral gain                                 */
    float kd;           /**< Derivative gain                               */

    float setpoint;     /**< Desired process value                         */

    float integral;     /**< Accumulated integral term                     */
    float prev_error;   /**< Previous error (for derivative term)          */

    float out_min;      /**< Output lower clamp                            */
    float out_max;      /**< Output upper clamp                            */
    float int_min;      /**< Integral lower clamp (anti-windup)            */
    float int_max;      /**< Integral upper clamp (anti-windup)            */
} PID_t;

/**
 * @brief Initialise a PID controller with the given gains and limits.
 *
 * @param pid      Pointer to PID structure to initialise.
 * @param kp       Proportional gain.
 * @param ki       Integral gain.
 * @param kd       Derivative gain.
 * @param out_min  Minimum controller output.
 * @param out_max  Maximum controller output.
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief Set the desired setpoint.
 *
 * @param pid       Pointer to PID structure.
 * @param setpoint  New target value.
 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

/**
 * @brief Reset integrator and derivative state.
 *
 * @param pid  Pointer to PID structure.
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Compute one PID step.
 *
 * @param pid           Pointer to PID structure.
 * @param measurement   Current process value.
 * @return Controller output, clamped to [out_min, out_max].
 */
float PID_Compute(PID_t *pid, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
