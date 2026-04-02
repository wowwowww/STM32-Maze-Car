/**
 * @file  motor.h
 * @brief DC motor control interface for an L298N dual H-bridge driver.
 *
 * Hardware connections
 * ───────────────────
 *  Left  motor: PWM → PA6 (TIM3_CH1), IN1 → PB0, IN2 → PB1
 *  Right motor: PWM → PA7 (TIM3_CH2), IN3 → PB10, IN4 → PB11
 *
 * Speed range
 * ───────────
 *  Speed values are in the range [-100, 100]:
 *    > 0  →  forward
 *    < 0  →  backward
 *    = 0  →  stop (coast)
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── PWM period (must match TIM3 ARR in tim.c) ────────────────────────── */
#define MOTOR_PWM_PERIOD    3600U   /* ticks – corresponds to 20 kHz       */

/* ── Speed limits ────────────────────────────────────────────────────────── */
#define MOTOR_SPEED_MAX     100
#define MOTOR_SPEED_MIN    -100

/* ── Turn speed presets (percent of MOTOR_SPEED_MAX) ────────────────────── */
#define MOTOR_BASE_SPEED    60   /* cruise speed (%)                        */
#define MOTOR_TURN_SPEED    50   /* outer wheel speed during in-place turns */

/**
 * @brief Initialise the motor driver (call after MX_TIM3_Init).
 */
void Motor_Init(void);

/**
 * @brief Set individual wheel speeds.
 *
 * @param left_speed   Left  wheel speed in [-100, 100].
 * @param right_speed  Right wheel speed in [-100, 100].
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/**
 * @brief Drive both motors at the same forward speed.
 *
 * @param speed  Speed in [0, 100].
 */
void Motor_Forward(uint8_t speed);

/**
 * @brief Drive both motors at the same backward speed.
 *
 * @param speed  Speed in [0, 100].
 */
void Motor_Backward(uint8_t speed);

/**
 * @brief In-place right turn: left wheel forward, right wheel backward.
 *
 * @param speed  Rotation speed in [0, 100].
 */
void Motor_TurnRight(uint8_t speed);

/**
 * @brief In-place left turn: right wheel forward, left wheel backward.
 *
 * @param speed  Rotation speed in [0, 100].
 */
void Motor_TurnLeft(uint8_t speed);

/**
 * @brief Stop both motors (coast).
 */
void Motor_Stop(void);

/**
 * @brief Brake both motors (active braking via H-bridge).
 */
void Motor_Brake(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
