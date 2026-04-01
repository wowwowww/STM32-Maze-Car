/**
 * @file  motor.c
 * @brief DC motor control for an L298N dual H-bridge driver.
 *
 * Speed is converted from the [-100, 100] percent range to a PWM compare
 * value (CCR) in [0, MOTOR_PWM_PERIOD].
 *
 * Direction truth table for the L298N:
 *   IN1 IN2 | Left motor   |  IN3 IN4 | Right motor
 *   ─────────────────────────────────────────────────
 *    1   0  |  Forward     |   1   0  |  Forward
 *    0   1  |  Backward    |   0   1  |  Backward
 *    0   0  |  Coast       |   0   0  |  Coast
 *    1   1  |  Brake       |   1   1  |  Brake
 */

#include "motor.h"
#include "tim.h"
#include "main.h"

/* ── Private helpers ─────────────────────────────────────────────────────── */

/** Clamp a value to [lo, hi]. */
static inline int16_t clamp16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Convert percent speed [-100,100] to a CCR value [0, MOTOR_PWM_PERIOD]. */
static inline uint32_t speed_to_ccr(int16_t speed)
{
    uint16_t abs_speed = (uint16_t)(speed < 0 ? -speed : speed);
    return ((uint32_t)abs_speed * MOTOR_PWM_PERIOD) / 100U;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

void Motor_Init(void)
{
    Motor_Stop();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    left_speed  = clamp16(left_speed,  MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    right_speed = clamp16(right_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    /* ── Left motor ─────────────────────────────────────────────────────── */
    if (left_speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
    else if (left_speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }

    /* ── Right motor ─────────────────────────────────────────────────────── */
    if (right_speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_RESET);
    }
    else if (right_speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_RESET);
    }

    /* ── PWM duty cycle ──────────────────────────────────────────────────── */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed_to_ccr(left_speed));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed_to_ccr(right_speed));
}

void Motor_Forward(uint8_t speed)
{
    int16_t s = (int16_t)clamp16((int16_t)speed, 0, MOTOR_SPEED_MAX);
    Motor_SetSpeed(s, s);
}

void Motor_Backward(uint8_t speed)
{
    int16_t s = (int16_t)clamp16((int16_t)speed, 0, MOTOR_SPEED_MAX);
    Motor_SetSpeed(-s, -s);
}

void Motor_TurnRight(uint8_t speed)
{
    int16_t s = (int16_t)clamp16((int16_t)speed, 0, MOTOR_SPEED_MAX);
    Motor_SetSpeed(s, -s);
}

void Motor_TurnLeft(uint8_t speed)
{
    int16_t s = (int16_t)clamp16((int16_t)speed, 0, MOTOR_SPEED_MAX);
    Motor_SetSpeed(-s, s);
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_RESET);
}

void Motor_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_SET);
}
