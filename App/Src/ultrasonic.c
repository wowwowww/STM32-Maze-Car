/**
 * @file  ultrasonic.c
 * @brief HC-SR04 ultrasonic distance sensor driver with servo scanning.
 *
 * Each measurement:
 *   1. Assert Trig HIGH for ≥ 10 µs.
 *   2. Wait for Echo to go HIGH (start of pulse).
 *   3. Measure Echo HIGH duration in µs using TIM2 (1 MHz).
 *   4. Compute distance = echo_us / 58 (round-trip).
 *
 * One sensor is physically steered to LEFT / FRONT / RIGHT positions.
 * A settle delay is inserted after each servo move before measurement.
 */

#include "ultrasonic.h"
#include "tim.h"
#include "main.h"

/* ── Private state ───────────────────────────────────────────────────────── */
static uint16_t s_distances[US_COUNT];  /* cached distances (cm)           */

/* Servo pulse widths in microseconds (TIM4 counter @ 1 MHz, 50 Hz frame). */
#define SERVO_PULSE_LEFT_US    1100U
#define SERVO_PULSE_FRONT_US   1500U
#define SERVO_PULSE_RIGHT_US   1900U

/* ── Private helpers ─────────────────────────────────────────────────────── */

/** Read TIM2 counter (1 µs per tick). */
static inline uint32_t us_now(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/** Busy-wait for approximately @p us microseconds using TIM2. */
static void us_delay(uint32_t us)
{
    uint32_t start = us_now();
    while ((us_now() - start) < us) { /* spin */ }
}

static void servo_set_scan(UltrasonicSensor_t sensor)
{
    uint16_t pulse_us = SERVO_PULSE_FRONT_US;
    switch (sensor)
    {
    case US_LEFT:
        pulse_us = SERVO_PULSE_LEFT_US;
        break;
    case US_RIGHT:
        pulse_us = SERVO_PULSE_RIGHT_US;
        break;
    case US_FRONT:
    default:
        pulse_us = SERVO_PULSE_FRONT_US;
        break;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

void Ultrasonic_Init(void)
{
    for (uint8_t i = 0; i < US_COUNT; i++)
    {
        s_distances[i] = US_MAX_DISTANCE_CM;
    }
}

uint16_t Ultrasonic_Read(UltrasonicSensor_t sensor)
{
    if (sensor >= US_COUNT) return US_MAX_DISTANCE_CM;
    servo_set_scan(sensor);
    HAL_Delay(US_SERVO_SETTLE_MS);

    /* ── 1. 10 µs trigger pulse ─────────────────────────────────────────── */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
    us_delay(2U);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    us_delay(10U);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    /* ── 2. Wait for echo to go HIGH (with timeout) ─────────────────────── */
    uint32_t t0 = us_now();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((us_now() - t0) > US_TIMEOUT_US) return US_MAX_DISTANCE_CM;
    }

    /* ── 3. Measure echo HIGH duration ──────────────────────────────────── */
    uint32_t echo_start = us_now();
    while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((us_now() - echo_start) > US_TIMEOUT_US) return US_MAX_DISTANCE_CM;
    }
    uint32_t echo_us = us_now() - echo_start;

    /* ── 4. Convert to centimetres ───────────────────────────────────────── */
    uint16_t distance = (uint16_t)(echo_us / 58U);

    if (distance > US_MAX_DISTANCE_CM) distance = US_MAX_DISTANCE_CM;
    if (distance < 2U)                 distance = 2U;

    return distance;
}

void Ultrasonic_ReadAll(void)
{
    for (uint8_t i = 0; i < US_COUNT; i++)
    {
        s_distances[i] = Ultrasonic_Read((UltrasonicSensor_t)i);
        if (i + 1U < US_COUNT)
        {
            HAL_Delay(US_INTER_SCAN_DELAY_MS);
        }
    }
}

uint16_t Ultrasonic_GetDistance(UltrasonicSensor_t sensor)
{
    if (sensor >= US_COUNT) return US_MAX_DISTANCE_CM;
    return s_distances[sensor];
}

uint8_t Ultrasonic_WallFront(void)
{
    return (s_distances[US_FRONT] <= US_WALL_THRESHOLD_CM) ? 1U : 0U;
}

uint8_t Ultrasonic_WallLeft(void)
{
    return (s_distances[US_LEFT] <= US_WALL_THRESHOLD_CM) ? 1U : 0U;
}

uint8_t Ultrasonic_WallRight(void)
{
    return (s_distances[US_RIGHT] <= US_WALL_THRESHOLD_CM) ? 1U : 0U;
}
