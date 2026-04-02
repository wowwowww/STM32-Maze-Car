/**
 * @file  ultrasonic.c
 * @brief HC-SR04 ultrasonic distance sensor driver.
 *
 * Each measurement:
 *   1. Assert Trig HIGH for ≥ 10 µs.
 *   2. Wait for Echo to go HIGH (start of pulse).
 *   3. Measure Echo HIGH duration in µs using TIM2 (1 MHz).
 *   4. Compute distance = echo_us / 58 (round-trip).
 *
 * Sensors are read sequentially to avoid crosstalk.
 * A 60 ms inter-measurement delay is inserted between sensors.
 */

#include "ultrasonic.h"
#include "tim.h"
#include "main.h"

/* ── Private state ───────────────────────────────────────────────────────── */
static uint16_t s_distances[US_COUNT];  /* cached distances (cm)           */

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

/** Sensor GPIO lookup table. */
typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} US_Sensor_IO_t;

static const US_Sensor_IO_t s_sensor_io[US_COUNT] = {
    /* FRONT */ { US_FRONT_TRIG_PORT, US_FRONT_TRIG_PIN,
                  US_FRONT_ECHO_PORT, US_FRONT_ECHO_PIN },
    /* LEFT  */ { US_LEFT_TRIG_PORT,  US_LEFT_TRIG_PIN,
                  US_LEFT_ECHO_PORT,  US_LEFT_ECHO_PIN  },
    /* RIGHT */ { US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN,
                  US_RIGHT_ECHO_PORT, US_RIGHT_ECHO_PIN },
};

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

    const US_Sensor_IO_t *io = &s_sensor_io[sensor];

    /* ── 1. 10 µs trigger pulse ─────────────────────────────────────────── */
    HAL_GPIO_WritePin(io->trig_port, io->trig_pin, GPIO_PIN_RESET);
    us_delay(2U);
    HAL_GPIO_WritePin(io->trig_port, io->trig_pin, GPIO_PIN_SET);
    us_delay(10U);
    HAL_GPIO_WritePin(io->trig_port, io->trig_pin, GPIO_PIN_RESET);

    /* ── 2. Wait for echo to go HIGH (with timeout) ─────────────────────── */
    uint32_t t0 = us_now();
    while (HAL_GPIO_ReadPin(io->echo_port, io->echo_pin) == GPIO_PIN_RESET)
    {
        if ((us_now() - t0) > US_TIMEOUT_US) return US_MAX_DISTANCE_CM;
    }

    /* ── 3. Measure echo HIGH duration ──────────────────────────────────── */
    uint32_t echo_start = us_now();
    while (HAL_GPIO_ReadPin(io->echo_port, io->echo_pin) == GPIO_PIN_SET)
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
        /* 60 ms gap between sensors prevents crosstalk                      */
        HAL_Delay(60U);
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
