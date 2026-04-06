/**
 * @file  ultrasonic.h
 * @brief HC-SR04 ultrasonic distance sensor driver.
 *
 * One HC-SR04 sensor is mounted on a steering servo:
 *   Sensor Trig/Echo  – PC0 / PC1
 *   Servo PWM         – TIM4_CH1 (PB6)
 *
 * The driver rotates the servo to LEFT/FRONT/RIGHT scan positions and
 * performs one measurement at each position.
 *
 * Timing is performed with TIM2 running at 1 MHz (1 µs resolution).
 *
 * Distance formula:
 *   distance_cm = echo_pulse_us / 58
 *   (sound travels ~34 300 cm/s; round-trip ÷ 2 = echo_us × 0.017 cm/µs)
 *
 * Maximum reliable range: ~400 cm.
 * Minimum reliable range: ~2 cm.
 * Timeout: 38 ms (> 400 cm round-trip).
 */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Sensor identifiers ──────────────────────────────────────────────────── */
typedef enum {
    US_FRONT = 0,
    US_LEFT  = 1,
    US_RIGHT = 2,
    US_COUNT = 3
} UltrasonicSensor_t;

/* ── Distance thresholds (cm) ────────────────────────────────────────────── */
#define US_WALL_THRESHOLD_CM     20U   /* closer than this → wall detected  */
#define US_CLEAR_THRESHOLD_CM    25U   /* farther than this → path is clear */
#define US_MAX_DISTANCE_CM      400U   /* sensor maximum range              */
#define US_TIMEOUT_US         38000U   /* 38 ms echo timeout                */
#define US_SERVO_SETTLE_MS       70U   /* ~40-50 ms motion + margin to reduce jitter */

/**
 * @brief Initialise the ultrasonic driver (call after MX_TIM2_Init).
 */
void Ultrasonic_Init(void);

/**
 * @brief Measure distance from a single sensor.
 *
 * Sends a 10 µs trigger pulse and measures the echo width.
 * Returns US_MAX_DISTANCE_CM when no echo is detected within the timeout.
 *
 * @param sensor  Which sensor to read.
 * @return Distance in centimetres [2, 400].
 */
uint16_t Ultrasonic_Read(UltrasonicSensor_t sensor);

/**
 * @brief Read all three sensors in sequence and cache the results.
 *        Call this once per control-loop iteration.
 */
void Ultrasonic_ReadAll(void);

/**
 * @brief Return the most recently cached distance for a sensor.
 *
 * @param sensor  Which sensor to query.
 * @return Distance in centimetres.
 */
uint16_t Ultrasonic_GetDistance(UltrasonicSensor_t sensor);

/**
 * @brief Return true if a wall is detected in front of the car.
 */
uint8_t Ultrasonic_WallFront(void);

/**
 * @brief Return true if a wall is detected on the left of the car.
 */
uint8_t Ultrasonic_WallLeft(void);

/**
 * @brief Return true if a wall is detected on the right of the car.
 */
uint8_t Ultrasonic_WallRight(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */
