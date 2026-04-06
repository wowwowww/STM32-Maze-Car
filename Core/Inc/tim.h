/**
 * @file  tim.h
 * @brief Timer initialisation for motor PWM and microsecond delay.
 *
 * TIM3 – 20 kHz PWM on CH1 (PA6, left motor) and CH2 (PA7, right motor).
 * TIM2 – Free-running 1 MHz counter used by the ultrasonic driver for
 *         microsecond-accurate pulse measurement.
 * TIM4 – 50 Hz PWM on CH1 (PB6) for ultrasonic sensor steering servo.
 */

#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;  /* microsecond timer                     */
extern TIM_HandleTypeDef htim3;  /* motor PWM timer                       */
extern TIM_HandleTypeDef htim4;  /* servo PWM timer                       */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */
