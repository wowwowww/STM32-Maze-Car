/**
 * @file  main.h
 * @brief Pin and peripheral definitions for the STM32F103VET6 Maze Car.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  Pin Map                                                                │
 * ├──────────────────┬──────────────┬─────────────────────────────────────┤
 * │  Function        │  Pin         │  Peripheral                         │
 * ├──────────────────┼──────────────┼─────────────────────────────────────┤
 * │  Left  motor PWM │  PA6         │  TIM3_CH1 (AF, 20 kHz)             │
 * │  Right motor PWM │  PA7         │  TIM3_CH2 (AF, 20 kHz)             │
 * │  Left  motor IN1 │  PB0         │  GPIO_Output                        │
 * │  Left  motor IN2 │  PB1         │  GPIO_Output                        │
 * │  Right motor IN3 │  PB10        │  GPIO_Output                        │
 * │  Right motor IN4 │  PB11        │  GPIO_Output                        │
 * ├──────────────────┼──────────────┼─────────────────────────────────────┤
 * │  US Trig         │  PC0         │  GPIO_Output                        │
 * │  US Echo         │  PC1         │  GPIO_Input                         │
 * │  Servo PWM       │  PB6         │  TIM4_CH1 (AF, 50 Hz)              │
 * ├──────────────────┼──────────────┼─────────────────────────────────────┤
 * │  Debug TX        │  PA9         │  USART1_TX                          │
 * │  Debug RX        │  PA10        │  USART1_RX                          │
 * ├──────────────────┼──────────────┼─────────────────────────────────────┤
 * │  Status LED      │  PE0         │  GPIO_Output (active-high)          │
 * └──────────────────┴──────────────┴─────────────────────────────────────┘
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ── Motor direction GPIO ──────────────────────────────────────────────── */
#define MOTOR_L_IN1_PIN     GPIO_PIN_0
#define MOTOR_L_IN1_PORT    GPIOB

#define MOTOR_L_IN2_PIN     GPIO_PIN_1
#define MOTOR_L_IN2_PORT    GPIOB

#define MOTOR_R_IN3_PIN     GPIO_PIN_10
#define MOTOR_R_IN3_PORT    GPIOB

#define MOTOR_R_IN4_PIN     GPIO_PIN_11
#define MOTOR_R_IN4_PORT    GPIOB

/* ── Ultrasonic sensor GPIO ────────────────────────────────────────────── */
#define US_TRIG_PIN         GPIO_PIN_0
#define US_TRIG_PORT        GPIOC

#define US_ECHO_PIN         GPIO_PIN_1
#define US_ECHO_PORT        GPIOC

#define US_SERVO_PWM_PIN    GPIO_PIN_6
#define US_SERVO_PWM_PORT   GPIOB

/* ── Status LED ────────────────────────────────────────────────────────── */
#define LED_STATUS_PIN      GPIO_PIN_0
#define LED_STATUS_PORT     GPIOE

/* ── Error handler ─────────────────────────────────────────────────────── */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
