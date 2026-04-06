/**
 * @file  gpio.c
 * @brief GPIO peripheral initialisation.
 *
 * Configures all GPIO pins used by the Maze Car:
 *  - Motor direction outputs (PB0, PB1, PB10, PB11)
 *  - Ultrasonic trigger output + echo input (PC0, PC1)
 *  - Status LED (PE0)
 *  - Alternate-function pins for TIM3 and USART1 are configured in their
 *    respective init functions (tim.c, usart.c).
 */

#include "gpio.h"
#include "main.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ── Enable port clocks ─────────────────────────────────────────────── */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* ── Default output levels ──────────────────────────────────────────── */
    /* Motor driver: all direction pins LOW → motors stopped (coasting)     */
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_RESET);

    /* Ultrasonic: trigger lines LOW (idle)                                  */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    /* Status LED: off                                                        */
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);

    /* ── Motor direction outputs (PB0, PB1, PB10, PB11) ────────────────── */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN
                        | MOTOR_R_IN3_PIN | MOTOR_R_IN4_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── Ultrasonic trigger output (PC0) ─────────────────────────────────── */
    GPIO_InitStruct.Pin = US_TRIG_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ── Ultrasonic echo input (PC1) ─────────────────────────────────────── */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin  = US_ECHO_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ── Status LED output (PE0) ─────────────────────────────────────────── */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin   = LED_STATUS_PIN;
    HAL_GPIO_Init(LED_STATUS_PORT, &GPIO_InitStruct);
}
