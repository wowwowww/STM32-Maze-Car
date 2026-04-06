/**
 * @file  stm32f1xx_hal_msp.c
 * @brief HAL MSP (MCU Support Package) callbacks.
 *        Peripheral-level clock enable / disable is performed here so that
 *        the higher-level init functions stay portable.
 */

#include "stm32f1xx_hal.h"
#include "main.h"

/**
 * @brief Called by HAL_Init() to configure the low-level hardware:
 *        enable SYSCFG clock and configure SysTick as the timebase source.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Disable JTAG, keep SWD (frees PB3/PB4/PA15 as general GPIO)        */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/**
 * @brief TIM base MSP init – enables the peripheral clock.
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

/**
 * @brief TIM PWM MSP init – enables peripheral clocks for PWM timers.
 *        GPIO pin configuration is handled inside MX_TIM3_Init().
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    else if (htim_pwm->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}
