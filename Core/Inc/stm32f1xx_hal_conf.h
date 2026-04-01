/**
 * @file  stm32f1xx_hal_conf.h
 * @brief HAL module enable/disable configuration.
 *        Only modules used by this project are enabled to reduce code size.
 */

#ifndef __STM32F1XX_HAL_CONF_H
#define __STM32F1XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── HAL module selection ──────────────────────────────────────────────── */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED          /* required by most peripherals    */
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ── Oscillator values (adjust to match your hardware) ─────────────────── */
#if !defined(HSE_VALUE)
  #define HSE_VALUE    8000000U   /* 8 MHz external crystal                */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U  /* ms */
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE    8000000U   /* 8 MHz internal RC (after trim)        */
#endif

#if !defined(LSI_VALUE)
  #define LSI_VALUE    40000U     /* 40 kHz internal low-speed RC          */
#endif

#if !defined(LSE_VALUE)
  #define LSE_VALUE    32768U     /* 32.768 kHz external RTC crystal       */
#endif

/* ── VDD voltage (used for Flash wait-state calculation) ───────────────── */
#define  VDD_VALUE     3300U      /* mV */

/* ── SysTick timebase ──────────────────────────────────────────────────── */
#define  TICK_INT_PRIORITY    0x0FU  /* lowest priority for SysTick        */

/* ── Ethernet / CAN / etc: not used in this project ───────────────────── */
#define  USE_RTOS    0
#define  USE_SD_TRANSCEIVER  0

/* ── Include peripheral HAL headers ────────────────────────────────────── */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f1xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f1xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f1xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f1xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f1xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f1xx_hal_pwr.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f1xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f1xx_hal_uart.h"
#endif

/* ── Assertion macro (remove in release builds for speed) ─────────────── */
#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_HAL_CONF_H */
