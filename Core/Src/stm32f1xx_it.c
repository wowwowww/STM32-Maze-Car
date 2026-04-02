/**
 * @file  stm32f1xx_it.c
 * @brief Cortex-M3 and STM32F1xx exception / interrupt handlers.
 */

#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"

/* ── Cortex-M3 exceptions ────────────────────────────────────────────────── */

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)   {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}

/**
 * @brief SysTick interrupt handler.
 *        HAL_IncTick() increments the 1 ms HAL time base used by
 *        HAL_Delay() and all timeout mechanisms.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
