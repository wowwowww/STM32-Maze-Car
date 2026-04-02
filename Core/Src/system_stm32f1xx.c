/**
 * @file  system_stm32f1xx.c
 * @brief CMSIS System Source File for STM32F1xx.
 *
 * Configures the system clock to 72 MHz using an 8 MHz HSE crystal and the
 * internal PLL (PLLMUL × 9).  After reset the HSI (8 MHz) is used until
 * HAL_RCC_ClockConfig() switches to HSE/PLL inside MX_SystemClock_Config().
 *
 * Clock tree (all in MHz):
 *   HSE 8 → PLL (×9) → SYSCLK 72 → AHB 72 → APB2 72 → APB1 36
 */

#include "stm32f1xx.h"

/* SystemCoreClock is updated by HAL_RCC_GetSysClockFreq() after init.     */
uint32_t SystemCoreClock = 8000000U;  /* default: HSI before PLL locks     */

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0,
                                     1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
 * @brief  Setup the microcontroller system: reset RCC clocks to default.
 *         The actual 72 MHz configuration is applied later in
 *         MX_SystemClock_Config() via the HAL API.
 */
void SystemInit(void)
{
    /* FPU settings (Cortex-M3 has no FPU, so nothing to do here)          */

    /* Reset the RCC clock configuration to the default reset state        */
    /* Set HSION bit                                                        */
    RCC->CR |= 0x00000001U;

    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits                  */
    RCC->CFGR &= 0xF8FF0000U;

    /* Reset HSEON, CSSON and PLLON bits                                   */
    RCC->CR &= 0xFEF6FFFFU;

    /* Reset HSEBYP bit                                                    */
    RCC->CR &= 0xFFFBFFFFU;

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits            */
    RCC->CFGR &= 0xFF80FFFFU;

    /* Disable all interrupts and clear pending bits                       */
    RCC->CIR = 0x009F0000U;

    /* Vector table relocation (default: Flash at 0x08000000)              */
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE  | VECT_TAB_OFFSET;
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif
}

/**
 * @brief  Update SystemCoreClock from current RCC register values.
 *         Called automatically by HAL after clock reconfiguration.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp, pllmull, pllsource;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case 0x00U:  /* HSI used as system clock                          */
            SystemCoreClock = HSI_VALUE;
            break;

        case 0x04U:  /* HSE used as system clock                         */
            SystemCoreClock = HSE_VALUE;
            break;

        case 0x08U:  /* PLL used as system clock                         */
            pllmull   = (RCC->CFGR & RCC_CFGR_PLLMULL) >> 18U;
            pllsource = (RCC->CFGR & RCC_CFGR_PLLSRC)  >> 16U;

            pllmull = (pllmull == 0x0FU) ? 16U : (pllmull + 2U);

            if (pllsource == 0U)
            {
                /* HSI oscillator / 2 selected as PLL clock source */
                SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
            }
            else
            {
#if defined(STM32F105xC) || defined(STM32F107xC)
                /* PLL2 … not applicable here */
                SystemCoreClock = HSE_VALUE * pllmull;
#else
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != RESET)
                    SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
                else
                    SystemCoreClock = HSE_VALUE * pllmull;
#endif
            }
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Compute HCLK frequency */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    SystemCoreClock >>= tmp;
}
