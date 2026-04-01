/**
 * @file  main.c
 * @brief STM32F103VET6 Maze Car – main entry point.
 *
 * Initialises the HAL, configures the system clock to 72 MHz, brings up all
 * peripherals, and then runs the maze-solving state machine.
 *
 * System clock configuration:
 *   HSE 8 MHz  →  PLL × 9  →  SYSCLK 72 MHz
 *   AHB = 72 MHz, APB1 = 36 MHz, APB2 = 72 MHz
 */

#include <stdio.h>

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "motor.h"
#include "ultrasonic.h"
#include "maze_solver.h"

/* ── Private function prototypes ─────────────────────────────────────────── */
static void MX_SystemClock_Config(void);

/* =========================================================================
 * main()
 * ========================================================================= */
int main(void)
{
    /* ── 1. HAL initialisation ──────────────────────────────────────────── */
    HAL_Init();

    /* ── 2. Configure system clock: HSE → PLL → 72 MHz ─────────────────── */
    MX_SystemClock_Config();

    /* ── 3. Peripheral initialisation ───────────────────────────────────── */
    MX_GPIO_Init();
    MX_TIM2_Init();   /* microsecond timer */
    MX_TIM3_Init();   /* motor PWM         */
    MX_USART1_UART_Init();

    printf("\r\n==============================\r\n");
    printf("  STM32 Maze Car  v1.0\r\n");
    printf("  SYSCLK = %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000UL);
    printf("==============================\r\n\r\n");

    /* ── 4. Application-layer initialisation ────────────────────────────── */
    Motor_Init();
    Ultrasonic_Init();
    MazeSolver_Init();

    /* Brief delay to let sensors settle                                     */
    HAL_Delay(500);

    printf("[INFO] Starting maze solver...\r\n");
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);

    /* ── 5. Main loop – run the maze solver ─────────────────────────────── */
    for (;;)
    {
        MazeSolver_Step();
    }
}

/* =========================================================================
 * Clock configuration: HSE → PLL → 72 MHz
 * ========================================================================= */
static void MX_SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* ── Enable HSE and PLL ─────────────────────────────────────────────── */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;   /* 8 × 9 = 72 MHz */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /* ── Configure bus clocks ───────────────────────────────────────────── */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 72 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1  = 36 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 72 MHz */

    /* Flash wait-state must be 2 for SYSCLK > 48 MHz                       */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* =========================================================================
 * Error handler
 * =========================================================================
 * Blinks the status LED rapidly and prints a message. In a production build
 * this should trigger a watchdog reset.
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    printf("[ERROR] Fatal error – halting.\r\n");

    for (;;)
    {
        HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN);
        /* Busy-wait ~200 ms (no HAL_Delay in error context)                */
        for (volatile uint32_t i = 0; i < 720000UL; i++) { __NOP(); }
    }
}
