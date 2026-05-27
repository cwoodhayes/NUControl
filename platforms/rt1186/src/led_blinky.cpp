/*
 * Copyright 2019, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.hpp"
#include "fsl_debug_console.h"
#include "current_sense_rt1186.hpp"

/*******************************************************************************
 * SysTick
 ******************************************************************************/
volatile uint32_t g_systickCounter;

extern "C" void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

static void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    BOARD_InitHardware();

    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    RT1186CurrentSensor sensor(5.f); // 5 A/V, matches Teensy position_sweep hardware
    sensor.init_sensor();

    while (1)
    {
        SysTick_DelayTicks(1000U);
        PRINTF("amps: %d mA\r\n", (int)(sensor.read() * 1000.f));
        RGPIO_TogglePinsOutput(EXAMPLE_LED_GPIO, 1UL << EXAMPLE_LED_GPIO_PIN);
    }
}
