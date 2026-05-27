/*
 * Copyright 2019, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.hpp"
#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "fsl_edma.h"

/*******************************************************************************
 * SysTick delay
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
 * ADC / DMA
 ******************************************************************************/
#define ADC_BUF_SIZE 8U

AT_NONCACHEABLE_SECTION_INIT(volatile uint16_t g_adcBuf[ADC_BUF_SIZE]);
static edma_handle_t g_edmaHandle;

static void ADC_Init(void)
{
    lpadc_config_t adcConfig;
    lpadc_conv_command_config_t commandConfig;
    lpadc_conv_trigger_config_t triggerConfig;

    LPADC_GetDefaultConfig(&adcConfig);
    adcConfig.enableAnalogPreliminary = true;
    adcConfig.FIFO0Watermark          = 0x7U;
    adcConfig.referenceVoltageSource  = DEMO_LPADC_VREF_SOURCE;
    LPADC_Init(DEMO_LPADC_BASE, &adcConfig);

    LPADC_SetOffsetCalibrationMode(DEMO_LPADC_BASE, kLPADC_OffsetCalibration12bitMode);
    LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);
    LPADC_DoAutoCalibration(DEMO_LPADC_BASE);

    LPADC_GetDefaultConvCommandConfig(&commandConfig);
    commandConfig.channelNumber            = DEMO_LPADC_CHANNEL_NUM;
    commandConfig.sampleTimeMode           = kLPADC_SampleTimeADCK5;
    commandConfig.chainedNextCommandNumber = 1U; /* self-loop for continuous sampling */
    commandConfig.sampleScaleMode          = kLPADC_SampleFullScale;
    LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, 1U, &commandConfig);

    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId = 1U;
    LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &triggerConfig);

    LPADC_DoResetFIFO0(DEMO_LPADC_BASE);
    LPADC_EnableFIFO0WatermarkDMA(DEMO_LPADC_BASE, true);
}

static void DMA_Init(void)
{
    edma_config_t edmaConfig;
    edma_transfer_config_t transferConfig;

    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(DEMO_DMA_BASE, &edmaConfig);
    EDMA_CreateHandle(&g_edmaHandle, DEMO_DMA_BASE, DEMO_DMA_CHANNEL);
    EDMA_SetChannelMux(DEMO_DMA_BASE, DEMO_DMA_CHANNEL, ADC_DMA_REQUEST_SOURCE);

    EDMA_PrepareTransfer(&transferConfig,
                         (void *)&(DEMO_LPADC_BASE->RESFIFO[0U]),
                         2U,
                         (void *)g_adcBuf,
                         sizeof(g_adcBuf[0]),
                         2U * 8U,
                         ADC_BUF_SIZE * 2U,
                         kEDMA_PeripheralToMemory);

    /* Wrap destination back to buffer start after each major loop for circular operation */
    transferConfig.destAddr = (uint32_t)g_adcBuf;

    EDMA_SubmitTransfer(&g_edmaHandle, &transferConfig);

    /* Disable major loop completion interrupt — we poll the buffer, no handler needed */
    EDMA_DisableChannelInterrupts(DEMO_DMA_BASE, DEMO_DMA_CHANNEL, kEDMA_MajorInterruptEnable);

    /* Circular destination: wrap back to buffer start after each major loop */
    EDMA_SetModulo(DEMO_DMA_BASE, DEMO_DMA_CHANNEL, kEDMA_ModuloDisable, kEDMA_Modulo16bytes);

    EDMA_StartTransfer(&g_edmaHandle);
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

    ADC_Init();
    DMA_Init();
    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1UL);

    while (1)
    {
        SysTick_DelayTicks(1000U);
        uint16_t sample = g_adcBuf[0] / 8U;
        PRINTF("ADC sample: %u\r\n", sample);
        RGPIO_TogglePinsOutput(EXAMPLE_LED_GPIO, 1UL << EXAMPLE_LED_GPIO_PIN);
    }
}
