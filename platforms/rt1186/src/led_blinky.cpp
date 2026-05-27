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

/*******************************************************************************
 * ADC / DMA
 ******************************************************************************/
#define ADC_BUF_SIZE 8U

AT_NONCACHEABLE_SECTION_INIT(volatile uint32_t g_adcBuf[ADC_BUF_SIZE]);
static edma_handle_t g_edmaHandle;
static volatile bool g_dmaTransferDone = false;

extern "C" void DMA4_CH0_CH1_CH32_CH33_IRQHandler(void)
{
    EDMA_HandleIRQ(&g_edmaHandle);
}

static void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
        g_dmaTransferDone = true;
    }
}

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
    commandConfig.chainedNextCommandNumber = 1U;
    commandConfig.sampleScaleMode          = kLPADC_SampleFullScale;
    LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, 1U, &commandConfig);

    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId = 1U;
    LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &triggerConfig);

    LPADC_DoResetFIFO0(DEMO_LPADC_BASE);
    LPADC_EnableFIFO0WatermarkDMA(DEMO_LPADC_BASE, true);
}

static void DMA_Arm(void)
{
    edma_transfer_config_t transferConfig;

    EDMA_PrepareTransfer(&transferConfig,
                         (void *)&(DEMO_LPADC_BASE->RESFIFO[0U]),
                         sizeof(uint32_t),
                         (void *)g_adcBuf,
                         sizeof(uint32_t),
                         sizeof(uint32_t) * 8U,
                         ADC_BUF_SIZE * sizeof(uint32_t),
                         kEDMA_PeripheralToMemory);

    EDMA_SubmitTransfer(&g_edmaHandle, &transferConfig);
    EDMA_StartTransfer(&g_edmaHandle);
}

static void DMA_Init(void)
{
    edma_config_t edmaConfig;

    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(DEMO_DMA_BASE, &edmaConfig);
    EDMA_CreateHandle(&g_edmaHandle, DEMO_DMA_BASE, DEMO_DMA_CHANNEL);
    EDMA_SetChannelMux(DEMO_DMA_BASE, DEMO_DMA_CHANNEL, ADC_DMA_REQUEST_SOURCE);
    EDMA_SetCallback(&g_edmaHandle, EDMA_Callback, NULL);
    DMA_Arm();
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
        /* Wait for DMA to fill the buffer */
        while (!g_dmaTransferDone)
        {
        }
        g_dmaTransferDone = false;

        /* Stop ADC, print latest sample, restart */
        LPADC_Deinit(DEMO_LPADC_BASE);
        uint16_t sample = (uint16_t)(g_adcBuf[ADC_BUF_SIZE - 1U] / 8U);
        PRINTF("ADC sample: %u\r\n", sample);
        RGPIO_TogglePinsOutput(EXAMPLE_LED_GPIO, 1UL << EXAMPLE_LED_GPIO_PIN);

        /* Reinit ADC and rearm DMA for next burst */
        ADC_Init();
        DMA_Arm();
        LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1UL);
    }
}
