/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_LED_GPIO     BOARD_USER_LED_GPIO
#define EXAMPLE_LED_GPIO_PIN BOARD_USER_LED_GPIO_PIN

#define DEMO_LPADC_BASE          ADC1
#define DEMO_LPADC_CHANNEL_NUM   1U
#define DEMO_LPADC_VREF_SOURCE   kLPADC_ReferenceVoltageAlt2
#define DEMO_DMA_BASE            DMA4
#define DEMO_DMA_CHANNEL         0U
#define ADC_DMA_REQUEST_SOURCE   kDma4RequestMuxADC1Request0
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
