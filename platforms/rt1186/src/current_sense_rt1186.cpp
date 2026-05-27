#include "current_sense_rt1186.hpp"

/*******************************************************************************
 * Static member definitions
 ******************************************************************************/
/*******************************************************************************
 * ICurrentSensor interface
 ******************************************************************************/
bool RT1186CurrentSensor::init_sensor()
{
    lpadc_config_t adcConfig;
    lpadc_conv_command_config_t commandConfig;
    lpadc_conv_trigger_config_t triggerConfig;

    LPADC_GetDefaultConfig(&adcConfig);
    adcConfig.enableAnalogPreliminary = true;
    adcConfig.referenceVoltageSource  = DEMO_LPADC_VREF_SOURCE;
    LPADC_Init(DEMO_LPADC_BASE, &adcConfig);

    LPADC_SetOffsetCalibrationMode(DEMO_LPADC_BASE, kLPADC_OffsetCalibration12bitMode);
    LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);
    LPADC_DoAutoCalibration(DEMO_LPADC_BASE);

    LPADC_GetDefaultConvCommandConfig(&commandConfig);
    commandConfig.channelNumber = DEMO_LPADC_CHANNEL_NUM;
    LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, 1U, &commandConfig);

    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId       = 1U;
    triggerConfig.enableHardwareTrigger = false;
    LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &triggerConfig);

    return true;
}

float RT1186CurrentSensor::read()
{
    lpadc_conv_result_t result;

    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U);
    while (!LPADC_GetConvResult(DEMO_LPADC_BASE, &result, 0U)) {}

    /* convValue is left-aligned; shift right by 3 to get 12-bit count */
    return (float)(result.convValue >> 3U) * amps_per_count_;
}

float RT1186CurrentSensor::read_filtered()
{
    // if (filter_)
    //     return filter_->update(read());
    return read();
}

void RT1186CurrentSensor::set_filter(DiscreteFilter<float, float> filter)
{
    // filter_.reset(new DiscreteFilter<float, float>(filter));
}
