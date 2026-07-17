#ifndef CURRENT_SENSE_RT1186_HPP
#define CURRENT_SENSE_RT1186_HPP

#include "transformations.hpp"    // must precede discrete_filter.hpp (defines PhaseValues)
#include "current_sensor_interface.hpp"
#include "app.hpp"
#include "fsl_lpadc.h"

class RT1186CurrentSensor : public ICurrentSensor
{
public:
    /// @param shunt_resistance_ohms  sense resistor value
    /// @param op_amp_gain            conditioning amplifier gain
    /// ADC full-scale: 1.8 V (kLPADC_ReferenceVoltageAlt2), 12-bit (4096 counts)
    RT1186CurrentSensor(float shunt_resistance_ohms, float op_amp_gain)
        : amps_per_count_(1.8f / (4096.f * shunt_resistance_ohms * op_amp_gain))
    {}

    /// @param amps_per_volt  combined transimpedance gain of the sensing circuit
    explicit RT1186CurrentSensor(float amps_per_volt)
        : amps_per_count_(amps_per_volt * (1.8f / 4096.f))
    {}

    ~RT1186CurrentSensor() override = default;

    bool init_sensor() override;
    float read() override;
    float read_filtered() override;
    void set_filter(DiscreteFilter<float, float> filter) override;

private:
    const float amps_per_count_;
};

#endif // CURRENT_SENSE_RT1186_HPP
