#ifndef CURRENT_SENSE_HPP
#define CURRENT_SENSE_HPP
#include <Arduino.h>
#include "driver.hpp"
#include "helpers.hpp"
#include "errors.hpp"
#include "discrete_filter.hpp"
#include "current_sensor_interface.hpp"

class InlineCurrentSensor : public ICurrentSensor
{
public:
  InlineCurrentSensor() = default;
  ~InlineCurrentSensor() override = default;

  /// @param pin - Which pin to analogRead for the current data
  /// @param amps_per_volt - how many amps / volt from the ADC
  /// @note Sets warning when current exceeds 1.5 * the gain
  InlineCurrentSensor(
    int pin, float amps_per_volt, int ADC_res = 10, void(*f)(ErrorCodes) = *handle_errors)
  : pin_(pin),
    gain_(amps_per_volt),
    SATURATE_READING_(1.5f * gain_),
    MAX_READING_(2.f * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res)),
    error_callback(f)
  {
    analogReadRes(ADC_res);
  }

  InlineCurrentSensor(int pin, float shunt_resistance_ohms, float op_amp_gain, int ADC_res = 10)
  : pin_(pin),
    gain_(1.f / (shunt_resistance_ohms * op_amp_gain)),
    SATURATE_READING_(1.5f * gain_),
    MAX_READING_(2.f * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res))
  {
    analogReadRes(ADC_res);
  }

  bool init_sensor() override
  {
    pinMode(pin_, INPUT);
    return validate_offset();
  }

  float read() override
  {
    auto amps = gain_ * (analogRead(pin_) * ADC_GAIN_ - offset_);
    if (fabs(amps) > SATURATE_READING_) {
      Serial.println("Current Sensor Saturated!");
      if (fabs(amps) > MAX_READING_) {
        error_callback(ErrorCodes::CURRENT_SENSE_OVER_LIMIT);
      }
    }
    return amps;
  }

  void set_filter(DiscreteFilter<float, float> filter) override
  {
    filter_ = filter;
  }

  float read_filtered() override
  {
    return filter_.update(read());
  }

private:
  const int pin_;
  const float gain_;
  float offset_ = 1.65f;
  const float SATURATE_READING_;
  const float MAX_READING_;
  const float ADC_GAIN_;

  DiscreteFilter<float, float> filter_;

  void (*error_callback)(ErrorCodes);

  bool validate_offset(size_t n = 10000) const
  {
    int sum_ = 0;
    for (size_t i = 0; i < n; ++i)
      sum_ += analogRead(pin_);
    float offset = static_cast<float>(sum_) / n * ADC_GAIN_;

    if (fabs(offset_ - offset) > 1e-2) {
      Serial.print("Sensor (pin ");
      Serial.print(pin_);
      Serial.print(") Failed to Init. Offset voltage (V): ");
      Serial.println(offset);
      return false;
    }
    return true;
  }
};

#endif
