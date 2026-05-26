#ifndef NUCONTROL_CORE_CURRENT_SENSOR_INTERFACE_HPP
#define NUCONTROL_CORE_CURRENT_SENSOR_INTERFACE_HPP

#include "discrete_filter.hpp"

class ICurrentSensor
{
public:
  virtual ~ICurrentSensor() = default;
  virtual bool init_sensor() = 0;
  virtual float read() = 0;
  virtual float read_filtered() = 0;
  virtual void set_filter(DiscreteFilter<float, float> filter) = 0;
};

#endif // NUCONTROL_CORE_CURRENT_SENSOR_INTERFACE_HPP
