#ifndef NUCONTROL_TESTS_MOCK_HARDWARE_HPP
#define NUCONTROL_TESTS_MOCK_HARDWARE_HPP

#include "brushless_controller.hpp"

struct MockEncoder : public IAbsoluteEncoder
{
  float angle = 0.f;
  float read() override { return angle; }
};

struct MockDriver : public IBrushlessDriver
{
  PhaseValues<float> last_voltages{0.f, 0.f, 0.f};
  bool enabled = false;
  int  set_count = 0;

  bool init() override { return true; }
  void enable()  override { enabled = true; }
  void disable() override { enabled = false; last_voltages = {0.f, 0.f, 0.f}; }

  PhaseValues<int> set_phase_voltages(PhaseValues<float> v) override
  {
    last_voltages = v;
    ++set_count;
    return {0, 0, 0};
  }
};

struct MockCurrentSensor : public ICurrentSensor
{
  float value = 0.f;
  bool init_sensor() override { return true; }
  float read() override { return value; }
  float read_filtered() override { return value; }
  void set_filter(DiscreteFilter<float, float>) override {}
};

static auto no_sleep = [](int){};

// Pre-aligned 2-sensor package for use in tests.
// Wraps two MockCurrentSensor instances with phase_idx {0,1,-1} and dirs {1,1,0}.
inline CurrentSensorPackage<2> make_mock_sensor_package(
    MockCurrentSensor &s0, MockCurrentSensor &s1)
{
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  pkg.load_calibration({0, 1, -1}, {1, 1, 0});
  return pkg;
}

#endif // NUCONTROL_TESTS_MOCK_HARDWARE_HPP
