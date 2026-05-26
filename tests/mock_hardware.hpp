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

struct MockSensorPackage : public ICurrentSensorPackage
{
  PhaseValues<float> currents{0.f, 0.f, 0.f};

  bool init_sensors() override { return true; }
  PhaseValues<float> get_phase_currents(bool /*filter*/) override { return currents; }
  void set_filters(DiscreteFilter<float, float>) override {}
  void print_calibration() override {}
  bool align_sensors(IBrushlessDriver &, float) override { return true; }
  bool load_calibration(PhaseValues<int>, PhaseValues<int>) override { return true; }
};

static auto no_sleep = [](int){};

#endif // NUCONTROL_TESTS_MOCK_HARDWARE_HPP
