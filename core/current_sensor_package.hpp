#ifndef NUCONTROL_CORE_CURRENT_SENSOR_PACKAGE_HPP
#define NUCONTROL_CORE_CURRENT_SENSOR_PACKAGE_HPP

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include "current_sensor_interface.hpp"
#include "transformations.hpp"
#include "discrete_filter.hpp"

template <size_t N>
class CurrentSensorPackage
{
  static_assert(N == 2 || N == 3, "only 2 or 3 current sensors supported");

public:
  using LogFn = std::function<void(const std::string &)>;

  CurrentSensorPackage() = delete;

  CurrentSensorPackage(
      std::array<ICurrentSensor *, N> sensors,
      LogFn log_fn = [](const std::string &) {})
      : sensors_(sensors), log_(log_fn)
  {
  }

  bool init_sensors()
  {
    bool all_inited = true;
    for (size_t i = 0; i < N; ++i)
      all_inited &= sensors_.at(i)->init_sensor();
    return all_inited;
  }

  void print_calibration()
  {
    log_("Phase Indices: ");
    log_(std::to_string(phase_idx_.a));
    log_(std::to_string(phase_idx_.b));
    log_(std::to_string(phase_idx_.c));
    log_("Phase Directions: ");
    log_(std::to_string(phase_dirs_.a));
    log_(std::to_string(phase_dirs_.b));
    log_(std::to_string(phase_dirs_.c));
  }

  bool load_calibration(PhaseValues<int> phase_idx, PhaseValues<int> phase_dirs)
  {
    // Validate indices are in range and no two phases share a sensor.
    const int indices[3] = {phase_idx.a, phase_idx.b, phase_idx.c};
    for (int idx : indices)
    {
      if (idx < -1 || idx >= static_cast<int>(N))
      {
        log_("load_calibration: sensor index out of range");
        return false;
      }
    }
    // Check for duplicate assignments (two phases claiming the same sensor).
    for (int i = 0; i < 3; ++i)
    {
      if (indices[i] == -1) continue;
      for (int j = i + 1; j < 3; ++j)
      {
        if (indices[i] == indices[j])
        {
          log_("load_calibration: duplicate sensor index");
          return false;
        }
      }
    }

    phase_idx_  = phase_idx;
    phase_dirs_ = phase_dirs;
    print_calibration();
    aligned_ = true;
    return true;
  }

  PhaseValues<float> get_phase_currents(bool filter = true)
  {
    if (!aligned_)
    {
      log_("Sensor Package Not Aligned To Driver");
      return {};
    }

    auto amps = filter ? read_filtered_sensors() : read_sensors();

    PhaseValues<float> phase_amps;
    if (phase_idx_.a > -1) phase_amps.a = phase_dirs_.a * amps.at(phase_idx_.a);
    if (phase_idx_.b > -1) phase_amps.b = phase_dirs_.b * amps.at(phase_idx_.b);
    if (phase_idx_.c > -1) phase_amps.c = phase_dirs_.c * amps.at(phase_idx_.c);

    if constexpr (N == 3)
      return phase_amps;

    // 2-sensor case: reconstruct missing phase via Kirchhoff
    if (phase_idx_.a == -1) phase_amps.a = -phase_amps.b - phase_amps.c;
    if (phase_idx_.b == -1) phase_amps.b = -phase_amps.a - phase_amps.c;
    if (phase_idx_.c == -1) phase_amps.c = -phase_amps.a - phase_amps.b;

    return phase_amps;
  }

  void set_filters(DiscreteFilter<float, float> filter)
  {
    for (size_t i = 0; i < N; ++i)
      sensors_.at(i)->set_filter(filter);
  }

  std::array<float, N> read_sensors() const
  {
    std::array<float, N> reads{};
    for (size_t i = 0; i < N; ++i)
      reads.at(i) = sensors_.at(i)->read();
    return reads;
  }

  std::array<float, N> read_filtered_sensors()
  {
    std::array<float, N> reads{};
    for (size_t i = 0; i < N; ++i)
      reads.at(i) = sensors_.at(i)->read_filtered();
    return reads;
  }

private:
  std::array<ICurrentSensor *, N> sensors_;
  LogFn log_;

  PhaseValues<int> phase_idx_{-1, -1, -1};
  PhaseValues<int> phase_dirs_{0, 0, 0};
  bool aligned_ = false;
};

#endif // NUCONTROL_CORE_CURRENT_SENSOR_PACKAGE_HPP
