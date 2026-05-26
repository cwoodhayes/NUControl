#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "mock_hardware.hpp"
#include "current_sensor_package.hpp"

using Catch::Matchers::WithinAbs;
static constexpr float tol = 1e-5f;

// MockCurrentSensor that counts how many times set_filter was called
struct TrackingCurrentSensor : public ICurrentSensor
{
  float value = 0.f;
  int filter_set_count = 0;

  bool init_sensor() override { return true; }
  float read() override { return value; }
  float read_filtered() override { return value * 2.f; } // distinguishable from read()
  void set_filter(DiscreteFilter<float, float>) override { ++filter_set_count; }
};

// MockCurrentSensor whose init_sensor always fails
struct FailingCurrentSensor : public ICurrentSensor
{
  bool init_sensor() override { return false; }
  float read() override { return 0.f; }
  float read_filtered() override { return 0.f; }
  void set_filter(DiscreteFilter<float, float>) override {}
};

// --- init_sensors ------------------------------------------------------------

TEST_CASE("init_sensors: returns true when all sensors init successfully", "[package]")
{
  MockCurrentSensor s0, s1;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  REQUIRE(pkg.init_sensors());
}

TEST_CASE("init_sensors: returns false if any sensor fails", "[package]")
{
  MockCurrentSensor s0;
  FailingCurrentSensor s1;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  REQUIRE_FALSE(pkg.init_sensors());
}

// --- load_calibration / get_phase_currents -----------------------------------

TEST_CASE("get_phase_currents: returns zero before alignment", "[package]")
{
  MockCurrentSensor s0, s1;
  s0.value = 1.f;
  s1.value = 2.f;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};

  auto result = pkg.get_phase_currents(false);
  REQUIRE_THAT(result.a, WithinAbs(0.f, tol));
  REQUIRE_THAT(result.b, WithinAbs(0.f, tol));
  REQUIRE_THAT(result.c, WithinAbs(0.f, tol));
}

TEST_CASE("load_calibration: maps sensors to phases with correct directions", "[package]")
{
  MockCurrentSensor s0, s1;
  s0.value = 3.f;
  s1.value = -5.f;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};

  // sensor 0 -> phase A (positive), sensor 1 -> phase B (negative direction), phase C reconstructed
  pkg.load_calibration({0, 1, -1}, {1, -1, 0});

  auto v = pkg.get_phase_currents(false);
  REQUIRE_THAT(v.a, WithinAbs(3.f,  tol));   // dir=1  * s0=3
  REQUIRE_THAT(v.b, WithinAbs(5.f,  tol));   // dir=-1 * s1=-5 = 5
  REQUIRE_THAT(v.c, WithinAbs(-8.f, tol));   // Kirchhoff: -(3 + 5) = -8
}

TEST_CASE("get_phase_currents: 3-sensor, all phases directly measured", "[package]")
{
  MockCurrentSensor s0, s1, s2;
  s0.value = 1.f;
  s1.value = 2.f;
  s2.value = -3.f;
  CurrentSensorPackage<3> pkg{{{&s0, &s1, &s2}}, no_sleep};

  pkg.load_calibration({0, 1, 2}, {1, 1, 1});

  auto v = pkg.get_phase_currents(false);
  REQUIRE_THAT(v.a, WithinAbs(1.f,  tol));
  REQUIRE_THAT(v.b, WithinAbs(2.f,  tol));
  REQUIRE_THAT(v.c, WithinAbs(-3.f, tol));
}

// --- Kirchhoff reconstruction ------------------------------------------------

TEST_CASE("get_phase_currents: 2-sensor Kirchhoff reconstructs missing phase A", "[package]")
{
  MockCurrentSensor s0, s1;
  s0.value = 1.f;
  s1.value = 2.f;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  pkg.load_calibration({-1, 0, 1}, {0, 1, 1}); // phase A missing

  auto v = pkg.get_phase_currents(false);
  REQUIRE_THAT(v.b, WithinAbs(1.f,  tol));
  REQUIRE_THAT(v.c, WithinAbs(2.f,  tol));
  REQUIRE_THAT(v.a, WithinAbs(-3.f, tol));  // -(1 + 2)
}

TEST_CASE("get_phase_currents: 2-sensor Kirchhoff reconstructs missing phase C", "[package]")
{
  MockCurrentSensor s0, s1;
  s0.value = 4.f;
  s1.value = -1.f;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  pkg.load_calibration({0, 1, -1}, {1, 1, 0}); // phase C missing

  auto v = pkg.get_phase_currents(false);
  REQUIRE_THAT(v.a, WithinAbs(4.f,  tol));
  REQUIRE_THAT(v.b, WithinAbs(-1.f, tol));
  REQUIRE_THAT(v.c, WithinAbs(-3.f, tol));  // -(4 + -1)
}

// --- Filtered vs unfiltered --------------------------------------------------

TEST_CASE("get_phase_currents: filter=false uses read(), filter=true uses read_filtered()", "[package]")
{
  TrackingCurrentSensor s0, s1;
  s0.value = 2.f;
  s1.value = 3.f;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};
  pkg.load_calibration({0, 1, -1}, {1, 1, 0});

  auto raw = pkg.get_phase_currents(false);
  REQUIRE_THAT(raw.a, WithinAbs(2.f, tol));   // read() = value
  REQUIRE_THAT(raw.b, WithinAbs(3.f, tol));

  auto filtered = pkg.get_phase_currents(true);
  REQUIRE_THAT(filtered.a, WithinAbs(4.f, tol));  // read_filtered() = value * 2
  REQUIRE_THAT(filtered.b, WithinAbs(6.f, tol));
}

// --- set_filters -------------------------------------------------------------

TEST_CASE("set_filters: propagates to all sensors", "[package]")
{
  TrackingCurrentSensor s0, s1;
  CurrentSensorPackage<2> pkg{{{&s0, &s1}}, no_sleep};

  pkg.set_filters(DiscreteFilter<float, float>{});

  REQUIRE(s0.filter_set_count == 1);
  REQUIRE(s1.filter_set_count == 1);
}

