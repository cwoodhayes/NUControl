#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "mock_hardware.hpp"

// --- Tests ---

TEST_CASE("DISABLE mode: update_control is a no-op")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::DISABLE);
  ctrl.start_control(100);

  int calls_before = drv.set_count;
  ctrl.update_control();
  REQUIRE(drv.set_count == calls_before);
}

TEST_CASE("TORQUE mode, zero target: output voltages centered near zero")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::TORQUE);
  ctrl.set_feedforward_state(false);
  ctrl.set_back_emf_comp_state(false);
  ctrl.set_feedback_state(false);
  ctrl.set_target(0.f);
  ctrl.start_control(100);

  ctrl.update_sensors();
  ctrl.update_control();

  // With all contributions disabled and zero target the three phases should be equal
  // (centering shifts them to a common offset of +1 V, but A=B=C)
  auto v = drv.last_voltages;
  REQUIRE_THAT(v.a, Catch::Matchers::WithinAbs(v.b, 1e-4f));
  REQUIRE_THAT(v.b, Catch::Matchers::WithinAbs(v.c, 1e-4f));
}

TEST_CASE("TORQUE mode, feedforward only: nonzero target produces nonzero voltages")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::TORQUE);
  ctrl.set_feedforward_state(true);
  ctrl.set_feedback_state(false);
  ctrl.set_back_emf_comp_state(false);
  ctrl.set_target(0.05f);  // 50 mNm — well within EC45_Flat limits
  ctrl.start_control(100);

  ctrl.update_sensors();
  ctrl.update_control();

  // At least one phase must differ from the 1 V centering offset
  auto v = drv.last_voltages;
  bool any_nonzero = (std::fabs(v.a - 1.f) > 1e-3f) ||
                     (std::fabs(v.b - 1.f) > 1e-3f) ||
                     (std::fabs(v.c - 1.f) > 1e-3f);
  REQUIRE(any_nonzero);
}

TEST_CASE("OPEN_LOOP_VELOCITY: shaft angle integrates at commanded rate")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  const float target_vel = 10.f;  // rad/s
  const int   period_us  = 1000;  // 1 ms steps
  const float period_s   = period_us * 1e-6f;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
  ctrl.set_target(target_vel);
  ctrl.start_control(period_us);

  const int steps = 100;
  for (int i = 0; i < steps; ++i) {
    ctrl.update_sensors();
    ctrl.update_control();
  }

  // The open-loop integrator should have accumulated target_vel * steps * period_s radians
  float expected = target_vel * steps * period_s;
  REQUIRE_THAT(ctrl.get_open_loop_angle(), Catch::Matchers::WithinAbs(expected, 1e-4f));
  REQUIRE(drv.set_count >= steps);
}

TEST_CASE("start_control resets velocity to zero and enables driver")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::TORQUE);
  ctrl.start_control(100);

  REQUIRE(drv.enabled == true);
  REQUIRE_THAT(ctrl.get_shaft_velocity(), Catch::Matchers::WithinAbs(0.f, 1e-6f));
}

TEST_CASE("stop_control disables driver")
{
  MockDriver drv;
  MockCurrentSensor s0, s1;
  auto cs = make_mock_sensor_package(s0, s1);
  MockEncoder enc;

  BrushlessController<2> ctrl{EC45_Flat, drv, cs, enc, no_sleep};
  ctrl.init_components();
  ctrl.set_control_mode(ControllerMode::TORQUE);
  ctrl.start_control(100);
  REQUIRE(drv.enabled == true);

  ctrl.stop_control();
  REQUIRE(drv.enabled == false);
}
