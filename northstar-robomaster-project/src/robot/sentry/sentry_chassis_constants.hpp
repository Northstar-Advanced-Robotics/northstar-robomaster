#ifndef SENTRY_CHASSIS_CONSTANTS_HPP_
#define SENTRY_CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_encoder.hpp"

#include "modm/math/interpolation/linear.hpp"

#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

using tap::can::CanBus;
using tap::motor::DjiMotor;

namespace src::chassis
{
static constexpr float VELOCITY_PID_KP = 10.0f;                 // 10.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;                  // 0.0f;
static constexpr float VELOCITY_PID_KD = 1.0f;                  // 1.25f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 16'000.0f;  // 0.0f;
static constexpr float VELOCITY_PID_KV = 0.0f;                  // 0.057f;
static constexpr float VELOCITY_PID_KS = 0.0f;                  // 350.0f;
static constexpr float VELOCITY_PID_MAX_OUTPUT = DjiMotor::MAX_OUTPUT_C620;
static constexpr float CHASSIS_ROTATION_P = 4.0f;
static constexpr float CHASSIS_ROTATION_D = 0.01f;
static constexpr float CHASSIS_ROTATION_MAX_VEL = M_TWOPI;
static constexpr float AUTO_ROTATION_ALPHA = 0.01f;

static constexpr float CHASSIS_GEAR_RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;

static const float DIST_TO_CENTER = .2201774561f;  // 2286   // from wheel to center
static const float WHEEL_DIAMETER_M = 0.1816861f;

static constexpr float MAX_CHASSIS_SPEED_MPS = 8.0f;

static constexpr float MAX_CHASSIS_WHEEL_SPEED = 9000.0f;

static constexpr float CHASSIS_WALK_MULTIPLIER = 0.5f;

static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {
    /*
    Hero - 100W
    Power Prio Standard - 90W
    HP priority Standard - 75W
    Sentry - 100W
    1v1 Standard - 120W
    */
    {50, 3'000},
    {80, 4'400},
    {100, 5'200},
    {125, 6'000}};
// At 9000 rpm the beyblade was around 130, Its over

static constexpr float CHASSIS_ACCEL_VALUE = 0.007f;   // 0.015f;
static constexpr float CHASSIS_DECCEL_VALUE = 0.015f;  // 0.015f;

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

}  // namespace src::chassis

#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_