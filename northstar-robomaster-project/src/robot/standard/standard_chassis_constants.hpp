/**
 * Chassis constants for the standard.
 *
 * Drive motor PID gains, chassis geometry, acceleration limits, and the top speed the operator
 * can request. Power limiting is done by the power loop in `HolonomicChassisSubsystem::refresh`.
 *
 * Include `chassis_constants.hpp` rather than this file; it picks the right robot's constants for
 * the build target, and this header refuses to compile on its own.
 */
#ifndef STANDARD_CHASSIS_CONSTANTS_HPP_
#define STANDARD_CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"


#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

namespace src::control::chassis
{
static constexpr float VELOCITY_PID_KP = 10.0f;                 // 10.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;                  // 0.0f;
static constexpr float VELOCITY_PID_KD = 1.0f;                  // 1.25f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 16'000.0f;  // 0.0f;
static constexpr float VELOCITY_PID_KV = 0.0f;                  // 0.057f;
static constexpr float VELOCITY_PID_KS = 0.0f;                  // 350.0f;
static constexpr float VELOCITY_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_C620;
static constexpr float CHASSIS_ROTATION_P = 4.0f;
static constexpr float CHASSIS_ROTATION_D = 0.01f;
static constexpr float CHASSIS_ROTATION_MAX_VEL = M_TWOPI;
static constexpr float AUTO_ROTATION_ALPHA = 0.01f;

static constexpr float CHASSIS_GEAR_RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;

static const float DIST_TO_CENTER = .2201774561f;  // from wheel to center
static const float WHEEL_DIAMETER_M = 0.191;

/// Chassis speed plain WASD asks for, in m/s. A request only: the power loop decides actual speed.
static constexpr float CHASSIS_WALK_SPEED_MPS = 1.65f;
/// Chassis speed full stick or Shift+WASD asks for, in m/s. A request ceiling only: the power
/// loop decides how fast the robot actually goes.
static constexpr float MAX_CHASSIS_SPEED_MPS = 3.3f;

// m/s/s
static constexpr float CHASSIS_ACCEL_VALUE = 3.5f;
static constexpr float CHASSIS_DECCEL_VALUE = 7.5f;

static constexpr float ACCEL_TAPER_FACTOR = 0.7f;

// rad/s/s
static constexpr float ROTATION_ACCEL_VALUE = 20.0f;
static constexpr float ROTATION_ACCEL_TAPER_FACTOR = 0.8f;

}  // namespace src::control::chassis

#endif