#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

/**
 * Selects the chassis constants for the robot currently being built, and defines the constants
 * shared by every chassis.
 *
 * Robot-specific values (wheel diameter, PID gains, dimensions) live in each target's
 * `*_chassis_constants.hpp`; `standard` is used as the fallback when no known target is defined.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_chassis_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_chassis_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_chassis_constants.hpp"
#elif TURRET
#include "robot/standard/standard_chassis_constants.hpp"
#elif TARGET_TEST_BED
#include "robot/standard/standard_chassis_constants.hpp"
#else
#include "robot/standard/standard_chassis_constants.hpp"
#endif

namespace src::chassis
{
// hardware constants, not specific to any particular chassis
/// CAN ID of the left front drive motor.
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
/// CAN ID of the left back drive motor.
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
/// CAN ID of the right front drive motor.
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
/// CAN ID of the right back drive motor.
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

/// Converts a motor output command into the current it draws, in amps. Used by the power
/// limiter to predict draw before committing to an output.
static constexpr float AMPS_DESIRED_OUTPUT_RATIO = 20.0f / 16384.0f;  // I/Output
/// Nominal chassis bus voltage, in volts.
static constexpr float CHASSIS_VOLTAGE = 24.0f;
/// Top speed of an M3508 at the **motor shaft**, in RPM: its 482 RPM free-running output speed
/// scaled up through the 3591:187 gearbox. Roughly 9256. These are the units `mpsToRpm` and the
/// wheel velocity PID work in.
static constexpr float MAX_M3508_RPM_CHASSIS =
    482.0 / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;

/**
 * Rescales a pair of joystick axes so that a full diagonal deflection commands the same speed
 * as a full deflection along one axis.
 *
 * The sticks are square but speed should be radial, so pushing both axes fully would otherwise
 * ask for `sqrt(2)` times the intended speed. Scaling by the ratio of the larger axis to the
 * vector's magnitude maps the square input range onto a circle while preserving direction.
 *
 * @param[in] vert The vertical (forward) axis, from -1 to 1.
 * @param[in] hor The horizontal (sideways) axis, from -1 to 1.
 * @return The rescaled vertical and horizontal axes, in that order. Zero input maps to zero.
 */
[[maybe_unused]] static modm::Pair<float, float> getNormalizedInput(float vert, float hor)
{
    if (vert == 0.0f && hor == 0.0f)
    {
        return modm::Pair<float, float>(0.0f, 0.0f);
    }

    float magnitude = sqrtf((vert * vert) + (hor * hor));

    float maxDeflection = modm::max(fabsf(vert), fabsf(hor));

    return modm::Pair<float, float>(
        vert * (maxDeflection / magnitude),
        hor * (maxDeflection / magnitude));
}
}  // namespace src::chassis

#endif  // CHASSIS_CONSTANTS_HPP_