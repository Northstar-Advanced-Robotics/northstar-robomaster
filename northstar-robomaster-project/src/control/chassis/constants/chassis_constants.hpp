#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"

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

namespace src::control::chassis
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

/**
 * Power model used by `getChassisPowerDraw` until the chassis has real power sensors. Per motor:
 *
 *     P = Kt * I * w  +  k1 * I^2  +  k2 * w^2
 *
 * summed over the motors, plus a static draw. `I` is the C620's measured current (A) and `w` the
 * wheel shaft speed (rad/s).
 *
 * @warning `k1`, `k2`, and the static draw are starting guesses, not measured. Calibrate them by
 *      driving over the power limit and matching the model to the referee system's power buffer,
 *      which falls at `(P - limit)` watts, or against a bench power meter.
 */
/// M3508 torque constant at the output shaft (after the gearbox), in N*m/A. Matches taproot's
/// `M3508Constants::getTorqueConstant()`.
static constexpr float M3508_TORQUE_CONSTANT_NM_PER_A = 0.3f;
/// Copper loss coefficient, in W/A^2. Starting value 1.5 * 0.194 ohm (M3508 phase resistance).
/// Calibrate.
static constexpr float POWER_MODEL_COPPER_LOSS_W_PER_A2 = 1.5f * 0.194f;
/// Speed-dependent loss (friction, iron loss) coefficient, in W/(rad/s)^2. Calibrate.
static constexpr float POWER_MODEL_SPEED_LOSS_W_PER_RAD2 = 0.0f;
/// Draw of the whole drivetrain at rest (motor controllers idling), in watts. Calibrate.
static constexpr float POWER_MODEL_STATIC_W = 0.0f;

/// Power target used when the referee system is offline, in watts, so an unplugged referee
/// system slows the robot rather than letting it draw freely. The ARCC 2026 default (HP Priority)
/// Infantry limit, and the lowest of the robot limits.
static constexpr float POWER_FALLBACK_LIMIT_W = 75.0f;

/**
 * Referee power buffer protection. Per the ARCC 2026 rules (Events Manual 3.3.1.4), chassis power
 * over the limit drains a buffer capped at 60 J, and if the buffer is empty while still over the
 * limit **the chassis is powered off for 5 seconds**. The power loop therefore never plans to
 * spend the buffer: it runs at the limit while the buffer is within `POWER_BUFFER_DEADBAND_J` of
 * full, and below that cuts the target in proportion so the buffer refills long before it empties.
 */
/// The referee's buffer energy limit, in joules.
static constexpr float POWER_BUFFER_FULL_J = 60.0f;
/// How far the buffer may fall below full before the power target is cut, in joules. Absorbs
/// small, brief overshoots without slowing the robot.
static constexpr float POWER_BUFFER_DEADBAND_J = 10.0f;
/// How much the power target drops per joule the buffer is below the deadband, in watts per joule.
/// This feedback is what corrects for error in the power model.
static constexpr float POWER_BUFFER_GAIN_W_PER_J = 3.0f;

/// How far the rotation ramp may run ahead of the measured rotation speed while the power loop is
/// limiting, in radians/second. The rotational counterpart of `RAMP_WINDUP_MARGIN_MPS`: stops
/// beyblade's large request from winding the ramp up so the spin stops soon after release.
static constexpr float ROTATION_WINDUP_MARGIN_RADPS = 2.0f;

/// How far a translation ramp may run ahead of the measured chassis velocity while the power loop
/// is limiting, in m/s. Keeps the setpoint from winding up beyond what the robot can reach (so it
/// stops promptly on release) while still letting it start moving when another demand, such as
/// beyblade spin, is what is using the power.
static constexpr float RAMP_WINDUP_MARGIN_MPS = 0.5f;

/// How fast the beyblade's share of the rotation budget shrinks while the power loop is limiting,
/// as a fraction of the budget per second. Backing spin off is what gives translation priority.
static constexpr float BEYBLADE_BUDGET_DOWN_RATE = 1.0f;
/// How fast the beyblade's share of the rotation budget recovers while power is not limiting, as
/// a fraction of the budget per second.
static constexpr float BEYBLADE_BUDGET_UP_RATE = 0.3f;
/// Smallest share of the rotation budget beyblade is ever backed off to.
static constexpr float BEYBLADE_BUDGET_MIN_FRACTION = 0.2f;
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
}  // namespace src::control::chassis

#endif  // CHASSIS_CONSTANTS_HPP_