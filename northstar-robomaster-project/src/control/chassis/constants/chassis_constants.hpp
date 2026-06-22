#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

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
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

static constexpr float AMPS_DESIRED_OUTPUT_RATIO = 20.0f / 16384.0f;  // I/Output
static constexpr float CHASSIS_VOLTAGE = 24.0f;
static constexpr float MAX_M3508_RPM_CHASSIS =
    482.0 / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;

static constexpr float BEYBLADE_SPEEDUP_FACTOR = 1.1f;

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