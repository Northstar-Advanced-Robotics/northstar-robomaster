#ifndef HERO_KICKER_CONSTANTS_HPP_
#define HERO_KICKER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem_config.hpp"
#include "control/kicker/kicker_subsystem_config.hpp"

// Do not include this file directly: use kicker_constants.hpp instead.
#ifndef KICKER_CONSTANTS_HPP_
#error "Do not include this file directly! Use kicker_constants.hpp instead."
#endif

namespace src::control::kicker::constants
{
// position PID terms
// PID terms for hero
static constexpr tap::algorithms::SmoothPidConfig KICKER_PID_CONFIG = {
    .kp = 5000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr src::kicker::KickerSubsystemConfig KICKER_CONFIG = {
    .gearRatio = 1 / 36.0f,
    .kickerMotorId = tap::motor::MOTOR3,
    .kickerCanBusId = tap::can::CanBus::CAN_BUS2,
    .isKickerInverted = false,
    .velocityPIDFeedForwardGain = 0.0f,
};
}  // namespace src::control::kicker::constants

#endif  // HERO_KICKER_CONSTANTS_HPP_