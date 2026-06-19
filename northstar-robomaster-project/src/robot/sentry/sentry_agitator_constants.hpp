#ifndef SENTRY_AGITATOR_CONSTANTS_HPP_
#define SENTRY_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem_config.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

namespace src::control::agitator::constants
{
static constexpr uint16_t HEAT_LIMIT_BUFFER = 25;
// position PID terms
// PID terms for SENTRY
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 11;        // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 30.0f;       // balls per second
static constexpr float OVERSHOOT_FUDGE_FACTOR = 0.10;  // how much agitator overshoots

static constexpr src::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 1 / (36.0f) * (20.0f / 21.0f),
    .agitatorMotorId = tap::motor::MOTOR5,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 700.0f / M_TWOPI,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    .targetIntegralChange = M_TWOPI / AGITATOR_NUM_POCKETS - OVERSHOOT_FUDGE_FACTOR,
    .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.1f,
};

constexpr float UNJAM_VELOCITY = 0.6f * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS);
constexpr float UNJAM_DISTANCE = 1.0f * (M_TWOPI / AGITATOR_NUM_POCKETS);
static constexpr src::control::agitator::UnjamSpokeAgitatorCommand::Config AGITATOR_UNJAM_CONFIG = {
    .targetUnjamIntegralChange = UNJAM_DISTANCE,
    .unjamSetpoint = UNJAM_VELOCITY,
    /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    /// seconds.Convert to ms, Add 100 ms extra tolerance.
    .maxWaitTime = static_cast<uint32_t>(1000.0f * UNJAM_DISTANCE / UNJAM_VELOCITY) + 200,
    .targetCycleCount = 3,
};
}  // namespace src::control::agitator::constants

#endif  // TESTBED_AGITATOR_CONSTANTS_HPP_