#include "kicker_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

/* Subsystem for the kicker on the hero turret. */
namespace src::control::kicker
{
KickerSubsystem::KickerSubsystem(
    tap::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    const KickerSubsystemConfig& kickerSubsystemConfig)
    : tap::control::Subsystem(drivers),
      config(kickerSubsystemConfig),
      velocityPid(pidParams),
      kickerMotor(
          drivers,
          config.kickerMotorId,
          config.kickerCanBusId,
          config.isKickerInverted,
          "kicker motor",
          false,
          config.gearRatio)
{
}

void KickerSubsystem::initialize() { kickerMotor.initialize(); }

void KickerSubsystem::refresh() { runVelocityPidControl(); }

void KickerSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = velocitySetpoint - getCurrentVelocity();

    velocityPid.runControllerDerivateError(velocityError, dt);

    kickerMotor.setDesiredOutput(
        velocityPid.getOutput() + velocitySetpoint * config.velocityPIDFeedForwardGain);
}

void KickerSubsystem::setSetpoint(float velocity)
{
    if (kickerMotor.isMotorOnline())
    {
        velocitySetpoint = velocity;
    }
}
}  // namespace src::control::kicker