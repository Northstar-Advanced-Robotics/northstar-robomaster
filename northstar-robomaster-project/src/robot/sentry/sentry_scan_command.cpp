#include "sentry_scan_command.hpp"

namespace src::control::turret::cv
{
SentryScanCommand::SentryScanCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    src::chassis::ChassisOdometry *chassisOdometry,
    float MIN_PITCH_ANGLE,
    float MAX_PITCH_ANGLE,
    float PITCH_SPEED,
    float YAW_SPEED)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      chassisOdometry(chassisOdometry),
      MIN_PITCH_ANGLE(MIN_PITCH_ANGLE),
      MAX_PITCH_ANGLE(MAX_PITCH_ANGLE),
      PITCH_SPEED(PITCH_SPEED * tap::Drivers::DT / 1000.0f),
      YAW_SPEED(YAW_SPEED * tap::Drivers::DT / 1000.0f)
{
    assert(MIN_PITCH_ANGLE < MAX_PITCH_ANGLE);
    assert(MIN_PITCH_ANGLE >= turretSubsystem->pitchMotor.getConfig().minAngle);
    assert(MAX_PITCH_ANGLE <= turretSubsystem->pitchMotor.getConfig().maxAngle);
    addSubsystemRequirement(turretSubsystem);
}

bool SentryScanCommand::isReady() { return !isFinished(); }

void SentryScanCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    pitchController->setSetpoint(Angle(turretSubsystem->pitchMotor.getConfig().startAngle));
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void SentryScanCommand::execute()
{
    float yawSetPoint = yawController->getSetpoint().getUnwrappedValue();
    yawController->runController(tap::Drivers::DT, Angle(yawSetPoint + YAW_SPEED));

    WrappedFloat pitchSetPoint = pitchController->getSetpoint();
    if (pitchSetPoint.minDifference(MAX_PITCH_ANGLE) < 0.05f && PITCH_SPEED > 0)
    {
        PITCH_SPEED = -PITCH_SPEED;
    }
    else if (pitchSetPoint.minDifference(MIN_PITCH_ANGLE) > -0.05f && PITCH_SPEED < 0)
    {
        PITCH_SPEED = -PITCH_SPEED;
    }
    pitchController->runController(
        tap::Drivers::DT,
        Angle(pitchSetPoint.getUnwrappedValue() + PITCH_SPEED));
}

bool SentryScanCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
}

void SentryScanCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace src::control::turret::cv