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
    float YAW_SPEED,
    float YAW_CHUNK_SIZE,
    float YAW_CHUNK_LINGER_TIME)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      chassisOdometry(chassisOdometry),
      MIN_PITCH_ANGLE(MIN_PITCH_ANGLE),
      MAX_PITCH_ANGLE(MAX_PITCH_ANGLE),
      PITCH_SPEED(PITCH_SPEED * tap::Drivers::DT / 1000.0f),
      YAW_SPEED(YAW_SPEED * tap::Drivers::DT / 1000.0f),
      YAW_CHUNK_SIZE(YAW_CHUNK_SIZE),
      YAW_CHUNK_LINGER_TIME(YAW_CHUNK_LINGER_TIME)
{
    assert(MIN_PITCH_ANGLE < MAX_PITCH_ANGLE);
    assert(MIN_PITCH_ANGLE >= turretSubsystem->pitchMotor.getConfig().minAngle);
    assert(MAX_PITCH_ANGLE <= turretSubsystem->pitchMotor.getConfig().maxAngle);
    assert(YAW_CHUNK_SIZE > 0);
    assert(YAW_CHUNK_LINGER_TIME > 0);
    addSubsystemRequirement(turretSubsystem);
}

bool SentryScanCommand::isReady() { return !isFinished(); }

void SentryScanCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    pitchController->setSetpoint(Angle(turretSubsystem->pitchMotor.getConfig().startAngle));
    prevTime = tap::arch::clock::getTimeMilliseconds();

    currentYawSetpoint = yawController->getSetpoint().getUnwrappedValue();
}

void SentryScanCommand::execute()
{
    currentYawChunkTimer += 0.002;
    if (currentYawChunkTimer >= YAW_CHUNK_LINGER_TIME)
    {
        currentYawChunkTimer = 0;
        currentYawSetpoint += YAW_CHUNK_SIZE;
    }

    yawController->runController(tap::Drivers::DT, Angle(currentYawSetpoint));

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
