#include "sentry_cv_manager_command.hpp"

#include "tap/drivers.hpp"

namespace src::control::turret::cv
{
SentryCvManagerCommand::SentryCvManagerCommand(
    tap::Drivers *drivers,
    src::serial::VisionComms &visionComms,
    src::control::turret::TurretSubsystem *sentryTurretSubsystem,
    src::control::turret::cv::TurretCVControlCommand &turretCVControlCommand,
    src::control::turret::algorithms::TurretYawControllerInterface *yawController,
    src::control::turret::algorithms::TurretPitchControllerInterface *pitchController,
    src::chassis::ChassisOdometry *chassisOdometry,
    float userYawInputScalar,
    float userPitchInputScalar,
    float MIN_PITCH_ANGLE,
    float MAX_PITCH_ANGLE,
    float PITCH_SPEED,
    float YAW_SPEED)
    : tap::control::ComprisedCommand(drivers),
      visionComms(visionComms),
      turretCVControlCommand(turretCVControlCommand),
      turretScanCommand(
          drivers,
          sentryTurretSubsystem,
          yawController,
          pitchController,
          chassisOdometry,
          MIN_PITCH_ANGLE,
          MAX_PITCH_ANGLE,
          PITCH_SPEED,
          YAW_SPEED)

{
    comprisedCommandScheduler.registerSubsystem(sentryTurretSubsystem);
    addSubsystemRequirement(sentryTurretSubsystem);
}

bool SentryCvManagerCommand::isReady()
{
    return turretScanCommand.isReady() || turretCVControlCommand.isReady();
}

void SentryCvManagerCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&turretScanCommand);
}
void SentryCvManagerCommand::execute()
{
    if (!comprisedCommandScheduler.isCommandScheduled(&turretCVControlCommand) &&
        (visionComms.isAimDataUpdated(0) || visionComms.isAimDataUpdated(1)))
    {
        comprisedCommandScheduler.addCommand(&turretCVControlCommand);
    }
    if (!comprisedCommandScheduler.isCommandScheduled(&turretScanCommand) &&
        (!visionComms.isAimDataUpdated(0) && !visionComms.isAimDataUpdated(1)))

    {
        comprisedCommandScheduler.addCommand(&turretScanCommand);
    }

    comprisedCommandScheduler.run();
}

bool SentryCvManagerCommand::isFinished() const
{
    return turretCVControlCommand.isFinished() && turretScanCommand.isFinished();
}

void SentryCvManagerCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&turretScanCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&turretCVControlCommand, interrupted);
}

}  // namespace src::control::turret::cv