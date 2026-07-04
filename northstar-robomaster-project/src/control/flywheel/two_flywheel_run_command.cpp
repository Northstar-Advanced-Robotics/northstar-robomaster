#include "two_flywheel_run_command.hpp"

namespace src::control::flywheel
{
TwoFlywheelRunCommand::TwoFlywheelRunCommand(
    TwoFlywheelSubsystem* flywheel,
    float launchSpeed = 20.0f,
    tap::communication::serial::RefSerial* refSerial = nullptr)
    : flywheel(flywheel),
      launchSpeed(launchSpeed),
      refSerial(refSerial)

{
    addSubsystemRequirement(flywheel);
}

void TwoFlywheelRunCommand::initialize() { flywheel->setDesiredLaunchSpeed(launchSpeed); }

void TwoFlywheelRunCommand::execute()
{
    if (refSerial != nullptr && lastShotSpeed != refSerial->getRobotData().turret.bulletSpeed)
    {
        if (refSerial->getRobotData().turret.bulletSpeed > upperLimit)
        {
            launchSpeed -= decrement;
        }
        else if (refSerial->getRobotData().turret.bulletSpeed < lowerLimit)
        {
            launchSpeed += increment;
        }
    }
    lastShotSpeed = refSerial->getRobotData().turret.bulletSpeed;
    flywheel->setDesiredLaunchSpeed(launchSpeed);
}

void TwoFlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel