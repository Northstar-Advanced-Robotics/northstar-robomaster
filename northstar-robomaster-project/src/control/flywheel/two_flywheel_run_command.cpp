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

void TwoFlywheelRunCommand::initialize()
{
    if (refSerial != nullptr)
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
    flywheel->setDesiredLaunchSpeed(launchSpeed);
}

void TwoFlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel