#include "three_flywheel_run_command.hpp"

namespace src::control::flywheel
{
ThreeFlywheelRunCommand::ThreeFlywheelRunCommand(
    ThreeFlywheelSubsystem *flywheel,
    float launchSpeed,
    float spin)
    : flywheel(flywheel),
      launchSpeed(launchSpeed),
      spin(spin)

{
    addSubsystemRequirement(flywheel);
}

void ThreeFlywheelRunCommand::initialize()
{
    flywheel->setDesiredSpin(spin);
    flywheel->setDesiredLaunchSpeed(launchSpeed);
}

void ThreeFlywheelRunCommand::end(bool) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel
