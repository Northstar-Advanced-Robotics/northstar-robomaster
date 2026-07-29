#include "two_flywheel_run_rpm_command.hpp"

namespace src::control::flywheel
{
TwoFlywheelRunRPMCommand::TwoFlywheelRunRPMCommand(
    TwoFlywheelSubsystem *flywheel,
    float rpm = 5000.0f)
    : flywheel(flywheel),
      rpm(rpm)

{
    addSubsystemRequirement(flywheel);
}

void TwoFlywheelRunRPMCommand::initialize() { flywheel->setDesiredFlywheelSpeed(rpm); }

void TwoFlywheelRunRPMCommand::execute()
{
    if (rpm != flywheel->getDesiredFlywheelSpeed()) flywheel->setDesiredFlywheelSpeed(rpm);
}

void TwoFlywheelRunRPMCommand::end(bool) { flywheel->setDesiredFlywheelSpeed(0); }

}  // namespace src::control::flywheel
