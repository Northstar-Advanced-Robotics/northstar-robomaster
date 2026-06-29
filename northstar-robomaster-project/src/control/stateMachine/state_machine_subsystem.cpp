#include "tap/algorithms/math_user_utils.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

#include "state_machine_subsytem.hpp"

namespace src::stateMachine
{
StateMachineSubsystem::StateMachineSubsystem(
    tap::Drivers* drivers,
    src::chassis::ChassisSubsystem* chassisSubsystem,
    src::chassis::ChassisAutoDrive* chassisAutoDrive)
    : Subsystem(drivers),
      drivers(drivers),
      chassisSubsystem(chassisSubsystem),
      chassisAutoDrive(chassisAutoDrive)
{
}

void StateMachineSubsystem::initialize() {}

bool beyblade = false;

void StateMachineSubsystem::refresh()
{
    if (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) !=
        tap::communication::serial::Remote::SwitchState::UP)
    {
        chassisSubsystem->setIsSprinting(false);
        return;
    }
    if (!chassisAutoDrive->hasValidPath())
    {
        chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        return;
    }

    chassisAutoDrive->updateAutoDrive();
    return;
    modm::Vector<float, 2> desiredGlobalVelocity = chassisAutoDrive->getDesiredGlobalVelocity();
    float desiredRotation = chassisAutoDrive->getDesiredRotation();
    chassisSubsystem->setIsSprinting(true);
    chassisSubsystem->setVelocityFieldDrive(  // fuck chassis subsystems fucked up coordinate frames
        desiredGlobalVelocity.y,
        -desiredGlobalVelocity.x,
        desiredRotation);
}  // namespace src::stateMachine

}  // namespace src::stateMachine