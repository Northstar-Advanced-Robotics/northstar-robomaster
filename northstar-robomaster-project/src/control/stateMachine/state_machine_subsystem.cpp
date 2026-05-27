//#define FLY_SKY
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

// #include "control/algorithms/CubicBezier.hpp"
// CubicBezier* leftSide = new CubicBezier({0, 0}, {0, 2}, {-1.5f, 0}, {-1.5f, 2});
// CubicBezier* rightSide = new CubicBezier({0, 2}, {0, 0}, {1.5f, 2}, {1.5f, 0});
// bool l = true;

void StateMachineSubsystem::refresh()
{
    // return;
#ifdef FLY_SKY
    if (drivers->remote.getChannel(5))
    {
#else
    if (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) !=
        tap::communication::serial::Remote::SwitchState::UP)
    {
#endif
        chassisSubsystem->setIsSprinting(false);
        return;
    }
    if (!chassisAutoDrive->hasValidPath())
    {
        chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        // if (l)
        // {
        //     chassisAutoDrive->setCurve(leftSide);
        //     l = false;
        // }
        // else
        // {
        //     chassisAutoDrive->setCurve(rightSide);
        //     l = true;
        // }

        return;
    }

    chassisAutoDrive->updateAutoDrive();
    modm::Vector<float, 2> desiredGlobalVelocity = chassisAutoDrive->getDesiredGlobalVelocity();
    float desiredRotation = chassisAutoDrive->getDesiredRotation();
    chassisSubsystem->setIsSprinting(true);
    chassisSubsystem->setVelocityFieldDrive(  // fuck chassis subsystems fucked up coordinate frames
        desiredGlobalVelocity.y,
        -desiredGlobalVelocity.x,
        desiredRotation);
}  // namespace src::stateMachine

}  // namespace src::stateMachine