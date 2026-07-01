#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "control/governor/match_running_governor.hpp"

#include "state_machine_subsytem.hpp"

namespace src::stateMachine
{
StateMachineSubsystem::StateMachineSubsystem(
    tap::Drivers* drivers,
    src::chassis::ChassisSubsystem* chassisSubsystem,
    src::chassis::ChassisAutoDrive* chassisAutoDrive,
    src::chassis::ChassisBeybladeCommand* beybladeCommand,
    src::control::governor::MatchRunningGovernor* matchRunningGovernor)
    : Subsystem(drivers),
      drivers(drivers),
      chassisSubsystem(chassisSubsystem),
      chassisAutoDrive(chassisAutoDrive),
      beybladeCommand(beybladeCommand),
      matchRunningGovernor(matchRunningGovernor)
{
}

void StateMachineSubsystem::initialize() {}

bool beyblade = true;

void StateMachineSubsystem::refresh()
{
    if ((matchRunningGovernor->isReady() ||
         ((drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
           tap::communication::serial::Remote::SwitchState::UP) &&
          (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
           tap::communication::serial::Remote::SwitchState::DOWN))))
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        uint32_t dt = (prevTime == 0) ? 0 : currTime - prevTime;
        prevTime = currTime;

        if (!chassisAutoDrive->hasValidPath())
        {
            if (beyblade && beybladeCommand != nullptr)
            {
                float maxRot = chassisSubsystem->calculateMaxRotationSpeed(0, 0);
                float rotation = beybladeCommand->calculateBeyBladeRotationSpeed(maxRot, dt);
                chassisSubsystem->isBeybladingOnly = true;
                chassisSubsystem->setVelocityFieldDrive(
                    0,
                    0,
                    rotation * src::chassis::BEYBLADE_SPEEDUP_FACTOR);
            }
            else
            {
                chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
            }
            return;
        }

        chassisAutoDrive->updateAutoDrive();
        modm::Vector<float, 2> desiredGlobalVelocity = chassisAutoDrive->getDesiredGlobalVelocity();
        float desiredRotation = chassisAutoDrive->getDesiredRotation();

        if (beyblade && beybladeCommand != nullptr)
        {
            float maxRot = chassisSubsystem->calculateMaxRotationSpeed(
                desiredGlobalVelocity.y,
                -desiredGlobalVelocity.x);
            desiredRotation = beybladeCommand->calculateBeyBladeRotationSpeed(maxRot, dt);

            if (chassisSubsystem->getChassisOdometry()->getVelocityLocal().getLength() < 0.3f)
            {
                chassisSubsystem->isBeybladingOnly = true;
                desiredRotation *= src::chassis::BEYBLADE_SPEEDUP_FACTOR;
            }
            else
            {
                chassisSubsystem->isBeybladingOnly = false;
            }
        }

        chassisSubsystem->setIsSprinting(true);
        chassisSubsystem->setVelocityFieldDrive(
            desiredGlobalVelocity.y,
            -desiredGlobalVelocity.x,
            desiredRotation);
    }
    else
    {
        chassisSubsystem->setIsSprinting(false);
        return;
    }
}

}  // namespace src::stateMachine
