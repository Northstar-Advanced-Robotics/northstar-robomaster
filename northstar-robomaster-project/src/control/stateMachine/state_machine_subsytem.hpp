#ifndef STATE_MACHINE_SUBSYTEM_HPP
#define STATE_MACHINE_SUBSYTEM_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"

namespace src::stateMachine
{
class StateMachineSubsystem : public tap::control::Subsystem
{
public:
    StateMachineSubsystem(
        tap::Drivers* drivers,
        src::chassis::ChassisSubsystem* chassisSubsystem,
        src::chassis::ChassisAutoDrive* chassisAutoDrive,
        src::chassis::ChassisBeybladeCommand* beybladeCommand);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "StateMachine"; }

private:
    src::chassis::ChassisSubsystem* chassisSubsystem;
    src::chassis::ChassisAutoDrive* chassisAutoDrive;
    src::chassis::ChassisBeybladeCommand* beybladeCommand;

    tap::Drivers* drivers;
    uint32_t prevTime = 0;
};

}  // namespace src::stateMachine

#endif