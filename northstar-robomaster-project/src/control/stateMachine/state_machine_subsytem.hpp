#ifndef STATE_MACHINE_SUBSYTEM_HPP
#define STATE_MACHINE_SUBSYTEM_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"

namespace src::control::state_machine
{
class StateMachineSubsystem : public tap::control::Subsystem
{
public:
    StateMachineSubsystem(
        tap::Drivers* drivers,
        src::control::chassis::ChassisSubsystem* chassisSubsystem,
        src::control::chassis::ChassisAutoDrive* chassisAutoDrive,
        src::control::chassis::ChassisBeybladeCommand* beybladeCommand,
        src::control::governor::MatchRunningGovernor* matchRunningGovernor);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "StateMachine"; }

private:
    src::control::chassis::ChassisSubsystem* chassisSubsystem;
    src::control::chassis::ChassisAutoDrive* chassisAutoDrive;
    src::control::chassis::ChassisBeybladeCommand* beybladeCommand;
    src::control::governor::MatchRunningGovernor* matchRunningGovernor;

    tap::Drivers* drivers;
    uint32_t prevTime = 0;
};

}  // namespace src::control::state_machine

#endif