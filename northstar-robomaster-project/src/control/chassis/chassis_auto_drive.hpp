#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include "control/governor/match_running_governor.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "chassis_subsystem.hpp"
#include "chassis_beyblade_command.hpp"

namespace src::control::chassis
{

class ChassisAutoDrive 
{   
public:

    ChassisAutoDrive(tap::Drivers* drivers, ChassisSubsystem* chassis, ChassisBeybladeCommand* beybladeCommand,
    src::control::governor::MatchRunningGovernor* matchRunningGovernor) : 
        drivers(drivers),
        chassisSubsystem(chassis),
        beybladeCommand(beybladeCommand),
        matchRunningGovernor(matchRunningGovernor) { }

    void TryDriveAtVelocity(float x, float y) {
        if ((matchRunningGovernor->isReady() || ((drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::UP) && (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN)))) {
            chassisSubsystem->setVelocityFieldDrive(x, y, 0);
        }
        else {
            chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        }
    }

private:
    tap::Drivers* drivers;
    ChassisSubsystem* chassisSubsystem;
    ChassisBeybladeCommand* beybladeCommand;
    src::control::governor::MatchRunningGovernor* matchRunningGovernor;

    bool autoDriveEnabled = true;
    
};

}  // namespace src::control::chassis

#endif
