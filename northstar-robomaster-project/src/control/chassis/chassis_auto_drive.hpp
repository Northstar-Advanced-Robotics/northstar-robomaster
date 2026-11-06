#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include "control/governor/match_running_governor.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "chassis_subsystem.hpp"
#include "chassis_beyblade_command.hpp"

namespace src::chassis
{

class ChassisAutoDrive 
{   
public:

    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisBeybladeCommand* beybladeCommand,
    src::control::governor::MatchRunningGovernor* matchRunningGovernor) : 
        chassisSubsystem(chassis),
        beybladeCommand(beybladeCommand),
        matchRunningGovernor(matchRunningGovernor) { }

    void TryDriveAtVelocity(float x, float y) {
        if (autoDriveEnabled) {
            chassisSubsystem->setVelocityFieldDrive(x, y, 0);
        }
        else {
            chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        }
    }

private:
    ChassisSubsystem* chassisSubsystem;
    ChassisBeybladeCommand* beybladeCommand;
    src::control::governor::MatchRunningGovernor* matchRunningGovernor;

    bool autoDriveEnabled = true;
    
};

}  // namespace src::chassis

#endif
