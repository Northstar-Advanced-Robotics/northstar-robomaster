#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include "../../../taproot/src/tap/control/subsystem.hpp"

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "chassis_odometry.hpp"
#include "chassis_subsystem.hpp"
#include "chassis_beyblade_command.hpp"

namespace src::chassis
{

class ChassisAutoDrive 
{   
public:

    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry, ChassisBeybladeCommand* beybladeCommand,
    src::control::governor::MatchRunningGovernor* matchRunningGovernor) : 
        chassisSubsystem(chassis),
        chassisOdometry(chassisOdometry),
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
    ChassisOdometry* chassisOdometry;

    ChassisBeybladeCommand* beybladeCommand;
    src::control::governor::MatchRunningGovernor* matchRunningGovernor;

    bool autoDriveEnabled = true;
    
};

}  // namespace src::chassis

#endif
