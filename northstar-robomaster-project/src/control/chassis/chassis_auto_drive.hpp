#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include "chassis_odometry.hpp"
#include "chassis_subsystem.hpp"

namespace src::chassis
{

class ChassisAutoDrive 
{   
public:
    void TryDriveAtVelocity(float x, float y) {
        if (autoDriveEnabled) {
            chassisSubsystem->setVelocityFieldDrive(x, y, 0);
        }
        else {
            chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        }
    }

private:
    src::chassis::ChassisSubsystem* chassisSubsystem;
    bool autoDriveEnabled = true;
    
};

}  // namespace src::chassis

#endif
