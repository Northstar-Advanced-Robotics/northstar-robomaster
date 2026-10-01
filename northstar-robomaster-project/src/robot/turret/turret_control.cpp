#ifdef TARGET_TURRET

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"

#include "tap/util_macros.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "robot/turret/turret_drivers.hpp"

#include "control/chassis/chassis_drive_command.hpp"
#include "control/imu/imu_calibrate_command.hpp"

using namespace src::robot::turret;

driversFunc drivers = DoNotUse_getDrivers;

namespace src::robot::turret
{
    
    
void initializeSubsystems(Drivers *drivers)
{
    
}

void registerSoldierSubsystems(Drivers *drivers)
{
   
}

void setDefaultSoldierCommands(Drivers *drivers)
{
    
}

void startSoldierCommands(Drivers *drivers) {}

void registerSoldierIoMappings(Drivers *drivers) {}

src::control::imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return nullptr; }

void initSubsystemCommands(src::robot::turret::Drivers *drivers)
{
    initializeSubsystems(drivers);
    registerSoldierSubsystems(drivers);
    setDefaultSoldierCommands(drivers);
    startSoldierCommands(drivers);
    registerSoldierIoMappings(drivers);
}
}  // namespace src::robot::turret

#endif