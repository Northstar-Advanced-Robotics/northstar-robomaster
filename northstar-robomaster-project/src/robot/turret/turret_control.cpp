#ifdef TURRET

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"
#include "../../robot-type/robot_type.hpp"

#include "tap/util_macros.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "robot/turret/turret_drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_drive_command.hpp"

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