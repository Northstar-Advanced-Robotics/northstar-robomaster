#ifdef TARGET_TURRET

#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/turret/constants/turret_constants.hpp"
#include "drivers_singleton.hpp"
#include "robot/robot_control.hpp"
#include "robot/turret/turret_drivers.hpp"

using namespace src::gyro;

driversFunc drivers = DoNotUse_getDrivers;

namespace turret_control
{
void initializeSubsystems(Drivers *) {}

void registerSoldierSubsystems(Drivers *) {}

void setDefaultSoldierCommands(Drivers *) {}

void startSoldierCommands(Drivers *) {}

void registerSoldierIoMappings(Drivers *) {}
}  // namespace turret_control

namespace src::gyro
{
void initSubsystemCommands(src::gyro::Drivers *drivers)
{
    turret_control::initializeSubsystems(drivers);
    turret_control::registerSoldierSubsystems(drivers);
    turret_control::setDefaultSoldierCommands(drivers);
    turret_control::startSoldierCommands(drivers);
    turret_control::registerSoldierIoMappings(drivers);
}

src::control::imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return nullptr; }
}  // namespace src::gyro

#endif
