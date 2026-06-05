#ifdef TARGET_TEST_BED

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

#include "ref_system_constants.hpp"

// Subsystems
#include "control/imu/imu_calibrate_command.hpp"

#include "test_def.hpp"
#include "using_agitator.hpp"
#include "using_chassis.hpp"
#include "using_flywheel.hpp"
#include "using_hud.hpp"
#include "using_turret.hpp"

namespace testbed_control
{
src::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

#ifdef USING_FLYWHEEL

#endif  // USING_FLYWHEEL

void initializeSubsystems(src::testbed::Drivers *drivers)
{
    dummySubsystem.initialize();
#ifdef USING_AGITATOR
    agitator.initialize();
#endif
#ifdef USING_FLYWHEEL
    flywheel.initialize();
#endif
#ifdef USING_TURRET
    turretSubsystem.initialize();
#endif  // USING_TURRET
#ifdef USING_CHASSIS
    chassisSubsystem.initialize();
#endif
#if defined(USING_TURRET) && defined(USING_REV)
    revTurret.initialize();
#endif
#ifdef USING_HUD
#endif
}

void registerTestSubsystems(src::testbed::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);

#ifdef USING_AGITATOR
    drivers->commandScheduler.registerSubsystem(&agitator);
#endif
#ifdef USING_FLYWHEEL
    drivers->commandScheduler.registerSubsystem(&flywheel);
#endif
#ifdef USING_TURRET
    drivers->commandScheduler.registerSubsystem(&turretSubsystem);
#endif  // USING_TURRET
#ifdef USING_CHASSIS
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
#endif
#ifdef REV_TEST
    drivers->commandScheduler.registerSubsystem(&revMotorTesterSingleMotor);
#endif
#if defined(USING_TURRET) && defined(USING_REV)
    drivers->commandScheduler.registerSubsystem(&revTurret);
#endif
#ifdef USING_HUD
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
#endif
}

void setDefaultTestCommands(src::testbed::Drivers *drivers)
{
#ifdef USING_TURRET
    turretSubsystem.setDefaultCommand(&turretUserControlCommand);
#endif  // USING_TURRET
#ifdef USING_CHASSIS
    chassisSubsystem.setDefaultCommand(&chassisOrientDriveCommand);
#endif
#if defined(USING_TURRET) && defined(USING_REV)
    revTurret.setDefaultCommand(&turretUserControlCommand);
#endif
#ifdef USING_HUD
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
#endif
}

void startTestCommands(src::testbed::Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(tap::algorithms::transforms::Transform(0, 0, 0, 0, 0, 0));
}

void registerTestIoMappings(src::testbed::Drivers *drivers)
{
#ifdef USING_AGITATOR
    // rightSwitchUp10RPS
    // rightSwitchMid20RPS
    // rightSwitchDownFullAuto
    // leftSwitchDownPressedShoot
#endif
#ifdef USING_FLYWHEEL
    drivers->commandMapper.addMap(&fPressed);
#endif
#ifdef USING_TURRET
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&turretTestCommandMapping);
#endif  // USING_TURRET
    // drivers->commandMapper.addMap(&ctrlCPressed);
#ifdef USING_CHASSIS
    drivers->commandMapper.addMap(&bPressed);
#endif
}
}  // namespace testbed_control

namespace src::testbed
{
src::control::imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return nullptr; }

void initSubsystemCommands(src::testbed::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems(drivers);
    testbed_control::registerTestSubsystems(drivers);
    testbed_control::setDefaultTestCommands(drivers);
    testbed_control::startTestCommands(drivers);
    testbed_control::registerTestIoMappings(drivers);
}
}  // namespace src::testbed

#endif