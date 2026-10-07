#ifdef TARGET_ENGINEER

#include <memory>

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/cycle_state_command_mapping.hpp"
#include "control/dummy_subsystem.hpp"
#include "robot/engineer/engineer_drivers.hpp"

#include "drivers_singleton.hpp"

// chassis
#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_drive_distance_command.hpp"
#include "control/chassis/chassis_drive_to_point_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "control/chassis/holonomic_chassis_subsystem.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

#include "ref_system_constants.hpp"

// BUZZER
#include "control/buzzer/buzzer_subsystem.hpp"
#include "control/buzzer/play_song_command.hpp"
#include "control/buzzer/song/megalovania.hpp"
#include "control/buzzer/song/tuff_startup_noise.hpp"
#include "control/buzzer/song/twinkle_twinkle.hpp"

// HUD
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/client_display/infantry_draw_command.hpp"
#include "control/client_display/ui_subsystem.hpp"

using tap::can::CanBus;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::robot::engineer;
using namespace src::control;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace src::control::buzzer;

driversFunc drivers = DoNotUse_getDrivers;

namespace src::robot::engineer
{
DummySubsystem dummySubsystem(drivers());

// songs
BuzzerSubsystem buzzerSubsystem(drivers());
PlaySongCommand playStartupSongCommand(&buzzerSubsystem, tsnSong);

ManualFireRateReselectionManager manualFireRateReselectionManager;

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

RemoteMapState cPressedNotCtrl({Remote::Key::C}, {Remote::Key::CTRL});
auto cPressedNotCtrlCVGovernorToggle =
    std::make_unique<CycleStateCommandMapping<bool, 2, CvOnTargetGovernor>>(
        drivers(),
        &cPressedNotCtrl,
        true,
        &cvOnTargetGovernor,
        &CvOnTargetGovernor::setGovernorEnabled);

RemoteMapState qPressed({Remote::Key::Q});
RemoteMapState ePressed({Remote::Key::E});
auto qOrEPressedCycleShotSpeed = std::make_unique<CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>>(
    drivers(),
    &qPressed,
    MultiShotCvCommandMapping::SINGLE,
    leftMousePressedShoot.get(),
    &MultiShotCvCommandMapping::setShooterState,
    ePressed);
// TODO this is bad, often acidently get to the wrong state should have CV send a fire rate that
// lines up with the rotation speed, so one 17 per plate at some rotation speed.

// chassis subsystem
// TODO unfuck cordiate frame, make right hand rule.
// TODO get curent and voltage sensors and make good power limiting
src::control::chassis::HolonomicChassisSubsystem chassisSubsystem(
    drivers(),
    src::control::chassis::ChassisConfig{
        .leftFrontId = src::control::chassis::LEFT_FRONT_MOTOR_ID,
        .leftBackId = src::control::chassis::LEFT_BACK_MOTOR_ID,
        .rightBackId = src::control::chassis::RIGHT_BACK_MOTOR_ID,
        .rightFrontId = src::control::chassis::RIGHT_FRONT_MOTOR_ID,
        .canBus = CanBus::CAN_BUS1,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(
            src::control::chassis::VELOCITY_PID_KP,
            src::control::chassis::VELOCITY_PID_KI,
            src::control::chassis::VELOCITY_PID_KD,
            src::control::chassis::VELOCITY_PID_MAX_ERROR_SUM,
            src::control::chassis::VELOCITY_PID_MAX_OUTPUT),
    },
    nullptr,  // engineer yaw motor
    chassisOdometry);

src::control::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::control::chassis::ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::control::chassis::ChassisBeybladeCommand chassisBeyBladeCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    -1,
    true);

src::control::chassis::ChassisWiggleCommand chassisWiggleCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1.0f,
    M_TWOPI);

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// chassis Mappings
Trigger fPressedBeyblade =
    TriggerHelpers::button(drivers(), Remote::Key::F).toggleOnTrue(&chassisBeyBladeCommand);

Trigger rPressedOrientDrive =
    TriggerHelpers::button(drivers(), Remote::Key::R).toggleOnTrue(&chassisOrientDriveCommand);

Trigger bPressedNormDrive =
    TriggerHelpers::button(drivers(), Remote::Key::B).toggleOnTrue(&chassisDriveCommand);

Trigger gPressedWiggle =
    TriggerHelpers::button(drivers(), Remote::Key::G).toggleOnTrue(&chassisWiggleCommand);

Trigger rightswitchDownBeyblade =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&chassisBeyBladeCommand);

// imu commands
// TODO Its anoying that this runs at the start every time, could make it use non imu control
// starting at whatever position the robot was in when it turns on then when you calabrate it
// changes to imu control
/*imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassisSubsystem,
    &playStartupSongCommand);
*/
Trigger ctrlZPressedImuCal = (TriggerHelpers::button(drivers(), Remote::Key::Z) &&
                              TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                                 .onTrue(&imuCalibrateCommand);

Trigger imuCalWhenWheelRight =
    TriggerHelpers::channelLessThan(drivers(), Remote::Channel::WHEEL, -0.8)
        .onTrue(&imuCalibrateCommand);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// TODO Need better ui for cv governor and yaw govornor
src::control::client_display::UISubsystem ui(drivers());
// Make engineer draw cmd

Trigger ctrlCPressedUI = (TriggerHelpers::button(drivers(), Remote::Key::C) &&
                          TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                             .onTrue(&infantryDrawCommand);

void initializeSubsystems([[maybe_unused]] Drivers *drivers)
{
    dummySubsystem.initialize();
    chassisSubsystem.initialize();
    buzzerSubsystem.initialize();
}

void registerengineerSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&buzzerSubsystem);
    drivers->commandScheduler.registerSubsystem(&ui);
}

void setDefaultengineerCommands([[maybe_unused]] Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisOrientDriveCommand);
    ui.setDefaultCommand(&infantryDrawCommand);
}

void startengineerCommands(Drivers *drivers)
{
    drivers->visionComms.attachPitchMotor(&pitchMotor);
    drivers->visionComms.attachOdometry(chassisOdometry);
    drivers->visionComms.attachRemote(&drivers->remote);

    drivers->mpu6500.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(0), modm::toRadian(180)));
}

void registerengineerIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(std::move(leftMousePressedShoot));
    drivers->commandMapper.addMap(std::move(cPressedNotCtrlCVGovernorToggle));
    drivers->commandMapper.addMap(std::move(qOrEPressedCycleShotSpeed));

    drivers->commandMapper.addMap(std::move(leftSwitchDownPressedShoot));

    /// TRIGGERS
    /// Triggers don't need to be added to the command mapper since they register themselves
    /// with the command scheduler when they are constructed, but just listing them here for
    /// clarity
    /*
    xNotCtrlPressedFlywheels
    fPressedBeyblade
    rightMousePressedCvControl
    gPressedWiggle
    rPressedOrientDrive
    bPressedNormDrive
    rightswitchDownBeyblade
    leftSwitchUpFlywheels
    vPressedTurretCvTargetingToggleCommand
    rightSwitchUpCvControl
    ctrlZPressedImuCal
    imuCalWhenWheelRight
    ctrlCPressedUI
    */
}

imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return &imuCalibrateCommand; }

void initSubsystemCommands(src::robot::engineer::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    initializeSubsystems(drivers);
    registerengineerSubsystems(drivers);
    setDefaultengineerCommands(drivers);
    startengineerCommands(drivers);
    registerengineerIoMappings(drivers);
}
}  // namespace src::robot::engineer

#endif