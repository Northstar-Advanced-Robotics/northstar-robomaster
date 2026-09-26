#ifdef TARGET_SENTRY

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
#include "robot/sentry/sentry_drivers.hpp"

#include "drivers_singleton.hpp"

// chassis
#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_drive_distance_command.hpp"
#include "control/chassis/chassis_drive_to_point_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "control/chassis/odometry_reset_command.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// turret
#include "control/turret/algorithms/chassis_frame_imu_cal_turret_controller.hpp"
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "robot/sentry/sentry_cv_manager_command.hpp"
#include "robot/sentry/sentry_scan_command.hpp"

// cv
#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/governor/cv_on_target_governor.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"
#include "control/turret/cv/turret_cv_targeting_toggle_command.hpp"

// flywheel
#include "control/flywheel/dji_two_flywheel_subsystem.hpp"
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/two_flywheel_run_command.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/imu_calibrating_governor.hpp"
#include "control/governor/match_running_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// STATE MACHINE
#include "control/stateMachine/state_machine_subsytem.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

// songs
#include "control/buzzer/buzzer_subsystem.hpp"
#include "control/buzzer/play_song_command.hpp"
#include "control/buzzer/song/rouser.hpp"
#include "control/buzzer/song/tuff_startup_noise.hpp"

// hud
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/client_display/sentry_draw_command.hpp"
#include "control/client_display/ui_subsystem.hpp"

using tap::can::CanBus;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::robot::sentry;
using namespace src::control::turret;
using namespace src::control;
using namespace src::control::flywheel;
using namespace src::control::agitator;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace src::control::buzzer;

driversFunc drivers = DoNotUse_getDrivers;

namespace src::robot::sentry
{
DummySubsystem dummySubsystem(drivers());

// songs
BuzzerSubsystem buzzerSubsystem(drivers());
PlaySongCommand playStartupSongCommand(&buzzerSubsystem, tsnSong);
// PlaySongCommand playStartupSongCommand(&buzzerSubsystem, rouser_song);

// flywheel subsystem
DJITwoFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, CAN_BUS);

// flywheel commands
TwoFlywheelRunCommand flywheelRunCommand(&flywheel, 19.3f, &drivers()->refSerial);

// flywheel mappings
Trigger xPressedFlywheels =
    TriggerHelpers::button(drivers(), Remote::Key::X).toggleOnTrue(&flywheelRunCommand);

Trigger leftSwitchUpFlywheels =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .toggleOnTrue(&flywheelRunCommand);

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    src::control::agitator::AGITATOR_PID_CONFIG,
    src::control::agitator::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(
    agitator,
    src::control::agitator::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, src::control::agitator::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator governors
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM,
    src::control::agitator::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM);

ManualFireRateReselectionManager manualFireRateReselectionManager;

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

extern TurretCVControlCommand turretCVControlCommand;
CvOnTargetGovernor cvOnTargetGovernor(
    drivers(),
    drivers()->visionComms,
    turretCVControlCommand,
    0,
    true);

RemoteMapState cPressedNotCtrl({Remote::Key::C}, {Remote::Key::CTRL});
auto cPressedNotCtrlCVGovernorToggle =
    std::make_unique<CycleStateCommandMapping<bool, 2, CvOnTargetGovernor>>(
        drivers(),
        &cPressedNotCtrl,
        true,
        &cvOnTargetGovernor,
        &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&cvOnTargetGovernor});

RemoteMapState leftMousePressed;
auto leftMousePressedShoot = std::make_unique<MultiShotCvCommandMapping>(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    leftMousePressed,
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

RemoteMapState leftSwitchDown(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
auto leftSwitchDownPressedShoot = std::make_unique<MultiShotCvCommandMapping>(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatLimiting,
    leftSwitchDown,
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

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

// turret subsystem
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH,
    false,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID_1,
    CAN_BUS_YAW,
    false,
    "YawMotor1",
    false,
    1,  // tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 *(54.0f / 81.0f),
    YAW_MOTOR_CONFIG.startEncoderValue,
    &drivers()->encoder);

TurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &yawMotor.getInternalEncoder());

// turret controlers
src::control::turret::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

src::control::turret::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

src::control::turret::
    ChassisFramePitchImuCalTurretController chassisFrameImuCalPitchTurretController(
        turret.pitchMotor,
        chassis_rel::PITCH_IMU_CAL_PID_CONFIG,
        modm::toRadian(15),
        4000,
        modm::toRadian(4));

src::control::turret::ChassisFrameYawImuCalTurretController chassisFrameImuCalYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_IMU_CAL_PID_CONFIG,
    modm::toRadian(15),
    4000,
    modm::toRadian(4));

src::control::turret::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

src::control::turret::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    turret.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu fixed on turret
src::control::turret::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
        *drivers(),
        turret.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

src::control::turret::
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
        *drivers(),
        turret.yawMotor,
        worldFrameYawTurretPosPid,
        worldFrameYawTurretVelPid);

// turret commands
TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

TurretCVTargetingToggleCommand turretCvTargetingToggleCommand(
    &dummySubsystem,
    &turretCVControlCommand);

Trigger vPressedTurretCvTargetingToggleCommand =
    TriggerHelpers::button(drivers(), Remote::Key::V).onTrue(&turretCvTargetingToggleCommand);

Trigger rightMousePressedCvControl =
    TriggerHelpers::rightMouseButton(drivers()).whileTrue(&turretCVControlCommand);

// chassis odometry
src::control::chassis::ChassisOdometry *chassisOdometry =
    new src::control::chassis::ChassisOdometry(
        &drivers()->bmi088,
        &turret.yawMotor,
        src::control::chassis::DIST_TO_CENTER,
        src::control::chassis::WHEEL_DIAMETER_M);

// chassis subsystem
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
    &turret.yawMotor,
    chassisOdometry);

// chassis auto drive
src::control::chassis::ChassisAutoDrive *chassisAutoDrive =
    new src::control::chassis::ChassisAutoDrive(&chassisSubsystem, chassisOdometry);

src::control::chassis::OdometryResetCommand odometryResetCommand(
    &chassisSubsystem,
    chassisOdometry);

src::control::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::control::chassis::ChassisFieldCommand chassisFieldCommand(
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

src::control::chassis::ChassisDriveToPointCommand driveToOneMeterForward(
    &chassisSubsystem,
    chassisOdometry,
    0,
    1,
    0.02);

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// chassis Mappings
Trigger fPressedBeyblade =
    TriggerHelpers::button(drivers(), Remote::Key::F).whileTrue(&chassisBeyBladeCommand);

Trigger rPressedOrientDrive =
    TriggerHelpers::button(drivers(), Remote::Key::R).toggleOnTrue(&chassisOrientDriveCommand);

Trigger bPressedNormDrive =
    TriggerHelpers::button(drivers(), Remote::Key::B).toggleOnTrue(&chassisDriveCommand);

Trigger gPressedWiggle =
    TriggerHelpers::button(drivers(), Remote::Key::G).toggleOnTrue(&chassisWiggleCommand);

Trigger rightswitchDownBeyblade =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&chassisBeyBladeCommand);

// sentry scan
SentryScanCommand sentryScanCommand(
    drivers(),
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    chassisOdometry,
    cv::SCAN_MIN_PITCH_ANGLE,
    cv::SCAN_MAX_PITCH_ANGLE,
    cv::SCAN_PITCH_SPEED,
    cv::SCAN_YAW_SPEED);

MatchRunningGovernor matchRunningGovernor(drivers()->refSerial);

Trigger cvGate =
    (TriggerHelpers::switchState(
         drivers(),
         Remote::Switch::RIGHT_SWITCH,
         Remote::SwitchState::UP) ||
     Trigger(drivers(), []() { return matchRunningGovernor.isReady(); }));
Trigger hasTarget =
    Trigger(drivers(), []() { return drivers()->visionComms.getSomeTurretHasTarget(); });

Trigger cvControl = (cvGate && hasTarget).whileTrue(&turretCVControlCommand);
Trigger scan = (cvGate && !hasTarget).whileTrue(&sentryScanCommand);

// imu commands
imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassisSubsystem,
    &playStartupSongCommand);

Trigger ctrlZPressedImuCal = (TriggerHelpers::button(drivers(), Remote::Key::Z) &&
                              TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                                 .onTrue(&imuCalibrateCommand);

Trigger imuCalWhenWheelRight =
    TriggerHelpers::channelLessThan(drivers(), Remote::Channel::WHEEL, -0.8)
        .onTrue(&imuCalibrateCommand);

ImuCalibratingGovernor imuCalibratingGovernor(drivers());

Trigger switchesMidOrientDriveWhenImuCalibratedAndNotInMatch =
    ((TriggerHelpers::switchState(
          drivers(),
          Remote::Switch::RIGHT_SWITCH,
          Remote::SwitchState::MID) ||
      TriggerHelpers::switchState(
          drivers(),
          Remote::Switch::LEFT_SWITCH,
          Remote::SwitchState::MID)) &&
     Trigger(drivers(), []() { return imuCalibratingGovernor.isReady(); }) &&
     Trigger(drivers(), []() { return !matchRunningGovernor.isReady(); }))
        .whileTrue(&chassisOrientDriveCommand);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// STATE MACHINE
src::control::state_machine::StateMachineSubsystem stateMachineSubsystem =
    src::control::state_machine::StateMachineSubsystem(
        drivers(),
        &chassisSubsystem,
        chassisAutoDrive,
        &chassisBeyBladeCommand,
        &matchRunningGovernor);

src::control::client_display::UISubsystem ui(drivers());
src::control::client_display::SentryDrawCommand sentryDrawCommand(
    drivers(),
    &ui,
    &turret,
    // &flywheel,
    &agitator,
    &chassisSubsystem,
    &flywheelOnGovernor,
    leftMousePressedShoot.get(),
    &imuCalibrateCommand,
    &drivers()->visionComms,
    &cvOnTargetGovernor);

Trigger ctrlCPressedUI = (TriggerHelpers::button(drivers(), Remote::Key::C) &&
                          TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                             .onTrue(&sentryDrawCommand);

void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitator.initialize();
    flywheel.initialize();
    turret.initialize();
    buzzerSubsystem.initialize();
}

void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&stateMachineSubsystem);
    drivers->commandScheduler.registerSubsystem(&buzzerSubsystem);
    drivers->commandScheduler.registerSubsystem(&ui);
}

void setDefaultSentryCommands([[maybe_unused]] Drivers *drivers)
{
    // chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
    ui.setDefaultCommand(&sentryDrawCommand);
}

void startSentryCommands(Drivers *drivers)
{
    drivers->visionComms.attachAutoDrive(chassisAutoDrive);
    drivers->visionComms.attachOdometry(chassisOdometry);
    drivers->visionComms.attachPitchMotor(&pitchMotor);
    drivers->visionComms.attachRemote(&drivers->remote);

    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(0), modm::toRadian(180)));
}
// from RM upside down left hand rule 180 around roll

void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(std::move(leftMousePressedShoot));
    drivers->commandMapper.addMap(std::move(cPressedNotCtrlCVGovernorToggle));
    drivers->commandMapper.addMap(std::move(qOrEPressedCycleShotSpeed));

    drivers->commandMapper.addMap(std::move(leftSwitchDownPressedShoot));

    /// TRIGGERS
    /// Triggers don't need to be added to the command mapper since they register themselves with
    /// the command scheduler when they are constructed, but just listing them here for clarity
    /*
    xPressedFlywheels
    leftSwitchUpFlywheels
    fPressedBeyblade
    rPressedOrientDrive
    bPressedNormDrive
    gPressedWiggle
    rightswitchDownBeyblade
    rightMousePressedCvControl
    vPressedTurretCvTargetingToggleCommand
    cvControl
    scan
    ctrlZPressedImuCal
    imuCalWhenWheelRight
    switchesMidOrientDriveWhenImuCalibratedAndNotInMatch
    ctrlCPressedUI
    */
}

imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return &imuCalibrateCommand; }

void initSubsystemCommands(src::robot::sentry::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    initializeSubsystems(drivers);
    registerSentrySubsystems(drivers);
    setDefaultSentryCommands(drivers);
    startSentryCommands(drivers);
    registerSentryIoMappings(drivers);
}
}  // namespace src::robot::sentry

#endif