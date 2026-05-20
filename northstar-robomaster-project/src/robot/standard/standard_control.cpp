#ifdef TARGET_STANDARD

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
#include "robot/standard/standard_drivers.hpp"

#include "drivers_singleton.hpp"

// chassis
#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_drive_distance_command.hpp"
#include "control/chassis/chassis_drive_to_point_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_sprint_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// turret
#include "tap/motor/double_dji_motor.hpp"

#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/test/turret_test_command.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"
#include "robot/standard/standard_turret_subsystem.hpp"

// cv
#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/governor/cv_on_target_governor.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"

// flywheel
#include "control/flywheel/dji_two_flywheel_subsystem.hpp"
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/two_flywheel_run_command.hpp"

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

// BUZZER
#include "control/buzzer/buzzer_subsystem.hpp"
#include "control/buzzer/play_song_command.hpp"
#include "control/buzzer/song/megalovania.hpp"
#include "control/buzzer/song/tuff_startup_noise.hpp"
#include "control/buzzer/song/twinkle_twinkle.hpp"

// HUD
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/clientDisplay/client_display_command.hpp"
#include "control/clientDisplay/client_display_subsystem.hpp"
#include "control/clientDisplay/graphics/core/infantry_draw_command.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/indicators/ammo_indicator.hpp"
#include "control/clientDisplay/indicators/circle_crosshair.hpp"
#include "control/clientDisplay/indicators/cv_aiming_indicator.hpp"
#include "control/clientDisplay/indicators/flywheel_indicator.hpp"
#include "control/clientDisplay/indicators/hud_indicator.hpp"
#include "control/clientDisplay/indicators/shooting_mode_indicator.hpp"
#include "control/clientDisplay/indicators/text_hud_indicators.hpp"
#include "control/clientDisplay/indicators/vision_indicator.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::standard;
using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::control::governor;
using namespace tap::control::governor;
// using namespace src::control::client_display;
using namespace src::control::client_display::graphics;
using namespace tap::communication::serial;
using namespace src::control::buzzer;

driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
DummySubsystem dummySubsystem(drivers());

inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }

// songs
BuzzerSubsystem buzzerSubsystem(drivers());
PlaySongCommand playStartupSongCommand(&buzzerSubsystem, tsnSong);

RemoteMapState ctrlShiftZPressed({Remote::Key::CTRL, Remote::Key::SHIFT, Remote::Key::Z});
auto ctrlShiftZSong = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command *>{&playStartupSongCommand},
    &ctrlShiftZPressed);

// flywheel subsystem
DJITwoFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, CAN_BUS);

// flywheel commands
TwoFlywheelRunCommand flywheelRunCommand(&flywheel, 24.0f);

// flywheel mappings
RemoteMapState fPressed({tap::communication::serial::Remote::Key::F});
auto fPressedFlywheels = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&flywheelRunCommand},
    &fPressed);

RemoteMapState leftSwitchUpState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
auto leftSwitchUpFlywheels = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&flywheelRunCommand},
    &leftSwitchUpState);

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

// tap::motor::DoubleDjiMotor yawMotor(
//     drivers(),
//     YAW_MOTOR_ID_1,
//     YAW_MOTOR_ID_2,
//     CAN_BUS_YAW,
//     CAN_BUS_YAW,
//     true,
//     true,
//     "YawMotor1",
//     "YawMotor2",
//     false,
//     1,  // tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 *(1.0f / 1.5f),
//     YAW_MOTOR_CONFIG.startEncoderValue,
//     &drivers()->encoder);

tap::motor::DoubleDjiMotor yawMotor2(
    drivers(),
    YAW_MOTOR_ID_1,
    YAW_MOTOR_ID_2,
    CAN_BUS_YAW,
    CAN_BUS_YAW,
    true,
    true,
    "YawMotor1",
    "YawMotor2",
    false,
    1,  // tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 *(54.0f / 81.0f),
    YAW_MOTOR_CONFIG.startEncoderValue,
    &drivers()->encoder);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID_1,
    CAN_BUS_YAW,
    true,
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
    &getTurretMCBCanComm());

// turret controlers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    turret.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu can com giving imu data from turret to chassis
algorithms::
    WorldFramePitchTurretCanImuCascadePidTurretController worldFramePitchTurretCanImuController(
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretCanImuCascadePidTurretController worldFrameYawTurretCanImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// for imu fixed on turret
algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    *drivers(),
    turret.pitchMotor,
    worldFramePitchTurretPosPid,
    worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    *drivers(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// turret commands
user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

// user::TurretQuickTurnCommand turret180TurnCommand(&turret, modm::toRadian(180));

// RemoteMapState ePressed({Remote::Key::Q});
// auto ePressed180 = std::make_unique<PressCommandMapping>(
//     drivers(),
//     std::vector<Command *>{&turret180TurnCommand},
//     &ePressed);

RemoteMapState xCtrlPressed({Remote::Key::X, Remote::Key::CTRL});
auto xCtrlPressedCvControl = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretCVControlCommand},
    &xCtrlPressed);

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator governors
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommandFullAuto(
    &dummySubsystem,
    manualFireRateReselectionManager,
    40,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand10RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    10,
    &rotateAgitator);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

CvOnTargetGovernor cvOnTargetGovernor(drivers(), drivers()->visionComms, turretCVControlCommand);

RemoteMapState rPressed({Remote::Key::R});
auto rPressedCVGovernoreToggle =
    std::make_unique<CycleStateCommandMapping<bool, 2, CvOnTargetGovernor>>(
        drivers(),
        &rPressed,
        true,
        &cvOnTargetGovernor,
        &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

RemoteMapState leftMousePressed(RemoteMapState::MouseButton::LEFT);
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
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    leftSwitchDown,
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

RemoteMapState gPressed({Remote::Key::G});
RemoteMapState vPressed({Remote::Key::V});
auto gOrVPressedCycleShotSpeed = std::make_unique<CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>>(
    drivers(),
    &gPressed,
    MultiShotCvCommandMapping::SINGLE,
    leftMousePressedShoot.get(),
    &MultiShotCvCommandMapping::setShooterState,
    vPressed);

// chassis odometry
src::chassis::ChassisOdometry *chassisOdometry = new src::chassis::ChassisOdometry(
    &drivers()->bmi088,
    &turret.yawMotor,
    src::chassis::DIST_TO_CENTER,
    src::chassis::WHEEL_DIAMETER_M);

// chassis subsystem
src::chassis::ChassisSubsystem chassisSubsystem(
    drivers(),
    src::chassis::ChassisConfig{
        .leftFrontId = src::chassis::LEFT_FRONT_MOTOR_ID,
        .leftBackId = src::chassis::LEFT_BACK_MOTOR_ID,
        .rightBackId = src::chassis::RIGHT_BACK_MOTOR_ID,
        .rightFrontId = src::chassis::RIGHT_FRONT_MOTOR_ID,
        .canBus = CanBus::CAN_BUS1,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(
            src::chassis::VELOCITY_PID_KP,
            src::chassis::VELOCITY_PID_KI,
            src::chassis::VELOCITY_PID_KD,
            src::chassis::VELOCITY_PID_MAX_ERROR_SUM),
    },
    &drivers()->turretMCBCanCommBus2,
    &turret.yawMotor,
    chassisOdometry);

src::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisBeybladeCommand chassisBeyBladeCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    -1,
    true);

src::chassis::ChassisWiggleCommand chassisWiggleCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1.0f,
    M_TWOPI);

src::chassis::ChassisDriveToPointCommand driveToOneMeterForward(
    &chassisSubsystem,
    chassisOdometry,
    0,
    1,
    0.02);

src::chassis::ChassisSprintCommand chassisSprintCommand(&chassisSubsystem);

Trigger chassisSprint =
    TriggerHelpers::button(drivers(), Remote::Key::SHIFT).whileTrue(&chassisSprintCommand);

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// chassis Mappings
RemoteMapState bPressedNotCntl({Remote::Key::B}, {Remote::Key::CTRL});
auto bPressedNotCntlPressedBeyblade = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisBeyBladeCommand},
    &bPressedNotCntl);

RemoteMapState qPressed({tap::communication::serial::Remote::Key::Q});
auto qPressedNormDrive = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisDriveCommand},
    &qPressed);

RemoteMapState ePressed({tap::communication::serial::Remote::Key::E});
auto ePressedOrientDrive = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisOrientDriveCommand},
    &ePressed);

RemoteMapState zPressedNotCtrl({tap::communication::serial::Remote::Key::Z}, {Remote::Key::CTRL});
auto zPressedNotCtrlWiggle = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisWiggleCommand},
    &zPressedNotCtrl);

RemoteMapState rightSwitchDown(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN);
auto rightSwiitchDownBeyblade = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisBeyBladeCommand},
    &rightSwitchDown,
    true);

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

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

RemoteMapState xPressed({tap::communication::serial::Remote::Key::X});
auto xPressedIMUCalibrate = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&imuCalibrateCommand},
    &xPressed);

src::control::client_display::graphics::UISubsystem ui(drivers());
src::control::client_display::graphics::InfantryDrawCommand infantryDrawCommand(
    drivers(),
    &ui,
    &turret,
    // &flywheel,
    &agitator,
    &chassisSubsystem);

void initializeSubsystems([[maybe_unused]] Drivers *drivers)
{
    dummySubsystem.initialize();
    chassisSubsystem.initialize();
    agitator.initialize();
    flywheel.initialize();
    turret.initialize();
    buzzerSubsystem.initialize();
}

void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&buzzerSubsystem);
    drivers->commandScheduler.registerSubsystem(&ui);
}

void setDefaultStandardCommands([[maybe_unused]] Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(
        &chassisOrientDriveCommand);                      //&chassisOrientDriveCommand);  //
    turret.setDefaultCommand(&turretUserControlCommand);  // when mcb is mounted on turret
    ui.setDefaultCommand(&infantryDrawCommand);
}

void startStandardCommands(Drivers *drivers)
{
    drivers->visionComms.attachOdometry(chassisOdometry);
    drivers->visionComms.attachPitchMotor(&pitchMotor);

    drivers->bmi088.setMountingTransform(tap::algorithms::transforms::Transform(
        0,
        0,
        0,
        0,
        modm::toRadian(180),    // modm::toRadian(-135),
        modm::toRadian(180)));  //-90 for current standard
}

void registerStandardIoMappings(Drivers *drivers)
{
    // Use ) to pass the raw pointers from the unique_ptrs to the command mapper
    drivers->commandMapper.addMap(std::move(leftMousePressedShoot));
    // drivers->commandMapper.addMap(vPressed));
    drivers->commandMapper.addMap(std::move(fPressedFlywheels));
    drivers->commandMapper.addMap(std::move(bPressedNotCntlPressedBeyblade));
    drivers->commandMapper.addMap(std::move(xCtrlPressedCvControl));
    // drivers->commandMapper.addMap(std::move(xPressedIMUCalibrate));
    drivers->commandMapper.addMap(std::move(rPressedCVGovernoreToggle));
    drivers->commandMapper.addMap(std::move(gOrVPressedCycleShotSpeed));
    drivers->commandMapper.addMap(std::move(zPressedNotCtrlWiggle));
    drivers->commandMapper.addMap(std::move(ePressedOrientDrive));
    drivers->commandMapper.addMap(std::move(qPressedNormDrive));
    drivers->commandMapper.addMap(std::move(rightSwiitchDownBeyblade));
    drivers->commandMapper.addMap(std::move(leftSwitchDownPressedShoot));
    drivers->commandMapper.addMap(std::move(leftSwitchUpFlywheels));
    drivers->commandMapper.addMap(std::move(ctrlShiftZSong));
    // drivers->commandMapper.addMap(std::move(ePressed180));
}
}  // namespace standard_control

namespace src::standard
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}

void initSubsystemCommands(src::standard::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems(drivers);
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace src::standard

#endif