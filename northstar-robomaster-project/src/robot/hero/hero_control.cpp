#ifdef TARGET_HERO

#include <memory>

#include "tap/control/concurrent_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/dummy_subsystem.hpp"
#include "robot/hero/hero_drivers.hpp"

#include "drivers_singleton.hpp"

// chassis
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_sprint_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"

// kicker
#include "control/kicker/constant_velocity_kicker_command.hpp"
#include "control/kicker/constants/kicker_constants.hpp"
#include "control/kicker/kicker_subsystem.hpp"
#include "control/kicker/kicker_subsystem_config.hpp"

// turret
#include "tap/motor/double_dji_motor.hpp"

#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"

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

// HUD
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/clientDisplay/client_display_command.hpp"
#include "control/clientDisplay/client_display_subsystem.hpp"
#include "control/clientDisplay/graphics/core/hero_draw_command.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/indicators/ammo_indicator.hpp"
#include "control/clientDisplay/indicators/circle_crosshair.hpp"
#include "control/clientDisplay/indicators/cv_aiming_indicator.hpp"
#include "control/clientDisplay/indicators/flywheel_indicator.hpp"
#include "control/clientDisplay/indicators/hero_spin_indicator.hpp"
#include "control/clientDisplay/indicators/hud_indicator.hpp"
#include "control/clientDisplay/indicators/shooting_mode_indicator.hpp"
#include "control/clientDisplay/indicators/text_hud_indicators.hpp"
#include "control/clientDisplay/indicators/vision_indicator.hpp"

// BUZZER
#include "control/buzzer/buzzer_subsystem.hpp"
#include "control/buzzer/play_song_command.hpp"
#include "control/buzzer/song/megalovania.hpp"
#include "control/buzzer/song/tuff_startup_noise.hpp"
#include "control/buzzer/song/twinkle_twinkle.hpp"

using tap::can::CanBus;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::control::turret;
using namespace src::control;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::hero;
using namespace src::control::flywheel;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace src::control::client_display;
using namespace tap::communication::serial;
using namespace src::control::buzzer;
using namespace src::control::client_display::graphics;

driversFunc drivers = DoNotUse_getDrivers;

namespace hero_control
{
DummySubsystem dummySubsystem(drivers());

// songs
BuzzerSubsystem buzzerSubsystem(drivers());
PlaySongCommand playStartupSongCommand(&buzzerSubsystem, tsnSong);

// flywheel
DJITwoFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, CAN_BUS, true);

TwoFlywheelRunCommand heroFlywheelRunCommand(&flywheel, 12);

// flywheel mappings
RemoteMapState fPressed =
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F}));
auto fPressedFlywheel = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&heroFlywheelRunCommand},
    &fPressed);

RemoteMapState leftSwitchUp = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
auto leftSwitchUpPressedFlywheel = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&heroFlywheelRunCommand},
    &leftSwitchUp);

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

src::kicker::KickerSubsystem kicker(
    drivers(),
    src::control::kicker::constants::KICKER_PID_CONFIG,
    src::control::kicker::constants::KICKER_CONFIG);

ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

src::control::kicker::ConstantVelocityKickerCommand rotateKicker(&kicker, 40.0f);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// ConcurrentCommand<2> rotateAndUnjamAgitatorWithKicker(
//     {&rotateAndUnjamAgitator, &rotateKicker},
//     "Rotate and Unjam Agitator with Kicker");

// agitator governors
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM,
    constants::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommand1RPS(&dummySubsystem, manualFireRateReselectionManager, 1);
SetFireRateCommand setFireRateCommand5SPR(&dummySubsystem, manualFireRateReselectionManager, .2);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<4> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor,
     &fireRateLimitGovernor,
     &flywheelOnGovernor,
     &heatLimitGovernor});

GovernorLimitedCommand<4>
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunchedWithKicker(
        {&agitator},
        rotateAndUnjamAgitator,
        {&refSystemProjectileLaunchedGovernor,
         &fireRateLimitGovernor,
         &flywheelOnGovernor,
         &heatLimitGovernor});

// agitator mappings
RemoteMapState vPressed({tap::communication::serial::Remote::Key::V});
auto vPressed1RPS = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&setFireRateCommand1RPS},
    &vPressed);

RemoteMapState gPressed({tap::communication::serial::Remote::Key::G});
auto gPressed5RPS = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&setFireRateCommand5SPR},
    &gPressed);

RemoteMapState leftMouse = RemoteMapState(RemoteMapState::MouseButton::LEFT);
auto leftMousePressedShoot = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{
        &rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},  // TODO
    &leftMouse,
    false);

RemoteMapState leftSwitchDown(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
auto leftSwitchDownPressedShoot = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{
        &rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},  // TODO
    &leftSwitchDown);

RemoteMapState rightSwitchUp(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);
auto rightSwitchUpRunKicker = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&rotateKicker},
    &rightSwitchUp);

// turret subsystem
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DoubleDjiMotor yawMotor(
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
    1,  // tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 *(1.0f / 3.6f),
    YAW_MOTOR_CONFIG.startEncoderValue,
    &drivers()->encoder);

TurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, PITCH_MOTOR_CONFIG, YAW_MOTOR_CONFIG);

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
    &worldFramePitchTurretImuController,  //&worldFramePitchChassisImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

// user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
//     drivers(),
//     drivers()->controlOperatorInterface,
//     &turret,
//     &worldFrameYawChassisImuController,
//     &worldFramePitchChassisImuController,
//     &worldFrameYawTurretCanImuController,
//     &worldFramePitchTurretCanImuController,
//     USER_YAW_INPUT_SCALAR,
//     USER_PITCH_INPUT_SCALAR);

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
    0.6f,
    M_PI * 4 / 3);

src::chassis::ChassisSprintCommand chassisSprintCommand(&chassisSubsystem);

Trigger chassisSprint =
    TriggerHelpers::button(drivers(), Remote::Key::SHIFT).whileTrue(&chassisSprintCommand);

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// GovernorWithFallbackCommand<2> beyBladeSlowOutOfCombat(
//     {&chassisSubsystem},
//     chassisBeyBladeSlowCommand,
//     chassisBeyBladeCommand,
//     {&firedRecentlyGovernor, &plateHitGovernor},
//     true);

// chassis Mappings
RemoteMapState bPressedNotCntl({Remote::Key::B}, {Remote::Key::CTRL});
auto bPressedNotCntlBeyblade = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisBeyBladeCommand},
    &bPressedNotCntl);

// imu commands
imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        false,
    }},
    &chassisSubsystem,
    &playStartupSongCommand);

RemoteMapState xPressed({tap::communication::serial::Remote::Key::X});
auto xPressedIMUCalibrate = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&imuCalibrateCommand},
    &xPressed);

RemoteMapState zPressed({tap::communication::serial::Remote::Key::Z});
auto zPressedWiggle = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisWiggleCommand},
    &zPressed);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// HUD
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);

CircleCrosshair circleCrosshair(refSerialTransmitter);

FlywheelIndicator flyWheelIndicator(refSerialTransmitter, drivers()->refSerial, flywheelOnGovernor);

HeroSpinIndicator heroSpinIndicator(
    refSerialTransmitter,
    drivers()->refSerial,
    *drivers(),
    &chassisBeyBladeCommand,
    &chassisWiggleCommand);

std::vector<HudIndicator *> hudIndicators = {
    &ammoIndicator,
    &circleCrosshair,
    // &textHudIndicators,
    &flyWheelIndicator,
    &heroSpinIndicator};

src::control::client_display::graphics::UISubsystem ui(drivers());
src::control::client_display::graphics::HeroDrawCommand heroDrawCommand(
    drivers(),
    &ui,
    &turret,
    // &flywheel,
    &agitator,
    &chassisSubsystem);

RemoteMapState ctrlShiftB({Remote::Key::CTRL, Remote::Key::SHIFT, Remote::Key::E});
auto ctrlShiftBPressedClientDisplay = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command *>{&heroDrawCommand},
    &ctrlShiftB);

void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitator.initialize();
    kicker.initialize();
    turret.initialize();
    flywheel.initialize();
}

void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&kicker);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&buzzerSubsystem);
    drivers->commandScheduler.registerSubsystem(&ui);
}

void setDefaultHeroCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);  // chassisOrientDriveCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
    ui.setDefaultCommand(&heroDrawCommand);
}

void startHeroCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(tap::algorithms::transforms::Transform(
        0,
        0,
        0,
        0,
        modm::toRadian(180),
        modm::toRadian(180)));
    // pitch up needs to be negitive up is on motor side
    // right neg
}

void registerHeroIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(std::move(leftMousePressedShoot));
    drivers->commandMapper.addMap(std::move(fPressedFlywheel));
    drivers->commandMapper.addMap(std::move(leftSwitchDownPressedShoot));
    drivers->commandMapper.addMap(std::move(leftSwitchUpPressedFlywheel));

    // drivers->commandMapper.addMap(vPressed));
    drivers->commandMapper.addMap(std::move(bPressedNotCntlBeyblade));
    drivers->commandMapper.addMap(std::move(gPressed5RPS));
    drivers->commandMapper.addMap(std::move(xPressedIMUCalibrate));
    drivers->commandMapper.addMap(std::move(zPressedWiggle));

    // drivers->commandMapper.addMap(ctrlShiftBPressedClientDisplay));
    drivers->commandMapper.addMap(std::move(rightSwitchUpRunKicker));
}
}  // namespace hero_control

namespace src::hero
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return nullptr;  //&hero_control::imuCalibrateCommand;
}

void initSubsystemCommands(src::hero::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &hero_control::remoteSafeDisconnectFunction);
    hero_control::initializeSubsystems(drivers);
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands(drivers);
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace src::hero

#endif  // TARGET_HERO