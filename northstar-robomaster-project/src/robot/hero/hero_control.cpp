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

#include "control/cycle_state_command_mapping.hpp"
#include "control/dummy_subsystem.hpp"
#include "robot/hero/hero_drivers.hpp"

#include "drivers_singleton.hpp"

// chassis
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
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

#include "control/turret/algorithms/chassis_frame_imu_cal_turret_controller.hpp"
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"

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

TwoFlywheelRunCommand flywheelRunCommand(&flywheel, 12);

// flywheel mappings
RemoteMapState xPressed({tap::communication::serial::Remote::Key::X});
auto xPressedFlywheels = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&flywheelRunCommand},
    &xPressed);

RemoteMapState leftSwitchUp(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
auto leftSwitchUpFlywheels = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&flywheelRunCommand},
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
SetFireRateCommand setFireRateCommand5SPR(&dummySubsystem, manualFireRateReselectionManager, 5);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

// Trigger rightSwitchUpRotateAndUnjamAgitator =
//     TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)
//         .onTrue(&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched);

extern cv::TurretCVControlCommand turretCVControlCommand;
CvOnTargetGovernor cvOnTargetGovernor(drivers(), drivers()->visionComms, turretCVControlCommand);

RemoteMapState cPressed({Remote::Key::C});
auto cPressedCVGovernorToggle =
    std::make_unique<CycleStateCommandMapping<bool, 2, CvOnTargetGovernor>>(
        drivers(),
        &cPressed,
        true,
        &cvOnTargetGovernor,
        &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

// agitator mappings
RemoteMapState qPressed({tap::communication::serial::Remote::Key::Q});
auto qPressed1RPS = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&setFireRateCommand1RPS},
    &qPressed);

RemoteMapState ePressed({tap::communication::serial::Remote::Key::E});
auto ePressed5RPS = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&setFireRateCommand5SPR},
    &ePressed);

RemoteMapState leftMousePressed = RemoteMapState(RemoteMapState::MouseButton::LEFT);
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

algorithms::ChassisFramePitchImuCalTurretController chassisFrameImuCalPitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_IMU_CAL_PID_CONFIG,
    modm::toRadian(15),
    4000,
    modm::toRadian(4));

algorithms::ChassisFrameYawImuCalTurretController chassisFrameImuCalYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_IMU_CAL_PID_CONFIG,
    modm::toRadian(15),
    4000,
    modm::toRadian(4));

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

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

RemoteMapState rightMousePressed(RemoteMapState::MouseButton::RIGHT);
auto rightMousePressedCvControl = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretCVControlCommand},
    &rightMousePressed,
    true);

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
    &turret.yawMotor);

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

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// chassis Mappings
RemoteMapState fPressed({Remote::Key::F});
auto fPressedBeyblade = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisBeyBladeCommand},
    &fPressed,
    true);

RemoteMapState rPressed({Remote::Key::R});
auto rPressedOrientDrive = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisOrientDriveCommand},
    &rPressed);

RemoteMapState bPressed({Remote::Key::B});
auto bPressedNormDrive = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisDriveCommand},
    &bPressed);

RemoteMapState gPressed({Remote::Key::G});
auto gPressedWiggle = std::make_unique<ToggleCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisWiggleCommand},
    &gPressed);

RemoteMapState rightSwitchDown(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN);
auto rightSwitchDownBeyblade = std::make_unique<HoldRepeatCommandMapping>(
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
        false,
    }},
    &chassisSubsystem,
    &playStartupSongCommand);

Trigger ctrlZPressedImuCal = (TriggerHelpers::button(drivers(), Remote::Key::Z) &&
                              TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                                 .onTrue(&imuCalibrateCommand);

Trigger imuCalWhenWheelRight =
    TriggerHelpers::channelLessThan(drivers(), Remote::Channel::WHEEL, -0.8)
        .onTrue(&imuCalibrateCommand);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// HUD
src::control::client_display::graphics::UISubsystem ui(drivers());
src::control::client_display::graphics::HeroDrawCommand heroDrawCommand(
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

Trigger ctrlCPressedUI = TriggerHelpers::button(drivers(), Remote::Key::C).onTrue(&heroDrawCommand);

void initializeSubsystems(Drivers *drivers)
{
    dummySubsystem.initialize();
    chassisSubsystem.initialize();
    agitator.initialize();
    kicker.initialize();
    turret.initialize();
    flywheel.initialize();
    buzzerSubsystem.initialize();
}

void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&kicker);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&buzzerSubsystem);
    drivers->commandScheduler.registerSubsystem(&ui);
}

void setDefaultHeroCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisOrientDriveCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
    ui.setDefaultCommand(&heroDrawCommand);
}

void startHeroCommands(Drivers *drivers)
{
    drivers->visionComms.attachPitchMotor(&pitchMotor);
    drivers->visionComms.attachRemote(&drivers->remote);

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
    drivers->commandMapper.addMap(std::move(xPressedFlywheels));
    drivers->commandMapper.addMap(std::move(fPressedBeyblade));
    drivers->commandMapper.addMap(std::move(rightMousePressedCvControl));
    drivers->commandMapper.addMap(std::move(cPressedCVGovernorToggle));
    drivers->commandMapper.addMap(std::move(qPressed1RPS));
    drivers->commandMapper.addMap(std::move(ePressed5RPS));
    drivers->commandMapper.addMap(std::move(gPressedWiggle));
    drivers->commandMapper.addMap(std::move(rPressedOrientDrive));
    drivers->commandMapper.addMap(std::move(bPressedNormDrive));

    drivers->commandMapper.addMap(std::move(rightSwitchDownBeyblade));
    drivers->commandMapper.addMap(std::move(leftSwitchDownPressedShoot));
    drivers->commandMapper.addMap(std::move(leftSwitchUpFlywheels));
    drivers->commandMapper.addMap(std::move(rightSwitchUpRunKicker));

    /// TRIGGERS
    /// Triggers don't need to be added to the command mapper since they register themselves
    /// with the command scheduler when they are constructed, but just listing them here for
    /// clarity
    /*
    ctrlZPressedImuCal
    imuCalWhenWheelRight
    ctrlCPressedUI
    */
}
}  // namespace hero_control

namespace src::hero
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return nullptr;  //&hero_control::imuCalibrateCommand; //TODO tune to make work
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