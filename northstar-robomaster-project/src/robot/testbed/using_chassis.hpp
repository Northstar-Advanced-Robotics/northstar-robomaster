#ifdef USING_TURRET

#ifndef USING_CHASSIS_HPP_
#define USING_CHASSIS_HPP_

#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "robot/testbed/test_def.hpp"

using namespace src::control::governor;
using namespace tap::control;
using namespace src::control::turret;
using namespace src::control::chassis;


extern src::control::turret::TurretSubsystem turretSubsystem;

src::control::turret::TurretMotor* yawMotor = &turretSubsystem.yawMotor;

#else

/// TODO UPDATE THIS FOR NEW CODE
// tap::motor::RevMotor yawMotor1(
//     drivers(),
//     YAW_MOTOR_ID_1,
//     CAN_BUS_MOTORS,
//     tap::motor::RevMotor::ControlMode::DUTY_CYCLE,  // Change from duty cycle
//     true,
//     "YawMotor1",
//     1,
//     YAW_MOTOR_CONFIG.startEncoderValue,
//     &drivers()->encoder);

// tap::motor::RevMotor yawMotor2(
//     drivers(),
//     YAW_MOTOR_ID_2,
//     CAN_BUS_MOTORS,
//     tap::motor::RevMotor::ControlMode::DUTY_CYCLE,
//     false,
//     "YawMotor2",
//     1,
//     YAW_MOTOR_CONFIG.startEncoderValue);

// src::control::turret::TurretDoubleMotorRev yawTurretMotor(&yawMotor1, &yawMotor2,
// YAW_MOTOR_CONFIG);

// src::control::turret::TurretMotor* yawMotor = &yawTurretMotor;

#endif

#ifdef USING_CHASSIS

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// GovernorWithFallbackCommand<2> beyBladeSlowOutOfCombat(
//     {&chassisSubsystem},
//     chassisBeyBladeSlowCommand,
//     chassisBeyBladeFastCommand,
//     {&firedRecentlyGovernor, &plateHitGovernor},
//     true);

// chassis Mappings
// ToggleCommandMapping bPressed(
//     drivers(),
//     {&beyBladeSlowOutOfCombat},
//     RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

// imu::ImuCalibrateCommand imuCalibrateCommand(
//     drivers(),
//     {{
//         &turret,
//         &chassisFrameYawTurretController,
//         &chassisFramePitchTurretController,
//         true,
//     }},
//     &chassisSubsystem);

src::control::chassis::ChassisOdometry* chassisOdometry = new src::control::chassis::ChassisOdometry(
    &drivers()->bmi088,
    yawMotor,
    src::control::chassis::DIST_TO_CENTER,
    src::control::chassis::WHEEL_DIAMETER_M);

ChassisSubsystem chassisSubsystem(
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
            src::control::chassis::VELOCITY_PID_MAX_ERROR_SUM),
    },
    &drivers()->turretMCBCanCommBus2,
    yawMotor,
    chassisOdometry);

ChassisDriveCommand chassisDriveCommand(&chassisSubsystem, &drivers()->controlOperatorInterface);

ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

ChassisBeybladeCommand chassisBeyBladeCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    true);

// chassis Mappings
Trigger bPressedBeyblade =
    TriggerHelpers::button(drivers(), tap::communication::serial::Remote::Key::B)
        .toggleOnTrue(&chassisBeyBladeCommand);

#endif

#endif  // USING_CHASSIS_HPP_