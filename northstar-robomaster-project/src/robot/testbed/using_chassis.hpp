#ifndef USING_CHASSIS_HPP_
#define USING_CHASSIS_HPP_

#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "robot/testbed/test_def.hpp"

using namespace src::control::governor;
using namespace tap::control;
using namespace src::control::turret;
using namespace src::chassis;

#ifdef USING_TURRET

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

src::chassis::ChassisOdometry* chassisOdometry = new src::chassis::ChassisOdometry(
    &drivers()->bmi088,
    yawMotor,
    src::chassis::DIST_TO_CENTER,
    src::chassis::WHEEL_DIAMETER_M);

ChassisSubsystem chassisSubsystem(
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
ToggleCommandMapping bPressed(
    drivers(),
    {&chassisBeyBladeCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

#endif

#endif  // USING_CHASSIS_HPP_