#ifndef USING_TURRET_HPP_
#define USING_TURRET_HPP_

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::turret;
using namespace tap::motor;

#if defined(USING_TURRET) && !defined(USING_REV)

// turret subsystem
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

src::control::turret::TurretMotorDJI pitchTurretMotor(&pitchMotor, PITCH_MOTOR_CONFIG);

tap::motor::RevMotor yawMotor1(
    drivers(),
    YAW_MOTOR_ID_1,
    CAN_BUS_MOTORS,
    tap::motor::RevMotor::ControlMode::DUTY_CYCLE,  // Change from duty cycle
    true,
    "YawMotor1",
    1,
    YAW_MOTOR_CONFIG.startEncoderValue,
    &drivers()->encoder);

tap::motor::RevMotor yawMotor2(
    drivers(),
    YAW_MOTOR_ID_2,
    CAN_BUS_MOTORS,
    tap::motor::RevMotor::ControlMode::DUTY_CYCLE,
    false,
    "YawMotor2",
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

src::control::turret::TurretDoubleMotorRev yawTurretMotor(&yawMotor1, &yawMotor2, YAW_MOTOR_CONFIG);

TurretSubsystem turretSubsystem(drivers(), pitchTurretMotor, yawTurretMotor);

// turret controlers
src::control::turret::algorithms::
    ChassisFramePitchTurretController chassisFramePitchTurretController(
        turretSubsystem.pitchMotor,
        chassis_rel::PITCH_PID_CONFIG);

src::control::turret::algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turretSubsystem.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

src::control::turret::algorithms::
    WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
        *drivers(),
        turretSubsystem.yawMotor,
        world_rel_chassis_imu::YAW_PID_CONFIG);

src::control::turret::algorithms::
    WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
        *drivers(),
        turretSubsystem.pitchMotor,
        world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu fixed on turret
src::control::turret::algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
        *drivers(),
        turretSubsystem.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

src::control::turret::algorithms::
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
        *drivers(),
        turretSubsystem.yawMotor,
        worldFrameYawTurretPosPid,
        worldFrameYawTurretVelPid);

// turret commands
user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

test::TurretTestCommand turretTestCommand(
    &turretSubsystem,
    modm::toRadian(90),
    modm::toRadian(0),
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    modm::toRadian(0.5));

// user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
//     drivers(),
//     drivers()->controlOperatorInterface,
//     &turretSubsystem,
//     &worldFrameYawChassisImuController,
//     &worldFramePitchChassisImuController,
//     &worldFrameYawTurretCanImuController,
//     &worldFramePitchTurretCanImuController,
//     USER_YAW_INPUT_SCALAR,
//     USER_PITCH_INPUT_SCALAR);

HoldCommandMapping xPressed(
    drivers(),
    {&turretCVControlCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::X})));

PressCommandMapping turretTestCommandMapping(
    drivers(),
    {&turretTestCommand},
    RemoteMapState(RemoteMapState(
        {tap::communication::serial::Remote::Switch::LEFT_SWITCH,
         tap::communication::serial::Remote::SwitchState::DOWN})));

#endif

#if defined(USING_TURRET) && defined(USING_REV)

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::RevMotor yawMotor1(
    drivers(),
    REV_MOTOR1,
    CanBus::CAN_BUS1,
    RevMotor::ControlMode::VOLTAGE,
    false,
    "YawMotor1",
    18.0f / 120.0f);  // gear ratio

tap::motor::RevMotor yawMotor2(
    drivers(),
    tap::motor::REV_MOTOR2,
    CanBus::CAN_BUS1,
    RevMotor::ControlMode::VOLTAGE,
    false,
    "YawMotor2",
    18.0f / 120.0f);  // gear ratio

RevTurretSubsystem revTurret(
    drivers(),
    &pitchMotor,
    &yawMotor1,
    &yawMotor2,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_REV_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    revTurret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    revTurret.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

user::NeoTurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &revTurret,
    &worldFrameYawChassisImuController,
    &worldFramePitchChassisImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

#endif

#endif  // USING_TURRET_HPP_