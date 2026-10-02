/**
 * Declares the turret: the subsystem, its controllers, and the operator and CV control commands.
 * Built the same way as the standard's turret in `standard_control.cpp`.
 *
 * `testbed_control.cpp` includes this file unconditionally; the `#ifdef USING_TURRET` below is what
 * actually selects it, and that switch is set in `test_def.hpp`.
 *
 * Everything is declared at file scope, which is why exactly one translation unit may include it.
 */
#ifdef USING_TURRET

#ifndef USING_TURRET_HPP_
#define USING_TURRET_HPP_

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::turret;

// turret subsystem
tap::motor::DjiMotor turretPitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH,
    false,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor turretYawMotor(
    drivers(),
    YAW_MOTOR_ID_1,
    CAN_BUS_YAW,
    false,
    "YawMotor1",
    false,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue,
    &drivers()->encoder);

TurretSubsystem turretSubsystem(
    drivers(),
    &turretPitchMotor,
    &turretYawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &turretYawMotor.getInternalEncoder());

// turret controlers
ChassisFramePitchTurretController chassisFramePitchTurretController(
    turretSubsystem.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

ChassisFrameYawTurretController chassisFrameYawTurretController(
    turretSubsystem.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turretSubsystem.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    turretSubsystem.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu fixed on turret
WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    *drivers(),
    turretSubsystem.pitchMotor,
    worldFramePitchTurretPosPid,
    worldFramePitchTurretVelPid);

WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    *drivers(),
    turretSubsystem.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// turret commands
TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

Trigger xPressedCvControl =
    TriggerHelpers::button(drivers(), tap::communication::serial::Remote::Key::X)
        .whileTrue(&turretCVControlCommand);

#endif  // USING_TURRET_HPP_

#endif  // USING_TURRET
