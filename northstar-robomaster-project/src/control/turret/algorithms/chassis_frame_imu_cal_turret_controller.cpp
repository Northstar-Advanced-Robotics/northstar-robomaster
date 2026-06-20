/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_frame_imu_cal_turret_controller.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::control::turret;
using tap::algorithms::WrappedFloat;

namespace src::control::turret::algorithms
{
ChassisFrameYawImuCalTurretController::ChassisFrameYawImuCalTurretController(
    TurretMotor &yawMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig,
    float errorForMaxOuput,
    float maxOutput,
    float errorForAveraging)
    : TurretYawControllerInterface(yawMotor),
      pid(pidConfig),
      errorForMaxOuput(errorForMaxOuput),
      maxOutput(maxOutput),
      errorForAveraging(errorForAveraging),
      positionBuffer(std::deque<float>())
{
}

void ChassisFrameYawImuCalTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}
void ChassisFrameYawImuCalTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError = turretMotor.getValidChassisMeasurementError();

    if (positionControllerError > errorForMaxOuput)
    {
        float pidOutput =
            pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);
        turretMotor.setMotorOutput(limitVal<float>(pidOutput, -maxOutput, maxOutput));
    }
    else if (positionControllerError < errorForAveraging)
    {
        if (positionBuffer.size() > 10)
        {
            positionBuffer.pop_front();
        }
        positionBuffer.push_back(positionControllerError);
        float pidOutput = pid.runController(
            getPositionBufferAverage(),
            turretMotor.getChassisFrameVelocity(),
            dt);
        turretMotor.setMotorOutput(pidOutput);
    }
    else
    {
        float pidOutput =
            pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);
        turretMotor.setMotorOutput(pidOutput);
    }
}

void ChassisFrameYawImuCalTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

WrappedFloat ChassisFrameYawImuCalTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

WrappedFloat ChassisFrameYawImuCalTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameMeasuredAngle();
}
bool ChassisFrameYawImuCalTurretController::isOnline() const { return turretMotor.isOnline(); }

ChassisFramePitchImuCalTurretController::ChassisFramePitchImuCalTurretController(
    TurretMotor &pitchMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig,
    float errorForMaxOuput,
    float maxOutput,
    float errorForAveraging)
    : TurretPitchControllerInterface(pitchMotor),
      pid(pidConfig),
      errorForMaxOuput(errorForMaxOuput),
      maxOutput(maxOutput),
      errorForAveraging(errorForAveraging),
      positionBuffer(std::deque<float>())
{
}

void ChassisFramePitchImuCalTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}

void ChassisFramePitchImuCalTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // position controller based on turret pitch gimbal
    float positionControllerError = turretMotor.getValidChassisMeasurementError();

    float pidOutput =
        pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);

    pidOutput += computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        -turretMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        GRAVITY_COMPENSATION_SCALAR);

    if (positionControllerError > errorForMaxOuput)
    {
        turretMotor.setMotorOutput(limitVal<float>(pidOutput, -maxOutput, maxOutput));
    }
    else if (positionControllerError < errorForAveraging)
    {
        if (positionBuffer.size() > 10)
        {
            positionBuffer.pop_front();
        }
        positionBuffer.push_back(positionControllerError);
        turretMotor.setMotorOutput(getPositionBufferAverage());
    }
    else
    {
        turretMotor.setMotorOutput(pidOutput);
    }
}

void ChassisFramePitchImuCalTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

WrappedFloat ChassisFramePitchImuCalTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

WrappedFloat ChassisFramePitchImuCalTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameMeasuredAngle();
}

bool ChassisFramePitchImuCalTurretController::isOnline() const { return turretMotor.isOnline(); }

}  // namespace src::control::turret::algorithms
