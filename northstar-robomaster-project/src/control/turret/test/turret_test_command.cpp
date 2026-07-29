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

#include "turret_test_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::test
{
TurretTestCommand::TurretTestCommand(
    TurretSubsystem *turretSubsystem,
    float yawMoveAmount,
    float pitchMoveAmount,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float allowedError)
    : turretSubsystem(turretSubsystem),
      yawMoveAmount(yawMoveAmount),
      pitchMoveAmount(pitchMoveAmount),
      allowedError(allowedError),
      yawController(yawController),
      pitchController(pitchController)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretTestCommand::isReady() { return turretSubsystem->yawMotor.isOnline(); }

float totalTime = 0;

void TurretTestCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();

    startYawAngle = yawController->getSetpoint();

    startPitchAngle = pitchController->getSetpoint();

    newYawSetpoint = startYawAngle + yawMoveAmount;

    yawController->setSetpoint(newYawSetpoint);

    newPitchSetpoint = startPitchAngle + pitchMoveAmount;

    pitchController->setSetpoint(newPitchSetpoint);

    startTime = tap::arch::clock::getTimeMilliseconds();

    prevTime = startTime;
}

float yawAngle;
float pitchAngle;

void TurretTestCommand::execute()
{
    yawAngle =
        abs(turretSubsystem->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startYawAngle.getWrappedValue());
    pitchAngle =
        abs(turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startPitchAngle.getWrappedValue());
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    yawController->runController(dt, newYawSetpoint);
    pitchController->runController(dt, newPitchSetpoint);
}

bool TurretTestCommand::isFinished() const
{
    WrappedFloat yawSetpoint = newYawSetpoint;
    bool yawMovementDone =
        abs(yawSetpoint.minDifference(turretSubsystem->yawMotor.getChassisFrameMeasuredAngle())) <=
        allowedError;

    // Pitch is intentionally not part of the finished check right now; restore this
    // if the test should wait on pitch as well.
    // WrappedFloat pitchSetpoint = pitchController->getSetpoint();
    // bool pitchMovementDone =
    //     abs(pitchSetpoint.minDifference(
    //         turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle())) <= allowedError;

    return yawMovementDone;
}

void TurretTestCommand::end(bool)
{
    endTime = tap::arch::clock::getTimeMilliseconds();
    totalTime = (endTime - startTime) / 1000.0f;
}

}  // namespace src::control::turret::test
