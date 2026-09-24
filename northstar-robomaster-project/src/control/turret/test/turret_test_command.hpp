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

#ifndef TURRET_TEST_COMMAND_HPP_
#define TURRET_TEST_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../turret_subsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace src::control::turret::test
{
/**
 * @ingroup turret
 *
 * Bench-test command: steps the turret by a fixed yaw and pitch offset and holds it there until it
 * arrives, for checking travel and controller tuning without a remote.
 *
 * Unlike `TurretQuickTurnCommand`, this **does** run the PID controllers every iteration and is
 * **not** one-shot -- it keeps driving until the yaw measurement is within `allowedError`.
 *
 * @warning Not built into any robot. Only `robot/testbed/using_turret.hpp` constructs it, behind
 *      `USING_TURRET`, which is currently commented out in `test_def.hpp`.
 */
class TurretTestCommand : public tap::control::Command
{
public:
    /**
     * @param[in] turretSubsystem The turret to move, taken as a subsystem requirement.
     * @param[in] yawMoveAmount How far to move yaw from its current measured angle, in radians.
     * @param[in] pitchMoveAmount How far to move pitch from its current measured angle, in radians.
     * @param[in] yawController The controller driving yaw to its setpoint.
     * @param[in] pitchController The controller driving pitch to its setpoint.
     * @param[in] allowedError How close yaw must get before the command finishes, in radians.
     *      Pitch is not checked.
     */
    TurretTestCommand(
        TurretSubsystem *turretSubsystem,
        float yawMoveAmount,
        float pitchMoveAmount,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        float allowedError = 0.5f);

    bool isReady() override;

    const char *getName() const override { return "turret test command"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    TurretSubsystem *turretSubsystem;
    float yawMoveAmount;
    float pitchMoveAmount;
    uint32_t startTime;
    uint32_t endTime;
    uint32_t prevTime;
    WrappedFloat startYawAngle = WrappedFloat(0, 0, M_TWOPI);
    WrappedFloat startPitchAngle = WrappedFloat(0, 0, M_TWOPI);
    WrappedFloat newYawSetpoint = WrappedFloat(0, 0, M_TWOPI);
    WrappedFloat newPitchSetpoint = WrappedFloat(0, 0, M_TWOPI);
    float allowedError;
    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;
};
}  // namespace src::control::turret::test

#endif  // TURRET_QUICK_TURN_COMMAND_HPP_
