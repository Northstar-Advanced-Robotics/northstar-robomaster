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
 * A command that performs a "u-turn" operation of the turret. Commands
 * the turret relative to where it is facing to rotate some set amount.
 * Used to turn around easily without having to do so manually.
 *
 * @note This command runs **exactly once**. As such, it should be scheduled
 *      **exactly once**. The command does not run a PID controller and such,
 *      re-scheduling it over and over will result in unexpected behavior (the
 *      turret will appear to not do anything).
 */
class TurretTestCommand : public tap::control::Command
{
public:
    /**
     * @param[in] turretSubsystem Turret whose setpoint to update
     * @param[in] targetOffsetToTurn Offset angle, in radians, that the turret setpoint
     *      will be updated to when this command is run.
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
