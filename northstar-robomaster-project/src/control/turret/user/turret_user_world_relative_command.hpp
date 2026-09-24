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

#ifndef TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
#define TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "../algorithms/turret_controller_interface.hpp"

#include "turret_user_control_command.hpp"

namespace src
{
class Drivers;
}

namespace src::control
{
class ControlOperatorInterface;
}

namespace src::control::turret
{
class TurretSubsystem;
}

namespace src::control::turret::user
{
/**
 * @ingroup turret
 *
 * Operator turret control in the world frame, so the aim the operator asks for is independent of
 * how the chassis is facing or rotating.
 *
 * A comprised command holding two inner commands, one per available IMU source, and switching
 * between them at runtime as their controllers report online. The robot runs on a RoboMaster
 * type C board with a BMI088 mounted to the chassis.
 *
 * @warning The two paths are less distinct than the parameter names suggest. Every controller the
 *      robots actually pass in reads the same chassis-mounted BMI088 -- the genuine turret-MCB
 *      controllers are never instantiated -- so in practice both paths use the same sensor.
 *
 * Takes in user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret.
 */
class TurretUserWorldRelativeCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     *
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] controlOperatorInterface Source of the operator's turret input.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] chassisImuYawController World frame turret controller that uses the chassis IMU.
     * @param[in] chassisImuPitchController Turret controller that is used when the chassis IMU is
     * in use.
     * @param[in] turretImuYawController World frame turret controller that uses the turret IMU.
     * @param[in] turretImuPitchController Turret controller intended for the turret-IMU path.
     * Doesn't strictly have to be world relative.
     * @param[in] userYawInputScalar Value to scale the operator's yaw input by; effectively mouse
     * sensitivity.
     * @param[in] userPitchInputScalar See `userYawInputScalar`.
     * @param[in] turretID Which turret this command drives, for robots with more than one.
     *
     * @warning `turretImuPitchController` is currently **ignored**: the constructor passes
     * `chassisImuPitchController` into the turret-IMU sub-command instead, and this argument is
     * not stored anywhere. See the `// TODO for actual use change back to pitch` note at the call
     * site in the .cpp.
     */
    TurretUserWorldRelativeCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *chassisImuYawController,
        algorithms::TurretPitchControllerInterface *chassisImuPitchController,
        algorithms::TurretYawControllerInterface *turretImuYawController,
        algorithms::TurretPitchControllerInterface *turretImuPitchController,
        float userYawInputScalar,
        float userPitchInputScalar,
        uint8_t turretID = 0);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    const char *getName() const override { return "turret WR"; }

private:
    TurretUserControlCommand turretWRChassisImuCommand;
    TurretUserControlCommand turretWRTurretImuCommand;
};  // class TurretUserWorldRelativeCommand

}  // namespace src::control::turret::user

#endif  // TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
