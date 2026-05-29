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

#include "turret_cv_control_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "robot/control_operator_interface.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::cv
{
TurretCVControlCommand::TurretCVControlCommand(
    tap::Drivers *drivers,
    ControlOperatorInterface &controlOperatorInterface,
    src::serial::VisionComms &visionComms,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      visionComms(visionComms),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    TurretCVControlCommandTemplate::addSubsystemRequirement(turretSubsystem);
}
bool TurretCVControlCommand::isReady() { return !isFinished(); }

void TurretCVControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    drivers->leds.set(tap::gpio::Leds::Green, true);
}
void TurretCVControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    if (visionComms.isAimDataUpdated(turretID))
    {
        // up has positive error so up positive
        const WrappedFloat pitchSetpoint = Angle(visionComms.getLastAimData(turretID).pitch);
        pitchController->runController(dt, pitchSetpoint);
        // left neg right post
        const WrappedFloat yawSetpoint = Angle(visionComms.getLastAimData(turretID).yaw);
        yawController->runController(dt, yawSetpoint);

        withinAimingTolerance =
            (abs(visionComms.getLastAimData(turretID).yaw -
                 yawController->getMeasurement().getUnwrappedValue()) <
                 visionComms.getLastAimData(turretID).maxErrorYaw &&
             abs(visionComms.getLastAimData(turretID).pitch -
                 pitchController->getMeasurement().getUnwrappedValue()) <
                 visionComms.getLastAimData(turretID).maxErrorPitch);
    }
    else
    {
        const WrappedFloat pitchSetpoint =
            pitchController->getSetpoint() +
            userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(turretID);
        pitchController->runController(dt, pitchSetpoint);

        const WrappedFloat yawSetpoint =
            yawController->getSetpoint() +
            userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID);
        yawController->runController(dt, yawSetpoint);

        withinAimingTolerance = false;
    }
}

bool TurretCVControlCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
}

void TurretCVControlCommand::end(bool interrupted)
{
    if (!interrupted)
    {
        turretSubsystem->yawMotor.setMotorOutput(0);
        turretSubsystem->pitchMotor.setMotorOutput(0);
    }
    drivers->leds.set(tap::gpio::Leds::Green, false);
}

}  // namespace src::control::turret::cv
