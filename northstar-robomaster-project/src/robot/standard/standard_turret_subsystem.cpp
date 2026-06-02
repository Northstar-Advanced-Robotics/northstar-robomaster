/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "standard_turret_subsystem.hpp"

namespace src::control::turret
{
StandardTurretSubsystem::StandardTurretSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* pitchMotor,
    tap::motor::MotorInterface* yawMotor,
    const TurretMotorConfig& pitchMotorConfig,
    const TurretMotorConfig& yawMotorConfig)
    : RobotTurretSubsystem(drivers, pitchMotor, yawMotor, pitchMotorConfig, yawMotorConfig),
      imu(&drivers->bmi088)
{
}

float StandardTurretSubsystem::getWorldYaw() const { return imu->getYaw(); }

float StandardTurretSubsystem::getWorldPitch() const { return imu->getPitch(); }

uint32_t StandardTurretSubsystem::getLastMeasurementTimeMicros() const
{
    return tap::arch::clock::getTimeMicroseconds();
}

}  // namespace src::control::turret
