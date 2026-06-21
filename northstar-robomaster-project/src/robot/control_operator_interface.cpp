/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "robot/control_operator_interface.hpp"

#include <random>

#include <tap/architecture/clock.hpp>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;
using namespace src::chassis;

namespace src
{
namespace control
{
float ControlOperatorInterface::getTurretYawInput()
{
    float input;

    input = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * REMOTE_TURRET_SCALAR +
            static_cast<float>(limitVal<int16_t>(
                drivers->remote.getMouseX(),
                -USER_MOUSE_YAW_MAX,
                USER_MOUSE_YAW_MAX)) *
                USER_MOUSE_YAW_SCALAR;
    if (!compareFloatClose(input, 0, .01))
    {
        return input;
    }
    return 0;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    float input;

    input = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * REMOTE_TURRET_SCALAR +
            static_cast<float>(limitVal<int16_t>(
                drivers->remote.getMouseY(),
                -USER_MOUSE_PITCH_MAX,
                USER_MOUSE_PITCH_MAX)) *
                USER_MOUSE_PITCH_SCALAR;
    if (!compareFloatClose(input, 0, .01))
    {
        return input;
    }
    return 0;
}

// Returns a MPS scaled by max chassis speed per the power limit
float ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    float maxWheelSpeedMPS = ChassisSubsystem::getMaxWheelSpeed(
                                 drivers->refSerial.getRefSerialReceivingData(),
                                 ChassisSubsystem::getChassisPowerLimit(drivers)) *
                             (WHEEL_DIAMETER_M * M_PI / 60.0f * CHASSIS_GEAR_RATIO) * 1.4142f;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) * maxWheelSpeedMPS,
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    float output = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::A) &&
        !drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output = -CHASSIS_WALK_SPEED_MPS;
    }
    else if (
        drivers->remote.keyPressed(Remote::Key::A) &&
        drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output = -maxWheelSpeedMPS;
    }
    if (drivers->remote.keyPressed(Remote::Key::D) &&
        !drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output += CHASSIS_WALK_SPEED_MPS;
    }
    else if (
        drivers->remote.keyPressed(Remote::Key::D) &&
        drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output += maxWheelSpeedMPS;
    }

    output = limitVal<float>(
        chassisYInput.getInterpolatedValue(currTime) + output,
        -maxWheelSpeedMPS,
        maxWheelSpeedMPS);

    return output;
}

// Returns a MPS scaled by max chassis speed per the power limit
float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    float maxWheelSpeedMPS = ChassisSubsystem::getMaxWheelSpeed(
                                 drivers->refSerial.getRefSerialReceivingData(),
                                 ChassisSubsystem::getChassisPowerLimit(drivers)) *
                             (WHEEL_DIAMETER_M * M_PI / 60.0f * CHASSIS_GEAR_RATIO) * 1.4142f;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(
            drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * maxWheelSpeedMPS,
            currTime);
        prevUpdateCounterX = updateCounter;
    }

    float output = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::S) &&
        !drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output = -CHASSIS_WALK_SPEED_MPS;
    }
    else if (
        drivers->remote.keyPressed(Remote::Key::S) &&
        drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output = -maxWheelSpeedMPS;
    }
    if (drivers->remote.keyPressed(Remote::Key::W) &&
        !drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output += CHASSIS_WALK_SPEED_MPS;
    }
    else if (
        drivers->remote.keyPressed(Remote::Key::W) &&
        drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        output += maxWheelSpeedMPS;
    }

    output = limitVal<float>(
        chassisXInput.getInterpolatedValue(currTime) + output,
        -maxWheelSpeedMPS,
        maxWheelSpeedMPS);

    return output;
}

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    // return drivers->remote.getChannel(Remote::Channel::WHEEL) * 0.6;
    // if (drivers->remote.keyPressed(Remote::Key::Q) &&
    // !drivers->remote.keyPressed(Remote::Key::SHIFT))
    // {
    //     return -0.3f;
    // }
    // else if (drivers->remote.keyPressed(Remote::Key::Q) &&
    // drivers->remote.keyPressed(Remote::Key::SHIFT))
    // {
    //     return -0.6f;
    // }
    // else if (drivers->remote.keyPressed(Remote::Key::E) &&
    // !drivers->remote.keyPressed(Remote::Key::SHIFT))
    // {
    //     return 0.3f;
    // }
    // else if (drivers->remote.keyPressed(Remote::Key::E) &&
    // drivers->remote.keyPressed(Remote::Key::SHIFT))
    // {
    //     return 0.6f;
    // }
    // else
    // {
    //     return 0.0f;
    // }
    return 0;
}

}  // namespace control

}  // namespace src
