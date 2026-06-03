/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "chassis_power_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace src::control::client_display
{
ChassisPowerIndicator::ChassisPowerIndicator(
    RefSerialTransmitter &refSerialTransmitter,
    const RefSerial &refSerial,
    src::chassis::ChassisSubsystem &chassisSubsystem)
    : HudIndicator(refSerialTransmitter),
      numberIndicator(refSerialTransmitter, &numberGraphic, updateChassisPower, (int32_t)0),
      refSerial(refSerial),
      chassisSubsystem(chassisSubsystem)
{
}

modm::ResumableResult<void> ChassisPowerIndicator::update()
{
    // Access the correct field depending on the robot type

    chassisPower = chassisSubsystem.getChassisPowerDraw();

    numberIndicator.setIndicatorState(chassisPower);
    // if (chassisPower > chassisSubsystem.ChassisSubsystem::getChassisPowerLimit(drivers))
    // {
    // }

    RF_BEGIN(1);

    RF_CALL(numberIndicator.draw());

    RF_END();
}

modm::ResumableResult<void> ChassisPowerIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&textGraphic));

    RF_CALL(numberIndicator.initialize());

    RF_END();
}

void ChassisPowerIndicator::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &textGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    RefSerialTransmitter::configCharacterMsg(
        SIZE,
        WIDTH,
        TEXT_X,
        TEXT_Y,
        "CHASSIS POWER: ",
        &textGraphic);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    updateChassisPower(0, &numberGraphic);
}

}  // namespace src::control::client_display
