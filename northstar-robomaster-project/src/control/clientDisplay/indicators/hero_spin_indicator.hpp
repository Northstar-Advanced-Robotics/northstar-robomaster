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

#ifndef HERO_SPIN_INDICATOR_HPP_
#define HERO_SPIN_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

using namespace tap::communication::serial;

namespace src::control::client_display
{
/**
 * @deprecated Not built into any robot. Superseded by `control/clientDisplay/graphics/`, which is
 * what every robot's control file actually instantiates. The only construction site for this class
 * is `robot/testbed/using_hud.hpp`, behind a `USING_HUD` switch that is commented out.
 */
/**
 * @ingroup client_display
 *
 * Draws an indicator that turns green while either the beyblade or the wiggle command is
 * scheduled, so the hero's operator can see the chassis is spinning.
 */
class HeroSpinIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * @param[in] refSerialTransmitter Sends the assembled graphics to the referee system.
     * @param[in] refSerial Referee system data. Stored but not read.
     * @param[in] drivers The global drivers object.
     * @param[in] beyblade Beyblade command, polled for whether it is scheduled.
     * @param[in] wiggle Wiggle command, polled for whether it is scheduled.
     */
    HeroSpinIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const tap::communication::serial::RefSerial &refSerial,
        tap::Drivers &drivers,
        chassis::ChassisBeybladeCommand *beyblade,
        chassis::ChassisWiggleCommand *wiggle);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 2 - 150;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 800;
    // WIDTH of the text
    static constexpr uint16_t WIDTH = 6;
    // SIZE of the text
    static constexpr uint16_t SIZE = 40;

    Tx::GraphicCharacterMessage textGraphic;
    const char *visisonTarget = "Spinning ";

    static constexpr uint16_t CIRCLE_X = TEXT_X + 175;

    static constexpr uint16_t CRICLE_Y = TEXT_Y;
    // SIZE of the circle
    static constexpr uint16_t CRICLE_SIZE = 2;
    // Thickness of the line
    static constexpr uint16_t LINE_THICKNESS = 5;

    const tap::communication::serial::RefSerial &refSerial;

    tap::Drivers &drivers;
    chassis::ChassisBeybladeCommand *beyblade;
    chassis::ChassisWiggleCommand *wiggle;

public:
    uint8_t circleName[3];

    Tx::GraphicColor circleColor;

    Tx::Graphic1Message circleGraphics;
};

}  // namespace src::control::client_display

#endif  // AMMO_INDICATOR_HPP_
