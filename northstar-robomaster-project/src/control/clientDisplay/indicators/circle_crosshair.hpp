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

#ifndef CIRCLE_CROSSHAIR_HPP_
#define CIRCLE_CROSSHAIR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

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
 * Draws a circular crosshair at a fixed screen position.
 */
class CircleCrosshair : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Makes a dot circle crosshair on the screen.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    CircleCrosshair(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

private:
    // Offset from the center of the screen for the crosshair
#ifdef TARGET_STANDARD_NULL
    static constexpr int16_t OFFSET_X = 25;
    static constexpr int16_t OFFSET_Y = -75;
#else
    static constexpr int16_t OFFSET_X = 0;
    static constexpr int16_t OFFSET_Y = 0;
#endif

    // X position of the circle
    static constexpr uint16_t CRICLE_X = SCREEN_WIDTH / 2 + OFFSET_X;
    // Y position of the circle
    static constexpr uint16_t CRICLE_Y = 450;
    // SIZE of the circle
    static constexpr uint16_t CRICLE_SIZE = 5;
    // Thickness of the line
    static constexpr uint16_t LINE_THICKNESS = 5;

    Tx::Graphic1Message crosshairGraphics;
};

}  // namespace src::control::client_display

#endif  // CIRCLE_CROSSHAIR_HPP_
