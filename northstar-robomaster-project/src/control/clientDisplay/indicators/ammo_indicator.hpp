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

#ifndef AMMO_INDICATOR_HPP_
#define AMMO_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

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
 * Draws an "AMMO: " label plus the projectile count the referee system reports.
 *
 * Reads `bulletsRemaining42` for the hero and `bulletsRemaining17` for every other robot. Drawn in
 * orange, not the yellow an earlier version of this comment claimed.
 */
class AmmoIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * @param[in] refSerialTransmitter Sends the assembled graphics to the referee system.
     * @param[in] refSerial Referee system data supplying the projectile count.
     */
    AmmoIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const tap::communication::serial::RefSerial &refSerial);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 2 - 150;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 200;
    // WIDTH of the text
    static constexpr uint16_t WIDTH = 4;
    // SIZE of the text
    static constexpr uint16_t SIZE = 40;

    Tx::GraphicCharacterMessage textGraphic;
    const char *bulletsRemainingText = "AMMO: ";

    Tx::Graphic1Message numberGraphic;
    tap::communication::referee::StateHUDIndicator<int32_t> numberIndicator;

    static constexpr uint16_t NUMBER_X = TEXT_X + 175;

    int bulletCount = 0;

    const tap::communication::serial::RefSerial &refSerial;

    static inline void updateAmmoCount(int32_t value, RefSerialData::Tx::Graphic1Message *graphic)
    {
        RefSerialTransmitter::configInteger(
            SIZE,
            WIDTH,
            NUMBER_X,
            TEXT_Y,
            value,
            &graphic->graphicData);
    }
};

}  // namespace src::control::client_display

#endif  // AMMO_INDICATOR_HPP_
