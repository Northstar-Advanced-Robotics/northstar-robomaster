#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Shows the referee system's pre-match countdown as a large number off to the side of the screen.
 *
 * The client's own countdown is hidden behind the purchase menu, which is exactly where the
 * operator is looking as the match starts, so this copy is drawn somewhere still visible.
 */
class Countdown : public GraphicsContainer
{
public:
    Countdown(tap::Drivers* drivers) : drivers(drivers)
    {
        addGraphicsObject(&number);
        number.x = X_POSITION;
        number.y = Y_POSITION;
        number.height = LINE_HEIGHT;
    }

    /// Reads the remaining stage time from the referee system and recenters the number, coloring it
    /// by game stage and hiding it once the match is underway.
    void update()
    {
        // if(drivers->remote.keyPressed(Remote::Key::R))
        //     drivers->recal.requestRecalibration();

        if (drivers->refSerial.getRefSerialReceivingData())
        {
            RefSerialData::Rx::GameData gameData = drivers->refSerial.getGameData();
            number.integer = gameData.stageTimeRemaining;
            number.calculateNumbers();
            number.x = X_POSITION - number.width / 2;

            if (gameData.gameStage == RefSerialData::Rx::GameStage::INITIALIZATION)
            {
                number.color = UISubsystem::Color::ORANGE;
                number.show();
            }
            else if (gameData.gameStage == RefSerialData::Rx::GameStage::COUNTDOWN)
            {
                number.color = UISubsystem::Color::CYAN;
                number.show();
            }
            else
            {
                number.hide();
            }
        }
    }

private:
    tap::Drivers* drivers;

    static constexpr uint16_t X_POSITION =
        1680;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    IntegerGraphic number{};
};

}  // namespace src::control::client_display::graphics