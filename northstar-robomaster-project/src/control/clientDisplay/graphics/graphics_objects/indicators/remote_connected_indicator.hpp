#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class RemoteConnectedIndicator : public GraphicsContainer
{
public:
    RemoteConnectedIndicator(tap::Drivers* drivers) : drivers(drivers), remote(&drivers->remote)
    {
        addGraphicsObject(&status);
    }

    void update()
    {
        if (cycles > 500)
        {
            hidden = !hidden;
            status.setHidden(hidden);
        }
        else
        {
            cycles++;
        }

        if (remote->isConnected())
        {
            status.color = UISubsystem::Color::GREEN;
            status.setString("Remote: Connected if flash");
        }
        else
        {
            status.color = UISubsystem::Color::PURPLISH_RED;
            status.setString("Remote: Disconnected");
        }

        status.calculateNumbers();
    }

private:
    tap::Drivers* drivers;

    Remote* remote;

    static constexpr uint16_t STATUS_X_POSITION =
        20;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t STATUS_Y_POSITION = 600;  // pixels, all numbers at the same y level
                                                        // on scre

    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    StringGraphic status{
        UISubsystem::Color::PURPLISH_RED,
        "Remote: ",
        STATUS_X_POSITION,
        STATUS_Y_POSITION,
        20,
        3};

    int cycles = 0;
    bool hidden = false;
};

}  // namespace src::control::client_display::graphics