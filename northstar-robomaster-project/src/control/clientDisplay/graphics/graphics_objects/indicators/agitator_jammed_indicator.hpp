#pragma once

#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class AgitatorJammedIndicator : public GraphicsContainer
{
public:
    AgitatorJammedIndicator(
        tap::Drivers* drivers,
        src::agitator::VelocityAgitatorSubsystem* agitator)
        : drivers(drivers),
          agitator(agitator)
    {
        addGraphicsObject(&jammed);
    }

    void update()
    {
        if (agitator->isJammed())
        {
            jammed.show();
        }
        else
        {
            jammed.hide();
        }

        jammed.calculateNumbers();
    }

private:
    tap::Drivers* drivers;

    src::agitator::VelocityAgitatorSubsystem* agitator;

    static constexpr uint16_t JAMMED_X_POSITION =
        UISubsystem::HALF_SCREEN_WIDTH;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t JAMMED_Y_POSITION =
        UISubsystem::HALF_SCREEN_HEIGHT + 160;  // pixels, all numbers at the same y level

    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    StringGraphic jammed{
        UISubsystem::Color::PURPLISH_RED,
        "JAMMED",
        JAMMED_X_POSITION,
        JAMMED_Y_POSITION,
        20,
        3};
};

}  // namespace src::control::client_display::graphics