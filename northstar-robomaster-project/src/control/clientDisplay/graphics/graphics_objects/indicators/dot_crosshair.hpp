#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class DotCrosshair : public GraphicsContainer
{
public:
    DotCrosshair(tap::Drivers* drivers) : drivers(drivers) { addGraphicsObject(&crosshair); }

private:
    tap::Drivers* drivers;

    static constexpr uint16_t X_POSITION = UISubsystem::HALF_SCREEN_WIDTH;
    static constexpr uint16_t Y_POSITION = UISubsystem::HALF_SCREEN_HEIGHT - 100;
    static constexpr uint16_t RADIUS = 20;
    static constexpr uint16_t THICKNESS = 5;

    UnfilledCircle crosshair{UISubsystem::Color::CYAN, X_POSITION, Y_POSITION, RADIUS, THICKNESS};
};

}  // namespace src::control::client_display::graphics