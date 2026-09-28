#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A small circle near the center of the screen, drawn as a fixed aiming reference.
 *
 * Static: it has no `update` because nothing about it depends on robot state.
 */
class DotCrosshair : public GraphicsContainer
{
public:
    DotCrosshair(tap::Drivers* drivers) : drivers(drivers) { addGraphicsObject(&crosshair); }

private:
    tap::Drivers* drivers;

    static constexpr uint16_t X_POSITION = UISubsystem::HALF_SCREEN_WIDTH - 75;
    static constexpr uint16_t Y_POSITION = UISubsystem::HALF_SCREEN_HEIGHT - 75;
    static constexpr uint16_t RADIUS = 5;
    static constexpr uint16_t THICKNESS = 2;

    UnfilledCircle crosshair{UISubsystem::Color::CYAN, X_POSITION, Y_POSITION, RADIUS, THICKNESS};
};

}  // namespace src::control::client_display::graphics