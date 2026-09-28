#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Fills the entire screen with a grid of circles.
 *
 * A stress test for the drawing pipeline: it produces far more graphics than any real HUD, which is
 * what exposes how the batching and layer handling behave under load. Not part of any robot's
 * display.
 */
class TestFill : public GraphicsContainer
{
public:
    TestFill()
    {
        // lets make each circle radius 30 (size 60), so 32 by 18 circles, 576 total

        for (int i = 0; i < UISubsystem::SCREEN_WIDTH; i += 2 * R)
        {
            for (int j = 0; j < UISubsystem::SCREEN_HEIGHT; j += 2 * R)
            {
                // using new is bad, this is just for testing
                addGraphicsObject(
                    new UnfilledCircle(UISubsystem::Color::ORANGE, i + R, j + R, R, 5));
            }
        }
    }

private:
    static constexpr int R = 60;
};

}  // namespace src::control::client_display::graphics