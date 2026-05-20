#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/governor/flywheel_on_governor.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class FlywheelReadyIndicator : public GraphicsContainer
{
public:
    FlywheelReadyIndicator(
        tap::Drivers* drivers,
        control::governor::FlywheelOnGovernor* flywheelGovernor)
        : drivers(drivers),
          flywheelGovernor(flywheelGovernor)
    {
        addGraphicsObject(&readyCircle);
    }

    void update()
    {
        // if(drivers->remote.keyPressed(Remote::Key::R))
        //     drivers->recal.requestRecalibration();

        if (flywheelGovernor->isReady())
        {
            readyCircle.color = UISubsystem::Color::GREEN;
        }
        else
        {
            readyCircle.color = UISubsystem::Color::BLACK;
        }
    }

private:
    tap::Drivers* drivers;

    control::governor::FlywheelOnGovernor* flywheelGovernor;

    static constexpr uint16_t X_POSITION =
        200;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    UnfilledCircle readyCircle{UISubsystem::Color::BLACK, X_POSITION, Y_POSITION, 200, 10};
};

}  // namespace src::control::client_display::graphics