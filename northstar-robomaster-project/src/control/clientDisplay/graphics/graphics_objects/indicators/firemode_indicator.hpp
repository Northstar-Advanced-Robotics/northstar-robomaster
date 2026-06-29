#pragma once

#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/governor/flywheel_on_governor.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class FiremodeIndicator : public GraphicsContainer
{
public:
    FiremodeIndicator(
        tap::Drivers* drivers,
        control::agitator::MultiShotCvCommandMapping* shootCommand,
        control::governor::FlywheelOnGovernor* flywheelGovernor)
        : drivers(drivers),
          shootCommand(shootCommand),
          flywheelGovernor(flywheelGovernor)
    {
        addGraphicsObject(&firemode);
    }

    void update()
    {
        // if(drivers->remote.keyPressed(Remote::Key::R))
        //     drivers->recal.requestRecalibration();

        switch (shootCommand->getLaunchMode())
        {
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::SINGLE:
                firemode.setString("FireMode: Single");
                break;
#ifndef TARGET_HERO
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::NO_HEATING:
                firemode.setString("FireMode: No Heating");
                break;
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::LIMITED_10HZ:
                firemode.setString("FireMode: 10hz Limited");
                break;
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::LIMITED_20HZ:
                firemode.setString("FireMode: 20Hz Limited");
                break;
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::FULL_AUTO:
                firemode.setString("FireMode: FULL AUTO");
                break;
#endif
            case control::agitator::MultiShotCvCommandMapping::LaunchMode::BURST:
                firemode.setString("FireMode: BURST");
                break;
            default:
                firemode.setString("FireMode: ");
                break;
        }

        firemode.calculateNumbers();
        // firemode.x = X_POSITION - firemode.width / 2;

        if (flywheelGovernor->isReady())
        {
            firemode.color = UISubsystem::Color::GREEN;
        }
        else
        {
            firemode.color = UISubsystem::Color::PURPLISH_RED;
        }
    }

private:
    tap::Drivers* drivers;

    control::agitator::MultiShotCvCommandMapping* shootCommand;

    control::governor::FlywheelOnGovernor* flywheelGovernor;

    static constexpr uint16_t X_POSITION = 20;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    StringGraphic
        firemode{UISubsystem::Color::PURPLISH_RED, "FireMode: ", X_POSITION, Y_POSITION, 20, 3};
};

}  // namespace src::control::client_display::graphics