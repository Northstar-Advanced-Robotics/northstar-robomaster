#pragma once

#include "communication/serial/vision_comms.hpp"
#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/governor/cv_on_target_governor.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class CVIndicator : public GraphicsContainer
{
public:
    CVIndicator(
        tap::Drivers* drivers,
        src::serial::VisionComms* visionComms,
        src::control::governor::CvOnTargetGovernor* cvOnTargetGovernor)
        : drivers(drivers),
          visionComms(visionComms),
          cvOnTargetGovernor(cvOnTargetGovernor)
    {
        addGraphicsObject(&status);
        addGraphicsObject(&onTarget);
    }

    void update()
    {
        if (visionComms->isCvOnline())
        {
            status.color = UISubsystem::Color::GREEN;
        }
        else
        {
            status.color = UISubsystem::Color::PURPLISH_RED;
        }

        if (visionComms->getSomeTurretHasTarget())
        {
            status.setString("CV: Target");
        }
        else
        {
            status.setString("CV: No Target");
        }

        if (cvOnTargetGovernor->isReady())
        {
            onTarget.show();
        }
        else
        {
            onTarget.hide();
        }

        status.calculateNumbers();
        onTarget.calculateNumbers();
    }

private:
    tap::Drivers* drivers;

    src::serial::VisionComms* visionComms;

    src::control::governor::CvOnTargetGovernor* cvOnTargetGovernor;

    static constexpr uint16_t STATUS_X_POSITION =
        20;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t STATUS_Y_POSITION = 750;  // pixels, all numbers at the same y level
                                                        // on scre
    static constexpr uint16_t ON_TARGET_X_POSITION = 20;

    static constexpr uint16_t ON_TARGET_Y_POSITION = 700;

    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    StringGraphic status{
        UISubsystem::Color::PURPLISH_RED,
        "CV: ",
        STATUS_X_POSITION,
        STATUS_Y_POSITION,
        20,
        3};

    StringGraphic onTarget{
        UISubsystem::Color::GREEN,
        "SHOOT",
        ON_TARGET_X_POSITION,
        ON_TARGET_Y_POSITION,
        20,
        3};
};

}  // namespace src::control::client_display::graphics