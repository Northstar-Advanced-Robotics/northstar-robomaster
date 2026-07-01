#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
// #include "subsystems/chassis/chassisSubsystem.hpp"
// #include "subsystems/turret/turretSubsystem.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::client_display::graphics
{
// looks like
//    __
//   /
//
//      __/
// at the center of the screen, the arcs represent the left and right inner
// panels if there are 2 arcs, if there are 4 then they are all four panels
class ChassisOrientationIndicator : public GraphicsContainer
{
public:
    ChassisOrientationIndicator(
        bool showPlsSpin,
        tap::Drivers* drivers,
        src::control::turret::TurretSubsystem* turret,
        src::chassis::ChassisSubsystem* chassis)
        : showPlsSpin(showPlsSpin),
          drivers(drivers),
          turret(turret),
          chassis(chassis)
    {
        addGraphicsObject(&front);
        addGraphicsObject(&side);

        if (showPlsSpin)
        {
            // addGraphicsObject(&plsSpinBox);
            addGraphicsObject(&plsSpinText);
        }
    }

    void update()
    {
        uint16_t heading = static_cast<uint16_t>(
            turret->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() * 180 / PI +
            YAW_OFFSET);
        // if the turret compared to the chassis (from the encoder) is facing forward, heading
        // would be 0, if facing right, heading would be 90

        // front arc is convex
        front.startAngle = heading - INNER_ARC_LEN / 2;
        UISubsystem::fixAngle(&front.startAngle);
        front.endAngle = front.startAngle + INNER_ARC_LEN;

        side.setHidden(!chassis->isPeeking);
#if defined(TARGET_STANDARD)
        // side arc is concave (because flyswatters), so angle is flipped
        side.startAngle = (chassis->isPeekingLeft ? 90 : 270) + heading - INNER_ARC_LEN / 2;
        UISubsystem::fixAngle(&side.startAngle);
        side.endAngle = side.startAngle + INNER_ARC_LEN;

        // and xy location isn't the center
        float angleRadians = (chassis->isPeekingLeft ? PI / 2 : 3 * PI / 2) +
                             turret->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue();
        side.cx = front.cx - 2 * side.width * sin(angleRadians);
        side.cy = front.cy - 2 * side.width * cos(angleRadians);
#else
        // side arc is like the front one, convex
        side.startAngle = (chassis->isPeekingLeft ? 270 : 90) + heading - INNER_ARC_LEN / 2;
        UISubsystem::fixAngle(&side.startAngle);
        side.endAngle = side.startAngle + INNER_ARC_LEN;
#endif

        // set side color to pink if on red team, cyan if on blue team
        if (drivers->refSerial.getRefSerialReceivingData())
        {
            side.color = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId)
                             ? UISubsystem::Color::CYAN
                             : UISubsystem::Color::PINK;
        }

        if (showPlsSpin)
        {
            plsSpinText.setHidden(chassis->beyBladeCommandRunning);
            // plsSpinBox.setHidden(!chassis->isPeeking && !chassis->isBeybladingOnly);

            plsSpinText.x = UISubsystem::HALF_SCREEN_WIDTH - plsSpinText.width / 2;
            // plsSpinBox.x1 = plsSpinText.x-TEXT_THICKNESS;
            // plsSpinBox.x2 = plsSpinText.x + plsSpinText.width+TEXT_THICKNESS*2;
        }
    }

    static constexpr float YAW_OFFSET =
        2 * 360;  // degrees, 0 from the yaw might not be top on the screen, also needs to make
                  // sure it is positive because we are using uints

private:
    bool showPlsSpin;
    tap::Drivers* drivers;
    src::control::turret::TurretSubsystem* turret;
    src::chassis::ChassisSubsystem* chassis;

    static constexpr uint16_t THICKNESS = 2;  // pixels
    static constexpr uint16_t INNER_SIZE =
        120;  // Used if the arcs are supposed to be inside the barrel heat circle, pixels
    static constexpr uint16_t INNER_ARC_LEN =
        40;  // Used if the arcs are supposed to be inside the barrel heat circle, degrees
    static constexpr uint16_t OUTER_SIZE =
        180;  // Used if the arcs are supposed to be outside the barrel heat circle, pixels
    static constexpr uint16_t OUTER_ARC_LEN =
        30;  // Used if the arcs are supposed to be outside the barrel heat circle, degrees

    Arc front{
        UISubsystem::Color::RED_AND_BLUE,
        UISubsystem::HALF_SCREEN_WIDTH,
        UISubsystem::HALF_SCREEN_HEIGHT,
        INNER_SIZE,
        INNER_SIZE,
        0,
        90,
        THICKNESS};
    Arc side{
        UISubsystem::Color::RED_AND_BLUE,
        UISubsystem::HALF_SCREEN_WIDTH,
        UISubsystem::HALF_SCREEN_HEIGHT,
        INNER_SIZE,
        INNER_SIZE,
        0,
        90,
        THICKNESS};

    static constexpr uint16_t TEXT_HEIGHT = 60;
    static constexpr uint16_t TEXT_Y = 830;
    static constexpr uint16_t TEXT_THICKNESS = 5;
    StringGraphic
        plsSpinText{UISubsystem::Color::PINK, "Pls spin", 0, TEXT_Y, TEXT_HEIGHT, TEXT_THICKNESS};
    // Line plsSpinBox{UISubsystem::Color::PINK, 0, TEXT_Y+TEXT_HEIGHT/2, 0,
    // TEXT_Y+TEXT_HEIGHT/2, TEXT_HEIGHT+TEXT_THICKNESS*2};
};
}  // namespace src::control::client_display::graphics