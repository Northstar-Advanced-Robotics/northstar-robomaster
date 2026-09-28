#pragma once

#include "control/clientDisplay/graphics/core/projections.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/clientDisplay/graphics/vector_3d.hpp"
// #include "subsystems/chassis/chassisSubsystem.hpp"
// #include "subsystems/chassis/chassisSubsystemConstants.hpp"
// #include "subsystems/turret/turretSubsystem.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Draws vertical lines marking how far the robot can edge out from cover before it is exposed.
 *
 * Only drawn while peeking is engaged. The lines are computed from the chassis' footprint projected
 * through the turret's current orientation, so they show where the robot's corners will appear
 * rather than where its center is.
 */
class PeekingLines : public GraphicsContainer
{
public:
    PeekingLines(
        src::chassis::ChassisSubsystem* chassis,
        src::control::turret::TurretSubsystem* turret)
        : turret(turret),
          chassis(chassis)
    {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    /// Recomputes the lines from the chassis footprint and turret orientation, and hides them when
    /// not peeking.
    void update()
    {
        if (chassis->isPeeking)
        {
            Vector3d vs{MAGNITUDE, 0, 0};
            Vector3d temp;
            Vector3d temp2;
            Vector2d temp3;
            float xPositions[4];
            int h[] = {0, 0};
            for (int i = 0; i < 4; i++)
            {
                // currently the point is to the right, needs rotated forward
                temp = vs.rotateYaw(
                    ANGLES[i] + turret->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());
                // now we are still in robot space, need to get to pivot
                temp2 = Projections::robotSpaceToPivotSpace(temp);
                // now in pivot, since we don't care about y we won't rotate by pitch
                temp = Projections::pivotSpaceToVtmSpace(temp2);
                // now get to screen space
                temp3 = Projections::vtmSpaceToScreenSpace(temp);
                xPositions[i] = temp3.getX();
                if (temp3.getX() < 0) h[i / 2]++;
                if (temp3.getX() > UISubsystem::SCREEN_WIDTH) h[i / 2]++;
            }
            left.x = std::min(xPositions[0], xPositions[1]);
            left.width = xPositions[0] > xPositions[1] ? xPositions[0] - xPositions[1]
                                                       : xPositions[1] - xPositions[0];
            // if(left.width == UISubsystem::SCREEN_WIDTH) left.width = 0;
            right.x = std::min(xPositions[2], xPositions[3]);
            right.width = xPositions[2] > xPositions[3] ? xPositions[2] - xPositions[3]
                                                        : xPositions[3] - xPositions[2];
            // if(right.width == UISubsystem::SCREEN_WIDTH) right.width = 0;

            left.setHidden(h[0] > 0);
            right.setHidden(h[1] > 0);
        }
        else
        {
            left.hide();
            right.hide();
        }
    }

private:
    src::control::turret::TurretSubsystem* turret;
    src::chassis::ChassisSubsystem* chassis;

    // these numbers need calculated, would be robot dependent but the infantry is the only
    // robot to draw peeking lines, these are for knowing what sector the enemy has to be in for
    // them to not be able to shoot at us if we are hiding our front panel behind a wall. ANGLE1
    // determines the inner edge of the rectangle, the one used to align with a wall. ANGLE2
    // determines the outer edge of the rectangle, the one used to mark how far away from the
    // wall the enemy can be and still not be able to hit us. As a driver, the goal while
    // peeking is to keep the enemy inside of these bounds.
    static constexpr float ANGLE1 =
        PI / 2 - 0.45;  // Make not 0.45 in future  // zero would be directly to the right/left,
                        // 90 would be directly forward
    static constexpr float ANGLE2 = ANGLE1 + 0.06;
    static constexpr float ANGLES[4] = {ANGLE1, ANGLE2, PI - ANGLE1, PI - ANGLE2};
    static constexpr float MAGNITUDE = 0.3;  // need a magnitude to make a 3d point to project

    static constexpr uint16_t BOTTOM_OFFSET =
        0;  // y distance from the end of the line to the bottom of the screen
    static constexpr uint16_t HEIGHT = UISubsystem::SCREEN_HEIGHT;  // height of both rectangles
    static constexpr uint16_t THICKNESS = 2;                        // pixels

    UnfilledRectangle left{UISubsystem::Color::CYAN, 0, BOTTOM_OFFSET, 0, HEIGHT, THICKNESS};
    UnfilledRectangle
        right{UISubsystem::Color::PURPLISH_RED, 0, BOTTOM_OFFSET, 0, HEIGHT, THICKNESS};
};
}  // namespace src::control::client_display::graphics