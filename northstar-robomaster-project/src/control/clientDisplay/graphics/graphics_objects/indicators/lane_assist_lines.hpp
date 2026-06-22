#pragma once

#include "control/clientDisplay/graphics/core/projections.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/clientDisplay/graphics/vector_3d.hpp"
// #include "subsystems/turret/turretSubsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::client_display::graphics
{
// looks like / \ at the bottom of the screen
class LaneAssistLines : public GraphicsContainer
{
public:
    LaneAssistLines(src::control::turret::TurretSubsystem* turret) : turret(turret)
    {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update()
    {
        float pitch = turret->pitchMotor.getChassisFrameMeasuredAngle().getUnwrappedValue();
        // vs's in robot space, don't need to rotate with drivetrain, does need to rotate with
        // pitch
        Vector3d rightTop3 = Vector3d{ROBOT_RADIUS, FORWARD_DISTANCE, 0};
        Vector3d rightRef3 = Vector3d{ROBOT_RADIUS, FORWARD_DISTANCE - ROBOT_RADIUS, 0};
        Vector3d leftTop3 = Vector3d{-ROBOT_RADIUS, FORWARD_DISTANCE, 0};
        Vector3d leftRef3 = Vector3d{-ROBOT_RADIUS, FORWARD_DISTANCE - ROBOT_RADIUS, 0};

        Vector2d rightTop2 = project(rightTop3, pitch);
        if (rightTop2.getY() < 0)
        {
            // if top line is off the bottom of the screen, don't project the other points and
            // draw nothing
            this->hide();  // hide all the lines I contain
            return;
        }

        Vector2d rightRef2 = project(rightRef3, pitch);
        Vector2d leftTop2 = project(leftTop3, pitch);
        Vector2d leftRef2 = project(leftRef3, pitch);

        Vector2d rightBottom2 = getBottom(rightTop2, rightRef2);
        Vector2d leftBottom2 = getBottom(leftTop2, leftRef2);

        this->show();  // show all the lines I contain

        right.x1 = rightTop2.getX();
        right.y1 = rightTop2.getY();

        right.x2 = rightBottom2.getX();
        right.y2 = rightBottom2.getY();

        left.x1 = leftTop2.getX();
        left.y1 = leftTop2.getY();

        left.x2 = leftBottom2.getX();
        left.y2 = leftBottom2.getY();
    }

private:
// might be robot specific in the future, but right now standard and hero have the same
// drivetrain
#ifdef TARGET_HERO
    static constexpr float ROBOT_RADIUS = 0.3125;  // meters
#else
    static constexpr float ROBOT_RADIUS = 0.27;  // meters
#endif
    static constexpr float FORWARD_DISTANCE =
        1.2;  // meters, how far forward the laneassistlines extend to

    static constexpr uint16_t THICKNESS = 2;  // pixels

    src::control::turret::TurretSubsystem* turret;

    Line left{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};
    Line right{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};

    Vector2d project(Vector3d in, float pitch)
    {
        Vector3d temp;
        Vector3d temp2;

        // currently  we are in robot space, need to get to pivot
        temp2 = Projections::robotSpaceToPivotSpace(in);
        // now in pivot, rotate by pitch
        temp = temp2.rotatePitch(pitch);
        // now that we rotated around pivot, go to vtm
        temp2 = Projections::pivotSpaceToVtmSpace(temp);
        // now get to screen space
        return Projections::vtmSpaceToScreenSpace(temp2);
    }

    Vector2d getBottom(Vector2d top, Vector2d ref)
    {
        Vector2d diff = ref - top;
        float slope = diff.getY() / diff.getX();
        return Vector2d(top.getX() - top.getY() / slope, 0);
    }
};
}  // namespace src::control::client_display::graphics