#pragma once

#include "tap/algorithms/math_user_utils.hpp"

#include "control/clientDisplay/graphics/core/projections.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/clientDisplay/graphics/vector_2d.hpp"
#include "control/clientDisplay/graphics/vector_3d.hpp"
// #include "subsystems/turret/turretSubsystem.hpp"
// #include "subsystems/indexer/IndexerSubsystem.hpp"
// #include "subsystems/jetson/JetsonSubsystemConstants.hpp"
#include "control/turret/turret_subsystem.hpp"

using namespace tap::communication::serial;
namespace src::control::client_display::graphics
{
class Reticle : public GraphicsContainer
{
public:  // important constants and enums
    // panel things
    static constexpr float PANEL_WIDTH = 0.135;   // meters, wider panel is 0.230
    static constexpr float PANEL_HEIGHT = 0.125;  // meters, height across the surface
    static constexpr float PANEL_ANGLE =
        15 * PI / 180;  // radians, tilt of the panel, 0 would be panel is not tilted
    static constexpr float AVERAGE_HEIGHT_OFF_GROUND =
        0.18;  // meters, center of panel to ground, for calculating where on screen reticle
               // things should be

    // distances down range and colors to draw them
    static constexpr int NUM_THINGS = 9;

#if defined(HERO)
    static constexpr float DISTANCES[NUM_THINGS] = {
        1,
        2,
        3,
        5,
        7,
        9,
        11,
        12,
        13};  // meters, y distances, measured from center of robot to center of panel
#else
    static constexpr float DISTANCES[NUM_THINGS] = {
        1,
        2,
        3,
        5,
        7,
        9,
        11,
        12,
        13};  // meters, y distances, measured from center of robot to center of panel
#endif

    static constexpr int NUM_COLORS = 1;
    static constexpr UISubsystem::Color COLORS[NUM_COLORS] = {UISubsystem::Color::GREEN};

    enum class ReticleDrawMode : uint8_t
    {
        RECTANGLES =
            0,  // multiple rectangles of different colors, tops and bottoms of them are tops
                // and bottoms of panels, midpoints of sides are edges of standard size panels
        HORIZ_LINES = 1,  // multiple of horizontal lines, widths represent panel widths, heights
                          // represent where the center of a standard panel needs to be to hit it,
                          // lines go across the center of a panel, endpoints are edges of panels
        VERT_LINES = 2,   // multiple of (almost) vertical lines, heights represent panel
                          // heights, heights represent where the top and bottom edges of either
                          // panel needs to be to hit it, lines go across the left and/or right
                          // edge of the panel, depending on ReticleSidedMode
        TRAPEZOIDS = 3,   // like RECTANGLES but uses many lines to make it a trapezoid, tracing
                          // the edges of panels. Not recommended for use with
                          // ReticleSolveMode::FOR_HEIGHT_OFF_GROUND
        BOTH_LINES = 4,   // combination of HORIZ_LINES and VERT_LINES
    };

    enum class ReticleSolveMode : uint8_t
    {
        FOR_HEIGHT_OFF_GROUND = 0,  // uses pitch and DISTANCES away to solve for height,
                                    // redraws the reticle often because pitch changes often
        FOR_PITCH = 1,  // uses AVERAGE_HEIGHT_OFF_GROUND and DISTANCES away to solve for pitch,
                        // reticle doesn't ever need redrawn because constants are constant
    };

    enum class ReticleSidedMode : uint8_t
    {
        LEFT = 0,
        RIGHT = 1,
        BOTH = 2,  // both left and right
        ALT = 3,   // alternate between left and right
    };

private:  // draw settings
    ReticleDrawMode drawMode = ReticleDrawMode::VERT_LINES;
    ReticleSolveMode solveMode = ReticleSolveMode::FOR_PITCH;
    ReticleSidedMode sidedMode = ReticleSidedMode::BOTH;

    static constexpr int DIAGONAL_OFFSET = 50;  // when can't shoot, the vertical line becomes
                                                // diagonal, by shifting x by this amount

public:
    Reticle(
        tap::Drivers* drivers,
        src::control::turret::TurretSubsystem* turret /*IndexerSubsystem* index*/)
        : drivers(drivers),
          turret(turret)
    //   index(index)
    {
        for (int i = 0; i < NUM_THINGS; i++)
        {
            rects[i].color = COLORS[i % NUM_COLORS];
            rectsContainer.addGraphicsObject(rects + i);
            for (int j = 0; j < NUM_LINES; j++)
            {
                lines[i][j].color = COLORS[i % NUM_COLORS];
                linesContainer.addGraphicsObject(&lines[i][j]);
            }
            forPitchLandingSpotsSolved[i] = false;
        }
        verticalLine.x1 = UISubsystem::HALF_SCREEN_WIDTH;
        verticalLine.x2 = UISubsystem::HALF_SCREEN_WIDTH;

        addGraphicsObject(&rectsContainer);
        addGraphicsObject(&linesContainer);
        addGraphicsObject(&verticalLine);
    }

    void update()
    {
        float pitch = turret->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue();

        ReticleSidedMode adjustedSidedMode =
            drawMode == ReticleDrawMode::TRAPEZOIDS ? ReticleSidedMode::BOTH : sidedMode;

        // bool canShoot = true;

        // if (!index->isIndexOnline())
        // {
        //     verticalLine.color = UISubsystem::Color::PINK;
        //     canShoot = false;
        // }

        // if (!index->isProjectileAtBeam())
        // {
        //     verticalLine.color = UISubsystem::Color::BLACK;
        //     canShoot = false;
        // }

        // if (!index->heatAllowsShooting())
        // {
        //     verticalLine.color = UISubsystem::Color::WHITE;
        //     canShoot = false;
        // }

        verticalLine.x1 = UISubsystem::HALF_SCREEN_WIDTH;
        verticalLine.x2 = UISubsystem::HALF_SCREEN_WIDTH;
        // if (canShoot)
        // {
        //     verticalLine.color =
        //         index->refPoweringIndex() ? UISubsystem::Color::WHITE :
        //         UISubsystem::Color::PINK;
        //     verticalLine.thickness = 1;
        // }
        // else
        // {
        //     verticalLine.thickness = 10;
        //     verticalLine.x1 += DIAGONAL_OFFSET;
        //     verticalLine.x2 -= DIAGONAL_OFFSET;
        // }

        solvedForPitchLandingSpotThisCycle = false;
        for (int i = 0; i < NUM_THINGS; i++)
        {
            // assume all lines are hidden, if we are drawing lines
            for (int j = 0; j < NUM_LINES; j++)
            {
                lines[i][j].hide();
            }
            rects[i].setHidden(drawMode == ReticleDrawMode::RECTANGLES);

            Vector3d landingSpot = calculateLandingSpot(&pitch, i);

            Vector2d r = project(landingSpot + panelEdges[0], pitch);
            Vector2d l = project(landingSpot + panelEdges[1], pitch);
            Vector2d t = project(landingSpot + panelEdges[2], pitch);
            Vector2d b = project(landingSpot + panelEdges[3], pitch);

            Vector2d rt = project(landingSpot + panelCorners[0], pitch);
            Vector2d rb = project(landingSpot + panelCorners[1], pitch);
            Vector2d lt = project(landingSpot + panelCorners[2], pitch);
            Vector2d lb = project(landingSpot + panelCorners[3], pitch);

            if (drawMode == ReticleDrawMode::RECTANGLES)
            {
                // need the edges because the corners will trace a trapezoid
                rects[i].x = l.getX();
                rects[i].width = r.getX() - l.getX();
                rects[i].y = b.getY();
                rects[i].height = t.getY() - b.getY();
            }
            if (drawMode == ReticleDrawMode::HORIZ_LINES || drawMode == ReticleDrawMode::BOTH_LINES)
            {
                // horiz lines just wants left and right edges
                lines[i][2].x1 = l.getX();
                lines[i][2].x2 = r.getX();
                lines[i][2].y1 = l.getY();
                lines[i][2].y2 = r.getY();  // should be the same as left.getY()

                lines[i][2].show();
            }
            bool drawVertLines = drawMode == ReticleDrawMode::VERT_LINES ||
                                 drawMode == ReticleDrawMode::BOTH_LINES ||
                                 drawMode == ReticleDrawMode::TRAPEZOIDS;
            if (drawVertLines)
            {
                // vert lines traces one edge of the panel, so they aren't always exactly
                // vertical

                lines[i][0].x1 = rt.getX();
                lines[i][0].y1 = rt.getY();
                lines[i][0].x2 = rb.getX();
                lines[i][0].y2 = rb.getY();

                lines[i][1].x1 = lt.getX();
                lines[i][1].y1 = lt.getY();
                lines[i][1].x2 = lb.getX();
                lines[i][1].y2 = lb.getY();

                bool showRight =
                    drawVertLines && (adjustedSidedMode == ReticleSidedMode::BOTH ||
                                      adjustedSidedMode == ReticleSidedMode::RIGHT ||
                                      (adjustedSidedMode == ReticleSidedMode::ALT && (i % 2)));
                bool showLeft =
                    drawVertLines && (adjustedSidedMode == ReticleSidedMode::BOTH ||
                                      adjustedSidedMode == ReticleSidedMode::LEFT ||
                                      (adjustedSidedMode == ReticleSidedMode::ALT && !(i % 2)));

                lines[i][0].setHidden(!showRight);
                lines[i][1].setHidden(!showLeft);
            }
            if (drawMode == ReticleDrawMode::TRAPEZOIDS)
            {
                lines[i][2].x1 = rt.getX();
                lines[i][2].y1 = rt.getY();
                lines[i][2].x2 = lt.getX();
                lines[i][2].y2 = lt.getY();

                lines[i][3].x1 = lb.getX();
                lines[i][3].y1 = lb.getY();
                lines[i][3].x2 = rb.getX();
                lines[i][3].y2 = rb.getY();

                lines[i][2].show();
                lines[i][3].show();
            }

            if (solvedForPitchLandingSpotThisCycle)
            {
                // if we just solved a pitch, don't solve any more this update(), do it the next
                // time
                break;
            }
        }

        verticalLine.y1 = lines[0][0].y1;
        verticalLine.y2 = lines[NUM_THINGS - 1][0].y2;
    }

private:
    tap::Drivers* drivers;
    src::control::turret::TurretSubsystem* turret;
    // IndexerSubsystem* index;

    Vector3d panelEdges[4] = {
        {PANEL_WIDTH / 2, 0, 0},   // right
        {-PANEL_WIDTH / 2, 0, 0},  // left
        {0,
         std::sin(PANEL_ANGLE) * PANEL_HEIGHT / 2,
         std::cos(PANEL_ANGLE) * PANEL_HEIGHT / 2},  // top
        {0,
         -std::sin(PANEL_ANGLE) * PANEL_HEIGHT / 2,
         -std::cos(PANEL_ANGLE) * PANEL_HEIGHT / 2}};  // bottom

    Vector3d panelCorners[4] = {
        panelEdges[0] + panelEdges[2],   // right top
        panelEdges[0] + panelEdges[3],   // right bottom
        panelEdges[1] + panelEdges[2],   // left top
        panelEdges[1] + panelEdges[3]};  // left bottom

    GraphicsContainer rectsContainer{};
    GraphicsContainer linesContainer{};

    static constexpr int NUM_LINES = 4;  // enough to support trapezoids
    Line lines[NUM_THINGS][NUM_LINES];   // not all are used in every mode
    UnfilledRectangle rects[NUM_THINGS];
    Line verticalLine;

    // for solving for pitch
    static constexpr int MAX_NUM_ITERATIONS =
        10;  // it is difficult to actually solve for pitch because initial launch positions
             // depend on pitch
    int forPitchLandingSpotsSolved[NUM_THINGS];  // so we do a binary search, guessing a pitch,
                                                 // calculating where it lands, and trying a
                                                 // higher or lower pitch accordingly
    Vector3d forPitchLandingSpots[NUM_THINGS];
    float forPitchPitches[NUM_THINGS];
    bool solvedForPitchLandingSpotThisCycle =
        false;  // prevent solving for multiple in one update() cycle so it doesn't take a long
                // time

    Vector2d project(Vector3d in, float pitch)
    {
        Vector3d temp;
        Vector3d temp2;

        // given in pivot, rotate by pitch
        temp = in.rotatePitch(pitch);
        // now that we rotated around pivot, go to vtm
        temp2 = Projections::pivotSpaceToVtmSpace(temp);
        // now get to screen space
        return Projections::vtmSpaceToScreenSpace(temp2);
    }

    Vector3d calculateLandingSpot(float* pitch, int i)
    {
        if (solveMode == ReticleSolveMode::FOR_HEIGHT_OFF_GROUND)
        {
            Vector3d temp{0, 14.5f, 0};
            Vector3d initialVelo = temp.rotatePitch(-*pitch);
            Vector3d initialPos{0, 0, 0};  // shot starts in barrel space
            temp = Projections::barrelSpaceToPivotSpace(initialPos);
            initialPos = temp.rotatePitch(-*pitch);
            // initialPos and initialVelo are in pivot space, parallel to the ground so gravity
            // is pulling in the -z direction only

            float t =
                (DISTANCES[i] - initialPos.getY()) /
                initialVelo.getY();  //(distance to travel [meters]) divided by (speed to get there
                                     //[meters/seconds]) gives (time to get there [seconds])
            float zFinal = initialPos.getZ() + initialVelo.getZ() * t -
                           tap::algorithms::ACCELERATION_GRAVITY / 2 * t *
                               t;  // make sure gravity is negative, the taproot constant is
                                   // positive, need to subtract

            return Vector3d{
                initialPos.getX(),
                DISTANCES[i],
                zFinal};  // side to side doesn't change, we are defining the down range
                          // distance, and we calculated the height off the ground
        }
        else
        {  // FOR_PITCH
            // if solved it earlier, return saved result
            if (forPitchLandingSpotsSolved[i])
            {
                *pitch = forPitchPitches[i];
                return forPitchLandingSpots[i];
            }

            // solve
            forPitchPitches[i] = 0;
            solveMode =
                ReticleSolveMode::FOR_HEIGHT_OFF_GROUND;  // switch to height mode to recurse
                                                          // and calculate the other way
            for (int j = 0; j < MAX_NUM_ITERATIONS; j++)
            {
                forPitchLandingSpots[i] = calculateLandingSpot(forPitchPitches + i, i);

                // this seems backwards, maybe positive pitch is downward?
                if (forPitchLandingSpots[i].getZ() > AVERAGE_HEIGHT_OFF_GROUND)
                {
                    // if j is 0, we add pi/4, 45 degrees
                    // if j is 1, we add pi/8, 22.5 degrees
                    forPitchPitches[i] += PI / (4 << j);
                }
                else
                {
                    // if j is 0, we subtract pi/4, 45 degrees
                    // if j is 1, we subtract pi/8, 22.5 degrees
                    forPitchPitches[i] -= PI / (4 << j);
                }
            }

            forPitchLandingSpotsSolved[i] = true;
            solvedForPitchLandingSpotThisCycle = true;
            solveMode = ReticleSolveMode::FOR_PITCH;  // go back to original mode

            *pitch = forPitchPitches[i];
            return forPitchLandingSpots[i];
        }
    }
};

}  // namespace src::control::client_display::graphics