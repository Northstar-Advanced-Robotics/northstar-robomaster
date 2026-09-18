#pragma once

#include <cmath>

#include "control/clientDisplay/graphics/vector_3d.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Projects points in the world onto the operator's screen, so the HUD can draw overlays that line
 * up with what the camera sees.
 *
 * A chain of rigid transforms, each stripping one offset:
 * `robotSpace -> pivotSpace -> vtmSpace -> screenSpace`, with `barrelSpace` as a side branch used
 * when reasoning about where a shot leaves the barrel rather than where the camera is.
 *
 * Every space is right-handed with **+X right, +Y forward, +Z up**. This is **not** the chassis
 * frame (+X forward, +Y left) that `ChassisSubsystem` and `ChassisOdometry` share, so anything taken
 * from them must be converted before entering these transforms. All offsets are in meters and are
 * robot-specific, selected below by build target.
 *
 * Used by `LaneAssistLines` and `PeekingLines`.
 */
class Projections
{
public:
#if defined(TARGET_HERO)  // todo
    // no OFFSET_X_ROBOT_TO_PITCH_PIVOT because pitch rotates around x
    static constexpr float OFFSET_Y_ROBOT_TO_PITCH_PIVOT =
        0;  // meters, forward distance from robot center to pitch rotate point, positive means
            // it is in front of robot center
    static constexpr float OFFSET_Z_ROBOT_TO_PITCH_PIVOT =
        0.5302125f;  // meters, vertical distance from floor to pitch rotate point, positive means
                     // gimbal is above the floor (kind of has to be positive)

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_VTM =
        0;  // meters, side to side distance from robot center to vtm, positive means vtm is to
            // the right of robot center
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_VTM =
        0.210f;  // meters, forward distance from pitch rotate point to vtm, positive means vtm
                 // is in front of pitch pivot point
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_VTM =
        0.110f;  // meters, vertical distance from pitch rotate point to vtm, positive means vtm
                 // is above the pitch pivot point

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_BARREL =
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0.107f;  // meters
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_BARREL = 0;       // meters
#elif defined(TARGET_SENTRY)                                         // todo
    // no OFFSET_X_ROBOT_TO_PITCH_PIVOT because pitch rotates around x
    static constexpr float OFFSET_Y_ROBOT_TO_PITCH_PIVOT =
        0;  // meters, forward distance from robot center to pitch rotate point, positive means
            // it is in front of robot center
    static constexpr float OFFSET_Z_ROBOT_TO_PITCH_PIVOT =
        0.36;  // meters, vertical distance from floor to pitch rotate point, positive means
               // gimbal is above the floor (kind of has to be positive)

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_VTM =
        0;  // meters, side to side distance from robot center to vtm, positive means vtm is to
            // the right of robot center
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_VTM =
        0;  // meters, forward distance from pitch rotate point to vtm, positive means vtm is in
            // front of pitch pivot point
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_VTM =
        0;  // meters, vertical distance from pitch rotate point to vtm, positive means vtm is
            // above the pitch pivot point

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_BARREL =
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0;  // meters
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_BARREL = 0;  // meters
#elif defined(TARGET_STANDARD)                                       // todo
    // no OFFSET_X_ROBOT_TO_PITCH_PIVOT because pitch rotates around x
    static constexpr float OFFSET_Y_ROBOT_TO_PITCH_PIVOT =
        0;  // meters, forward distance from robot center to pitch rotate point, positive means
            // it is in front of robot center
    static constexpr float OFFSET_Z_ROBOT_TO_PITCH_PIVOT =
        0.352358;  // meters, vertical distance from floor to pitch rotate point, positive means
                   // gimbal is above the floor (kind of has to be positive)

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_VTM =
        0;  // meters, side to side distance from robot center to vtm, positive means vtm is to
            // the right of robot center
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_VTM =
        0.1555;  // meters, forward distance from pitch rotate point to vtm, positive means vtm
                 // is in front of pitch pivot point
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_VTM =
        0.0072336;  // meters, vertical distance from pitch rotate point to vtm, positive means
                    // vtm is above the pitch pivot point

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_BARREL =
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0.1555;  // meters
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_BARREL = 0;       // meters
#else                                                                // old infantry, todo
    // no OFFSET_X_ROBOT_TO_PITCH_PIVOT because pitch rotates around x
    static constexpr float OFFSET_Y_ROBOT_TO_PITCH_PIVOT =
        0;  // meters, forward distance from robot center to pitch rotate point, positive means
            // it is in front of robot center
    static constexpr float OFFSET_Z_ROBOT_TO_PITCH_PIVOT =
        0.36;  // meters, vertical distance from floor to pitch rotate point, positive means
               // gimbal is above the floor (kind of has to be positive)

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_VTM =
        0;  // meters, side to side distance from robot center to vtm, positive means vtm is to
            // the right of robot center
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_VTM =
        0;  // meters, forward distance from pitch rotate point to vtm, positive means vtm is in
            // front of pitch pivot point
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_VTM =
        0;  // meters, vertical distance from pitch rotate point to vtm, positive means vtm is
            // above the pitch pivot point

    static constexpr float OFFSET_X_PITCH_PIVOT_TO_BARREL =
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0;  // meters
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_BARREL = 0;  // meters
#endif

    /**
     * Robot space to pivot space.
     *
     * Robot space has the robot's center at (x, y) = (0, 0) and the ground at z = 0.
     *
     * @param[in] v A point in robot space, in meters.
     * @return The same point in pivot space.
     *
     * @note The output is not yet rotated by the turret's pitch -- callers must apply that
     *      themselves, since pivot space turns with the gimbal.
     */
    static Vector3d robotSpaceToPivotSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX(),
            v.getY() - OFFSET_Y_ROBOT_TO_PITCH_PIVOT,
            v.getZ() - OFFSET_Z_ROBOT_TO_PITCH_PIVOT);
    }

    /**
     * Pivot space to VTM (camera) space.
     *
     * Pivot space has the pitch pivot at the origin.
     *
     * @param[in] v A point in pivot space, in meters.
     * @return The same point relative to the video transmission module's lens.
     */
    static Vector3d pivotSpaceToVtmSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX() - OFFSET_X_PITCH_PIVOT_TO_VTM,
            v.getY() - OFFSET_Y_PITCH_PIVOT_TO_VTM,
            v.getZ() - OFFSET_Z_PITCH_PIVOT_TO_VTM);
    }

    /**
     * Pivot space to barrel space.
     *
     * Barrel space has the muzzle -- where the projectile leaves -- at the origin.
     *
     * @param[in] v A point in pivot space, in meters.
     * @return The same point relative to the muzzle.
     */
    static Vector3d pivotSpaceToBarrelSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX() - OFFSET_X_PITCH_PIVOT_TO_BARREL,
            v.getY() - OFFSET_Y_PITCH_PIVOT_TO_BARREL,
            v.getZ() - OFFSET_Z_PITCH_PIVOT_TO_BARREL);
    }

    /**
     * Barrel space back to pivot space.
     *
     * @param[in] v A point in barrel space, in meters.
     * @return The same point relative to the pitch pivot.
     *
     * @warning This is **not** the inverse of `pivotSpaceToBarrelSpace`. That subtracts the offset,
     *      so the inverse would add it; this computes `offset - v`, which also negates the vector.
     *      Round-tripping a point through both does not return it.
     */
    static Vector3d barrelSpaceToPivotSpace(Vector3d& v)
    {
        return Vector3d(
            OFFSET_X_PITCH_PIVOT_TO_BARREL - v.getX(),
            OFFSET_Y_PITCH_PIVOT_TO_BARREL - v.getY(),
            OFFSET_Z_PITCH_PIVOT_TO_BARREL - v.getZ());
    }

    /**
     * VTM space to screen pixels -- a pinhole camera projection.
     *
     * Divides by forward distance so things further away land nearer the center, scales by the
     * camera's focal lengths in pixels, and offsets by the principal point (960, 540), the center
     * of the 1920x1080 display.
     *
     * @param[in] v A point in VTM space, in meters. Must have a positive y (be in front of the
     *      camera); a point at or behind the lens divides by zero or projects nonsensically.
     * @return The corresponding screen position, in pixels from the bottom-left.
     *
     * @note The focal lengths are the camera's own, taken from aruw's `projection_utils.hpp`, and
     *      so are not robot specific.
     */
    static Vector2d vtmSpaceToScreenSpace(Vector3d& v)
    {
        return Vector2d(
            923.4504870 * v.getX() / v.getY() + 960,
            951.2135278 * v.getZ() / v.getY() + 540);
    }
};

}  // namespace src::control::client_display::graphics