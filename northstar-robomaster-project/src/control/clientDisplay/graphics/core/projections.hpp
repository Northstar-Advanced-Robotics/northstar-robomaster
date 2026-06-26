#pragma once

#include <cmath>

#include "control/clientDisplay/graphics/vector_3d.hpp"

namespace src::control::client_display::graphics
{
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
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit. Exit velocity
            // is in JetsonSubsystemConstants.hpp, as J
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
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit. Exit velocity
            // is in JetsonSubsystemConstants.hpp, as J
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
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit. Exit velocity
            // is in JetsonSubsystemConstants.hpp, as J
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
        0;  // meters, like OFFSET_X_PITCH_PIVOT_TO_VTM but for where shots exit. Exit velocity
            // is in JetsonSubsystemConstants.hpp
    static constexpr float OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0;  // meters
    static constexpr float OFFSET_Z_PITCH_PIVOT_TO_BARREL = 0;  // meters
#endif

    /** robot space is defined as: center of robot at (x, y)=(0, 0), ground at z=0,
     * positive x is to the right, positive y is forward, positive z is above
     * make sure to rotate pitch on the output*/
    static Vector3d robotSpaceToPivotSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX(),
            v.getY() - OFFSET_Y_ROBOT_TO_PITCH_PIVOT,
            v.getZ() - OFFSET_Z_ROBOT_TO_PITCH_PIVOT);
    }

    /** pivot space is defined as: center of pitch pivot point at origin,
     * positive x is to the right, positive y is forward, positive z is above */
    static Vector3d pivotSpaceToVtmSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX() - OFFSET_X_PITCH_PIVOT_TO_VTM,
            v.getY() - OFFSET_Y_PITCH_PIVOT_TO_VTM,
            v.getZ() - OFFSET_Z_PITCH_PIVOT_TO_VTM);
    }

    /** barrel space is defined as: origin is where the projectile gets launched from at the
     * initialShotVelocity from JetsonSubsystemConstants.hpp
     * positive x is to the right, positive y is forward, positive z is above */
    static Vector3d pivotSpaceToBarrelSpace(Vector3d& v)
    {
        return Vector3d(
            v.getX() - OFFSET_X_PITCH_PIVOT_TO_BARREL,
            v.getY() - OFFSET_Y_PITCH_PIVOT_TO_BARREL,
            v.getZ() - OFFSET_Z_PITCH_PIVOT_TO_BARREL);
    }

    /** barrel space is defined as: origin is where the projectile gets launched from at the
     * initialShotVelocity from JetsonSubsystemConstants.hpp
     * positive x is to the right, positive y is forward, positive z is above */
    static Vector3d barrelSpaceToPivotSpace(Vector3d& v)
    {
        return Vector3d(
            OFFSET_X_PITCH_PIVOT_TO_BARREL - v.getX(),
            OFFSET_Y_PITCH_PIVOT_TO_BARREL - v.getY(),
            OFFSET_Z_PITCH_PIVOT_TO_BARREL - v.getZ());
    }

    /** vtm space is defined as: vtm at origin, positive x is to the right, positive y is
       forward, positive z is above these numbers are from aruw, projection_utils.hpp, and
       aren't robot specific*/
    static Vector2d vtmSpaceToScreenSpace(Vector3d& v)
    {
        return Vector2d(
            923.4504870 * v.getX() / v.getY() + 960,
            951.2135278 * v.getZ() / v.getY() + 540);
    }
};

}  // namespace src::control::client_display::graphics