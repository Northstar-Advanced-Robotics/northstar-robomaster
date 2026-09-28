/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

// mm tasty imports
#include <tap/algorithms/linear_interpolation_predictor.hpp>
#include <tap/algorithms/ramp.hpp>

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

namespace src
{
namespace control
{
/**
 * @ingroup robots
 *
 * The single place commands read operator input from.
 *
 * `CommandMapper` decides *when* a command runs; this decides what the operator is currently
 * asking for once it is running. Keeping the remote behind one interface means deadzones, scaling,
 * and the blending of stick and keyboard/mouse input are applied consistently everywhere.
 *
 * The two turret getters return a normalized, deadzoned value. The drivetrain getters do **not**
 * -- they return meters/second already scaled against the referee system's power limit.
 */
class ControlOperatorInterface
{
public:
    /// Mouse x movement is clamped to +/- this before scaling, bounding how fast one frame of
    /// mouse motion can slew the turret.
    static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
    /// Mouse y equivalent of `USER_MOUSE_YAW_MAX`.
    static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
    /// Maps clamped mouse x onto [-1, 1]. Negative, so moving the mouse right yields negative yaw.
    static constexpr float USER_MOUSE_YAW_SCALAR = -(1.0f / USER_MOUSE_YAW_MAX);
    /// Maps clamped mouse y onto [-1, 1]. Negative, so moving the mouse down yields negative pitch.
    static constexpr float USER_MOUSE_PITCH_SCALAR = -(1.0f / USER_MOUSE_PITCH_MAX);

    /// Scales the remote stick's contribution to turret input, making the sticks less sensitive
    /// than the mouse.
    static constexpr float REMOTE_TURRET_SCALAR = 0.6f;

    /**
     * @param[in] drivers The global drivers object, polled for remote and referee system state.
     */
    ControlOperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}

    /**
     * @return How fast the operator wants the turret to yaw, as the sum of the remote stick
     *      (scaled by `REMOTE_TURRET_SCALAR`) and the mouse (clamped then normalized), so at most
     *      +/-1.6. Values with magnitude under 0.01 are snapped to exactly 0, so a resting mouse
     *      does not drift the turret.
     */
    mockable float getTurretYawInput();

    /**
     * @return How fast the operator wants the turret to pitch. Same construction and same 0.01
     *      deadzone as `getTurretYawInput`.
     */
    mockable float getTurretPitchInput();

    /**
     * @return Desired sideways chassis velocity in **meters/second**, positive to the left.
     *
     * Blends the remote stick with A/D keyboard input, uses `CHASSIS_WALK_SPEED_MPS` unless shift
     * is held for the full power-limited speed, and clamps the total to what the referee system's
     * current power budget allows.
     */
    float getDrivetrainHorizontalTranslation();

    /**
     * @return Desired forward chassis velocity in **meters/second**. Same construction as
     *      `getDrivetrainHorizontalTranslation`, with W/S as the keyboard axis.
     */
    float getDrivetrainVerticalTranslation();

    /**
     * @return Desired chassis rotational velocity.
     *
     * @warning Currently returns 0 unconditionally -- the entire body is commented out, so the
     *      operator has no direct rotation control. Chassis rotation comes only from the drive
     *      commands (beyblade, orient, wiggle).
     */
    float getDrivetrainRotationalTranslation();

private:
    tap::Drivers *drivers;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterR = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor chassisRInput;
};
}  // namespace control

}  // namespace src

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_