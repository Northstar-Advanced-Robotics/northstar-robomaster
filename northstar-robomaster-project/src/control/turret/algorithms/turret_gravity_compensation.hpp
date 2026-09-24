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

#ifndef TURRET_GRAVITY_COMPENSATION_HPP_
#define TURRET_GRAVITY_COMPENSATION_HPP_

#include <cmath>
#include <cstdint>

#include "modm/math/geometry/angle.hpp"

namespace src::control::turret::algorithms
{
/**
 * Computes the motor output needed to hold the turret's pitch against gravity.
 *
 * Without this the pitch axis sags below its setpoint and the PID has to carry a standing error to
 * hold aim. The offset peaks when the centre of gravity is level with the pivot -- where the mass
 * exerts the most torque -- and falls to zero when it is directly above or below it.
 *
 * @param[in] cgX Centre of gravity relative to the pitch pivot along the barrel, positive forward.
 * @param[in] cgZ Centre of gravity relative to the pitch pivot perpendicular to the barrel,
 *      positive up.
 * @param[in] pitchAngleFromCenter Turret pitch relative to horizontal, in radians.
 * @param[in] gravityCompensatorMax The peak output this function may return, i.e. what it takes to
 *      hold the turret level.
 * @return The output to add to the pitch controller, in [-`gravityCompensatorMax`,
 *      `gravityCompensatorMax`].
 *
 * @note `cgX` and `cgZ` only enter as the ratio `atan(cgZ / cgX)`, so their scale cancels -- any
 *      consistent length unit works, and the values in the robot constants are not literally
 *      millimetres.
 */
float computeGravitationalForceOffset(
    const float cgX,
    const float cgZ,
    const float pitchAngleFromCenter,
    const float gravityCompensatorMax);
}  // namespace src::control::turret::algorithms

#endif  // GRAVITY_COMPENSATION_HPP_
