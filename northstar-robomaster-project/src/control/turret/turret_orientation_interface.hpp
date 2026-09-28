/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_ORIENTATION_INTERFACE_HPP_
#define TURRET_ORIENTATION_INTERFACE_HPP_

#include <stdint.h>

#include "modm/math/geometry/vector3.hpp"

namespace src::can
{
class TurretMCBCanComm;
}

namespace src::control::turret
{
/**
 * @ingroup turret
 *
 * An interface that provides turret world yaw and pitch.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above. Pitch is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class TurretOrientationInterface
{
public:
    /**
     * @return The turret's world-relative yaw in radians, counterclockwise seen from above.
     *
     * @warning `StandardTurretSubsystem` implements this as the raw board IMU yaw, without adding
     *      the yaw motor's chassis-relative angle. On a robot whose IMU is chassis-mounted that
     *      makes the result the *chassis* heading, not the turret's.
     */
    virtual inline float getWorldYaw() const = 0;

    /**
     * @return The turret's world-relative pitch in radians, counterclockwise seen from the
     *      turret's right.
     *
     * @warning Same caveat as `getWorldYaw`: the standard's implementation returns the board IMU's
     *      pitch directly.
     */
    virtual inline float getWorldPitch() const = 0;

    /**
     * @return When the angles above were measured, in microseconds.
     *
     * @warning `StandardTurretSubsystem` returns the current time at the moment of the call, not
     *      the time the IMU sample was actually taken, so it cannot be used to compensate for
     *      sensor latency.
     */
    virtual inline uint32_t getLastMeasurementTimeMicros() const = 0;

    /**
     * @return Offset from the chassis origin to the turret's pivot, in the chassis frame, in
     *      meters. Needed to convert between chassis-relative and turret-relative target
     *      positions.
     *
     * @warning `StandardTurretSubsystem` returns a zero vector -- the real offset has not been
     *      measured and filled in.
     */
    virtual modm::Vector3f getTurretOffset() const = 0;

    /**
     * @return Distance between the pitch axis and the yaw axis in the X-Y plane, in meters.
     *
     * @warning `StandardTurretSubsystem` returns 0, as above.
     */
    virtual inline float getPitchOffset() const = 0;

};  // class TurretOrientation

}  // namespace src::control::turret

#endif  // TURRET_ORIENTATION_INTERFACE_HPP_
