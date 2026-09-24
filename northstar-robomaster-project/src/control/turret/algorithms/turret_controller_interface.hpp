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

#ifndef TURRET_CONTROLLER_INTERFACE_HPP_
#define TURRET_CONTROLLER_INTERFACE_HPP_

#include "tap/algorithms/wrapped_float.hpp"

using namespace tap::algorithms;

namespace src::control::turret
{
class TurretMotor;
}

namespace src::control::turret::algorithms
{
/**
 * @ingroup turret
 *
 * An interface describing the functionality of a turret controller. When implementing this class,
 * the user is responsible for designing a controller that will set the desired output of some
 * turret subsystem. Instances of this interface are designed to be used in a command. Using this
 * interface allows you to easily interchange which turret controller is being used for a particular
 * robot.
 *
 * @note All units of setpoints mentioned below are in radians, the same units that the
 * `TurretMotor` uses.
 */
class TurretControllerInterface
{
public:
    /**
     * @param[in] turretMotor The axis this controller drives. Not owned; must outlive the
     *      controller. Exposed to subclasses as the protected `turretMotor` member.
     */
    TurretControllerInterface(TurretMotor &turretMotor) : turretMotor(turretMotor) {}

    /**
     * Initializes the controller, resetting any controllers and configuring any variables that need
     * to be set initially. Expected to be called once before the turret controller's
     * `runController` function is called.
     */
    virtual void initialize() = 0;

    /**
     * Main controller update loop. Expected that the controller is initialized and that this
     * function is only called while `isOnline` returns `true`. Call periodically.
     *
     * @param[in] dt The time difference in **milliseconds** between the previous and current call
     * of `runController`. Note taproot's `SmoothPid` treats `dt` as unitless, so milliseconds is a
     * convention this codebase maintains at every call site rather than something enforced.
     *
     * @param[in] desiredSetpoint The controller's desired setpoint in whatever frame the
     * controller operates in, in radians.
     */
    virtual void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) = 0;

    /**
     * Updates the controller's target without stepping the control loop. Use when taking over from
     * another controller, so the new one starts from the current aim rather than snapping.
     *
     * @param[in] desiredSetpoint The desired setpoint, in radians, in the controller's own frame.
     */
    virtual void setSetpoint(WrappedFloat desiredSetpoint) = 0;

    /**
     * Convenience overload wrapping a raw angle.
     *
     * @param[in] desiredSetpoint The desired setpoint, in radians, in the controller's own frame.
     */
    inline void setSetpoint(float desiredSetpoint) { setSetpoint(Angle(desiredSetpoint)); }

    /**
     * @return The controller's setpoint, units radians, in whatever frame this controller operates
     * in -- which is **not** necessarily the chassis frame the `TurretMotor` stores its setpoint
     * in.
     */
    virtual WrappedFloat getSetpoint() const = 0;

    /**
     * @return The controller's measurement (current value of the system), units radians. **Does
     * not** have to be in the same reference frame as the TurretMotor's `getChassisFrame*`
     * functions. Does not need to be normalized.
     */
    virtual WrappedFloat getMeasurement() const = 0;

    /// @return The measurement taken from the motor encoder specifically, as opposed to
    /// `getMeasurement`, which a world-frame controller sources from an IMU instead. Lets a caller
    /// compare the two. The base implementation returns a constant zero; controllers that can
    /// distinguish the two sources override it.
    virtual WrappedFloat getMeasurementMotor() const { return WrappedFloat(0, 0, M_TWOPI); };

    /**
     * @return `false` if the turret controller should not be running, whether this is because the
     * turret is offline or some sensor the turret controller is using is invalid. Otherwise return
     * `true`.
     */
    virtual bool isOnline() const = 0;

    /**
     * Converts the passed in controllerFrameAngle from the controller frame to the chassis frame of
     * reference.
     *
     * @param[in] controllerFrameAngle Some angle (in radians) in the controller frame. Not required
     * to be normalized.
     * @return The controllerFrameAngle converted to the chassis frame, a value in radians that is
     * not required to be normalized.
     */
    virtual WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const = 0;

    /**
     * Converts the passed in controllerFrameAngle from the chassis frame to the controller frame of
     * reference.
     *
     * @param[in] chassisFrameAngle Some angle (in radians) in the chassis frame. Not required
     * to be normalized.
     * @return The chassisFrameAngle converted to the controller frame, a value in radians that is
     * not required to be normalized.
     */
    virtual WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const = 0;

protected:
    /// The axis this controller drives. Subclasses read its measurement and write its output.
    TurretMotor &turretMotor;
};

/**
 * @ingroup turret
 *
 * A `TurretControllerInterface` that drives a pitch axis.
 *
 * Exists so a command can hold "some pitch controller" without knowing which frame it works in, and
 * so pitch and yaw controllers cannot be passed in the wrong order.
 */
class TurretPitchControllerInterface : public TurretControllerInterface
{
public:
    TurretPitchControllerInterface(TurretMotor &turretMotor)
        : TurretControllerInterface(turretMotor)
    {
    }
};

/**
 * @ingroup turret
 *
 * A `TurretControllerInterface` that drives a yaw axis. The yaw counterpart to
 * `TurretPitchControllerInterface`.
 */
class TurretYawControllerInterface : public TurretControllerInterface
{
public:
    TurretYawControllerInterface(TurretMotor &turretMotor) : TurretControllerInterface(turretMotor)
    {
    }
};
}  // namespace src::control::turret::algorithms

#endif  // TURRET_CONTROLLER_INTERFACE_HPP_
