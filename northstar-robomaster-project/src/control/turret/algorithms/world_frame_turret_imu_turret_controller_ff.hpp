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

#ifndef WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_FF_HPP_
#define WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_FF_HPP_

#include <cstdint>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "turret_controller_interface.hpp"

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
 * World frame turret yaw controller with a static-friction feedforward term, driven by the onboard
 * BMI088 IMU.
 *
 * Identical to `WorldFrameYawTurretImuCascadePidTurretController` except that it also computes a
 * feedforward intended to break stiction when the turret is stalled just off target -- the regime
 * where the position error is small enough that the PID output alone cannot overcome friction.
 *
 * @warning The feedforward is currently computed and then **discarded**: `runController` ends with
 *      `return velocityPidOutput;  //+ staticFrictionFF;`. Until that is restored, this class
 *      behaves exactly like the non-FF controller, and the `STATIC_FRICTION_FF_*` constants it
 *      passes to that helper have no effect.
 *
 * @note Reads `drivers.bmi088` directly, not a turret-mounted board over `TurretMCBCanComm`.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFrameYawTurretImuCascadePidTurretControllerFF final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFrameYawTurretImuCascadePidTurretControllerFF(
        tap::Drivers &drivers,
        TurretMotor &yawMotor,
        SmoothPid &positionPid,
        SmoothPid &velocityPid);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] dt Milliseconds since the previous call.
     * @param[in] desiredSetpoint The unwrapped yaw desired setpoint in the world frame. Clamped
     * within chassis frame turret angle limits if applicable.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    /// Sets the world frame yaw angle setpoint, refer to top level documentation for more details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return World frame yaw angle setpoint, refer to top level documentation for more details.
    inline WrappedFloat getSetpoint() const final { return worldFrameSetpoint; }

    /// @return World frame yaw angle measurement from IMU, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurement() const final;

    /// @return World frame yaw angle measurement from MOTOR, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurementMotor() const override final;

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    tap::Drivers &drivers;

    SmoothPid &positionPid;
    SmoothPid &velocityPid;

    WrappedFloat worldFrameSetpoint;

    float worldFrameMeasurementIMU;
    int32_t IMUrevolutions;

    inline WrappedFloat getBmi088Yaw(bool negitive = false) const
    {
        return negitive ? Angle(drivers.bmi088.getYaw() * -1) : Angle(drivers.bmi088.getYaw());
    }

    inline float getBmi088YawVelocity() const { return drivers.bmi088.getGz(); }
};

/**
 * @ingroup turret
 *
 * World frame turret pitch controller with a static-friction feedforward term, driven by the
 * onboard BMI088 IMU. Also applies gravity compensation, as the non-FF pitch controller does.
 *
 * @warning Shares the same helper as the yaw class above, so the feedforward is likewise computed
 *      and then discarded. Until that is restored this behaves exactly like
 *      `WorldFramePitchTurretImuCascadePidTurretController`.
 *
 * @note Reads `drivers.bmi088` directly, not a turret-mounted board over `TurretMCBCanComm`.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFramePitchTurretImuCascadePidTurretControllerFF final
    : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFramePitchTurretImuCascadePidTurretControllerFF(
        tap::Drivers &drivers,
        TurretMotor &pitchMotor,
        SmoothPid &positionPid,
        SmoothPid &velocityPid);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] dt Milliseconds since the previous call.
     * @param[in] desiredSetpoint The pitch desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    /// Sets the world frame pitch angle setpoint, refer to top level documentation for more
    /// details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return World frame pitch angle setpoint, refer to top level documentation for more details.
    inline WrappedFloat getSetpoint() const final { return worldFrameSetpoint; }

    /// @return World frame pitch angle measurement, taken from the IMU.
    WrappedFloat getMeasurement() const final;

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    tap::Drivers &drivers;

    SmoothPid &positionPid;
    SmoothPid &velocityPid;

    WrappedFloat worldFrameSetpoint;

    inline WrappedFloat getBmi088Pitch(bool negitive = false) const
    {
        return negitive ? Angle(drivers.bmi088.getPitch() * -1) : Angle(drivers.bmi088.getPitch());
    }

    inline float getBmi088PitchVelocity() const { return drivers.bmi088.getGy(); }
};
}  // namespace src::control::turret::algorithms

#endif  //  WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_FF_HPP_
