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
 * World frame turret yaw controller. Requires that a development board be mounted rigidly on the
 * turret and connected via the `TurretMCBCanComm` class. The development board's IMU is used to
 * determine the turret's world frame coordinates directly, making this controller better than the
 * `WorldFrameChassisImuTurretControllerFF`.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret yaw.
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
 * World frame turret pitch controller. Requires that a development board be mounted rigidly on the
 * turret and connected via the `TurretMCBCanComm` class. The development board's IMU is used to
 * determine the turret's world frame coordinates directly, making this controller better than the
 * `WorldFrameChassisImuTurretControllerFF`.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret pitch.
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
     * @param[in] desiredSetpoint The pitch desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    /// Sets the world frame pitch angle setpoint, refer to top level documentation for more
    /// details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return World frame pitch angle setpoint, refer to top level documentation for more details.
    inline WrappedFloat getSetpoint() const final { return worldFrameSetpoint; }

    /// @return World frame pitch angle setpoint, refer to top level documentation for more details.
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
