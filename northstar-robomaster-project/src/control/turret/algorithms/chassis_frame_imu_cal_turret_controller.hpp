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

#ifndef CHASSIS_FRAME_IMU_CAL_TURRET_CONTROLLER_HPP_
#define CHASSIS_FRAME_IMU_CAL_TURRET_CONTROLLER_HPP_

#include <cstdint>
#include <queue>

#include "tap/algorithms/smooth_pid.hpp"

#include "turret_controller_interface.hpp"

namespace src::control::turret
{
class TurretMotor;
}

namespace src::control::turret::algorithms
{
/**
 * @ingroup turret
 *
 * Chassis-frame yaw controller used while the IMU is being calibrated.
 *
 * Calibration needs the turret held still, and a plain position PID is not good enough for that: it
 * saturates on a large error and jitters on a small one. So this controller adds two behaviours on
 * top of `ChassisFrameYawTurretController` -- it clamps its output to `maxOutput` once the error
 * exceeds `errorForMaxOuput`, and below `errorForAveraging` it averages the error over the last ten
 * samples to damp encoder noise.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFrameYawImuCalTurretController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     * @param[in] errorForMaxOuput Error, in radians, at or above which the output is clamped to
     *      `maxOutput`. (Spelling of this parameter matches the declaration.)
     * @param[in] maxOutput The clamp applied above `errorForMaxOuput`, in raw motor output units.
     * @param[in] errorForAveraging Error, in radians, below which the error is averaged over the
     *      last ten samples rather than used directly.
     */
    ChassisFrameYawImuCalTurretController(
        TurretMotor &yawMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        float errorForMaxOuput,
        float maxOutput,
        float errorForAveraging);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] dt Milliseconds since the previous call.
     * @param[in] desiredSetpoint The yaw desired setpoint in the chassis frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return The chassis frame yaw turret measurement, refer to top level documentation for more
    /// details.
    WrappedFloat getMeasurement() const final;

    /**
     * @return The yaw setpoint, in the chassis frame.
     */
    WrappedFloat getSetpoint() const final;

    bool isOnline() const final;

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final
    {
        return controllerFrameAngle;
    }

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const final
    {
        return chassisFrameAngle;
    }

    inline float getPositionBufferAverage()
    {
        float posTotal = 0;
        int numVals = 0;
        for (const auto &value : positionBuffer)
        {
            posTotal += value;
            numVals++;
        }
        return posTotal / numVals;
    }

private:
    tap::algorithms::SmoothPid pid;
    float errorForMaxOuput;
    float maxOutput;
    float errorForAveraging;
    std::deque<float> positionBuffer;
};

/**
 * @ingroup turret
 *
 * Chassis-frame pitch controller used while the IMU is being calibrated.
 *
 * The pitch counterpart to `ChassisFrameYawImuCalTurretController`; see that class for why plain
 * position PID is not sufficient during calibration.
 *
 * @warning Unlike the yaw class, this one sets the motor output to the averaged **error** in
 *      radians rather than to the PID output, so its output is not in motor units at all.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFramePitchImuCalTurretController final : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     * @param[in] errorForMaxOuput Error, in radians, at or above which the output is clamped to
     *      `maxOutput`. (Spelling of this parameter matches the declaration.)
     * @param[in] maxOutput The clamp applied above `errorForMaxOuput`, in raw motor output units.
     * @param[in] errorForAveraging Error, in radians, below which the error is averaged over the
     *      last ten samples rather than used directly.
     */
    ChassisFramePitchImuCalTurretController(
        TurretMotor &pitchMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        float errorForMaxOuput,
        float maxOutput,
        float errorForAveraging);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] dt Milliseconds since the previous call.
     * @param[in] desiredSetpoint The pitch desired setpoint in the chassis frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /**
     * @return The pitch setpoint, in the chassis frame.
     */
    WrappedFloat getSetpoint() const final;

    /// @return The chassis frame pitch turret measurement, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurement() const final;

    bool isOnline() const final;

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final
    {
        return controllerFrameAngle;
    }

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const final
    {
        return chassisFrameAngle;
    }

    inline float getPositionBufferAverage()
    {
        float posTotal = 0;
        int numVals = 0;
        for (const auto &value : positionBuffer)
        {
            posTotal += value;
            numVals++;
        }
        return posTotal / numVals;
    }

private:
    tap::algorithms::SmoothPid pid;
    float errorForMaxOuput;
    float maxOutput;
    float errorForAveraging;
    std::deque<float> positionBuffer;
};

}  // namespace src::control::turret::algorithms

#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
