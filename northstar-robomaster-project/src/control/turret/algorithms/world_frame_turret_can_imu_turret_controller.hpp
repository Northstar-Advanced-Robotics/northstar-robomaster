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

#ifndef WORLD_FRAME_TURRET_CAN_IMU_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_TURRET_CAN_IMU_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "communication/can/turret/turret_mcb_can_comm.hpp"

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
 * World frame turret yaw controller driven by a second microcontroller mounted on the turret,
 * which streams its IMU over CAN via `TurretMCBCanComm`.
 *
 * An IMU physically on the turret measures turret orientation directly, with no dependence on the
 * yaw encoder -- the reason to prefer this over the chassis-IMU controllers.
 *
 * @warning Not built into any robot. No robot's `Drivers` declares a `TurretMCBCanComm` member,
 *      this class is never instantiated, and `TurretMCBCanComm::sendData` is commented out in
 *      `main.cpp`. What the robots actually run is
 *      `WorldFrameYawTurretImuCascadePidTurretController`, which reads the chassis board's BMI088
 *      despite its own comment historically claiming otherwise.
 *
 * @warning `TurretMCBCanComm`'s angle getters return **degrees**, not radians, so this controller
 *      would need that fixed before it could work. See that class for details.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret yaw.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFrameYawTurretCanImuCascadePidTurretController final
    : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] turretMCBCanComm A TurretMCBCanComm object that will be queried for IMU
     * information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFrameYawTurretCanImuCascadePidTurretController(
        const src::can::TurretMCBCanComm &turretMCBCanComm,
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

    /// @return World frame yaw angle measurement, refer to top level documentation for more
    /// details.
    WrappedFloat getMeasurement() const final;

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    const src::can::TurretMCBCanComm &turretMCBCanComm;

    SmoothPid &positionPid;
    SmoothPid &velocityPid;

    WrappedFloat worldFrameSetpoint;
};

/**
 * @ingroup turret
 *
 * World frame turret pitch controller driven by the turret-mounted microcontroller's IMU over CAN.
 *
 * The pitch counterpart to `WorldFrameYawTurretCanImuCascadePidTurretController`; see that class,
 * including its warnings -- this one is likewise never instantiated by any robot.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret pitch.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFramePitchTurretCanImuCascadePidTurretController final
    : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] turretMCBCanComm A TurretMCBCanComm object that will be queried for IMU
     * information.
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFramePitchTurretCanImuCascadePidTurretController(
        const src::can::TurretMCBCanComm &turretMCBCanComm,
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
    const src::can::TurretMCBCanComm &turretMCBCanComm;

    SmoothPid &positionPid;
    SmoothPid &velocityPid;

    WrappedFloat worldFrameSetpoint;
};
}  // namespace src::control::turret::algorithms

#endif  //  WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_HPP_
