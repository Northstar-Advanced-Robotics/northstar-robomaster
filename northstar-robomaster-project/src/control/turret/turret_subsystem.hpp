/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/control/turret_subsystem_interface.hpp"
#include "tap/communication/sensors/encoder/encoder_interface.hpp"
#include "tap/motor/motor_interface.hpp"

#include "turret_motor.hpp"
#include "turret_motor_config.hpp"

#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::turret
{
class TurretPitchControllerInterface;
class TurretYawControllerInterface;

/**
 * @ingroup turret
 *
 * A turret's two axes -- pitch and yaw -- bundled into one subsystem, so a command can reserve the
 * whole turret rather than each gimbal separately.
 *
 * Owns a `TurretMotor` per axis and does little beyond forwarding `initialize`/`refresh` to them;
 * the actual control loops live in the `algorithms/` controllers, which commands attach to these
 * motors.
 *
 * Angles use a right-handed frame: yaw increases counterclockwise seen from above, pitch increases
 * counterclockwise seen from the turret's right.
 *
 * @note Angle *ranges* are per-axis, not 0 to 2*PI as an earlier version of this comment claimed.
 *      Pitch is typically a signed range about zero (the standard uses -40 to +45 degrees), and a
 *      freely rotating yaw axis is unbounded. See each robot's `TurretMotorConfig`.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] pitchMotor Hardware motor for the pitch axis. Not owned; must outlive this object.
     * @param[in] yawMotor Hardware motor for the yaw axis. Not owned; must outlive this object.
     * @param[in] pitchMotorConfig Mounting and travel limits for the pitch axis.
     * @param[in] yawMotorConfig Mounting and travel limits for the yaw axis.
     * @param[in] yawVelocityEncoder Encoder to read yaw velocity from, or `nullptr` to use the yaw
     *      motor's own `getEncoder()`. See `TurretMotor`.
     */
    explicit TurretSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* pitchMotor,
        tap::motor::MotorInterface* yawMotor,
        const TurretMotorConfig& pitchMotorConfig,
        const TurretMotorConfig& yawMotorConfig,
        const tap::encoder::EncoderInterface* yawVelocityEncoder = nullptr);

    /// Initializes both axes' motors. Must be called before the turret can be driven.
    void initialize() override;

    /// Refreshes both axes' cached encoder angles. Called once per control loop iteration by the
    /// scheduler. Note it does **not** run the control loops -- the attached controllers do that,
    /// driven by whichever command is scheduled.
    void refresh() override;

    /// Zeroes both motors' output. Called instead of `refresh` when the remote disconnects.
    void refreshSafeDisconnect() override
    {
        yawMotor.setMotorOutput(0);
        pitchMotor.setMotorOutput(0);
    }

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char* getName() const override { return "Turret"; }

    /// @return `true` if both the pitch and yaw motors are online. `ImuCalibrateCommand` waits on
    /// this before calibrating.
    mockable inline bool isOnline() const { return pitchMotor.isOnline() && yawMotor.isOnline(); }

    /// Associated with and contains logic for controlling the turret's pitch motor
    TurretMotor pitchMotor;
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretMotor yawMotor;

};  // class TurretSubsystem

}  // namespace src::control::turret

#endif  // TURRET_SUBSYSTEM_HPP_