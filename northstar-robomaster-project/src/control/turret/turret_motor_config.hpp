#ifndef TURRET_MOTOR_CONFIG_HPP_
#define TURRET_MOTOR_CONFIG_HPP_

#include <cassert>
#include <cstdint>

namespace src::control::turret
{
/**
 * @ingroup turret
 *
 * How one turret axis' motor is mounted and how far it is allowed to travel.
 *
 * Supplied per robot and per axis from `robot/<target>/<target>_turret_constants.hpp`, since pitch
 * and yaw have different travel limits and every robot mounts its encoders differently.
 */
struct TurretMotorConfig
{
    /// Angle in radians the turret is assumed to be at while its motor is offline. `TurretMotorDJI`
    /// reports this as the measurement until the motor comes up.
    float startAngle = 0;

    /// Encoder count that `startAngle` corresponds to.
    ///
    /// @warning `TurretMotorDJI` never reads this; it derives the angle from the raw encoder
    ///      position scaled by `ratio`. The value is consumed at construction by the `DjiMotor`
    ///      itself (see each robot's control file), not by the turret motor wrapper.
    uint16_t startEncoderValue = 0;

    /// Lower travel limit in radians, applied only when `limitMotorAngles` is set. Need not be
    /// wrapped to [0, 2*PI), but must be **less than or equal to** `maxAngle` -- `TurretMotorDJI`
    /// asserts this at construction.
    float minAngle = 0;

    /// Upper travel limit in radians, applied only when `limitMotorAngles` is set. Must be greater
    /// than or equal to `minAngle`.
    float maxAngle = 0;

    /// `true` to clamp the setpoint to [`minAngle`, `maxAngle`]; `false` to let the axis turn
    /// freely, which is what a continuously rotating yaw axis wants.
    bool limitMotorAngles = true;

    /// Gearing between the motor and the axis it drives: output revolutions per motor revolution.
    /// Scales the encoder angle into an axis angle. 1 means the motor drives the axis directly.
    ///
    /// @note Applied to the position measurement but **not** to `getChassisFrameVelocity`, so with
    ///      a ratio other than 1 the position and velocity getters are in different units.
    float ratio = 1;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_CONFIG_HPP_
