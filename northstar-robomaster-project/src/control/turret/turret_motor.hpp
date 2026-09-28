#ifndef TURRET_MOTOR_HPP_
#define TURRET_MOTOR_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/util_macros.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor_config.hpp"

namespace src::control::turret
{
/**
 * @ingroup turret
 *
 * Logic encapsulating the control of a single axis of a turret gimbal motor. Contains logic for
 * storing chassis relative position measurements and setpoints and logic for limiting the angle
 * setpoint.
 *
 * Currently, there are GM6020-specific motor parameters in this object such that it is expected
 * that the gimbal motor used is a 6020, but in general with some taproot-side MRs, this class can
 * be generalized to work with any motor interface.
 */
class TurretMotor
{
public:
    virtual ~TurretMotor() = default;

    /// Brings the underlying hardware motor up. Must be called before the axis can be driven.
    virtual inline void initialize() = 0;

    /// Samples the encoder and refreshes the cached chassis-frame angle. Implementations fall back
    /// to `config.startAngle` while the motor is offline. Call once per control loop iteration,
    /// before reading any measurement.
    virtual void updateMotorAngle() = 0;

    /**
     * Commands the hardware motor, if it is online. Ignored while offline.
     *
     * @param[in] out The desired output in the motor's own command units. Implementations clamp
     *      this to their own ceiling -- `TurretMotorDJI` uses +/-30,000 for the GM6020. Note the
     *      PID controllers upstream are separately capped at each robot's `MAX_OUTPUT_GM6020`
     *      (25,000 on the standard), so in practice the clamp here is rarely the binding one.
     */
    virtual void setMotorOutput(float out) = 0;

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    virtual inline void attachTurretController(
        const algorithms::TurretControllerInterface *turretController) = 0;
    /**
     * Sets the chassis-frame angle this axis should be driven to.
     *
     * Clamped to [`config.minAngle`, `config.maxAngle`], but **only** when
     * `config.limitMotorAngles` is set; a freely rotating axis stores the setpoint unchanged.
     *
     * @param[in] setpoint The desired chassis-frame angle, in radians.
     */
    virtual void setChassisFrameSetpoint(WrappedFloat setpoint) = 0;

    /// @return `true` if the hardware motor is connected and powered on
    virtual inline bool isOnline() const = 0;

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians
     */
    virtual inline WrappedFloat getChassisFrameSetpoint() const = 0;

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    virtual inline const WrappedFloat &getChassisFrameMeasuredAngle() const = 0;

    /**
     * @return This axis' angular velocity in rad/s, positive in the motor's own direction of
     *      rotation.
     *
     * @warning Unlike `getChassisFrameMeasuredAngle`, this is **not** scaled by `config.ratio`, so
     *      on a geared axis the position and velocity getters are in different units.
     */
    virtual inline float getChassisFrameVelocity() const = 0;

    /// @return The controller currently driving this axis, as set by `attachTurretController`, or
    /// `nullptr` if none. Lets a command discover which controller is running, since controllers
    /// outlive the commands that use them.
    virtual const algorithms::TurretControllerInterface *getTurretController() const = 0;

    /**
     * @return How far this axis is from its own stored setpoint, in radians. **Signed**: positive
     *      means the setpoint is counterclockwise of the measurement.
     *
     * @note Which distance is returned depends on whether the axis is angle-limited:
     * - Unlimited: the shortest wrapped distance, in [-PI, PI]. Rotating either way is allowed, so
     *   the short way round is always the right answer.
     * - Limited: the unwrapped difference, which may exceed PI. A limited axis cannot pass through
     *   its endstops, so the short way round is not necessarily reachable.
     */
    virtual float getValidChassisMeasurementError() const = 0;

    /**
     * The same error calculation as `getValidChassisMeasurementError`, but against a caller-supplied
     * setpoint and measurement rather than the ones this object stores.
     *
     * Use this when the measurement comes from somewhere other than the encoder -- typically a
     * turret-mounted IMU transformed into the chassis frame -- which is what the world-frame
     * controllers do.
     *
     * @param[in] setpoint The setpoint to measure from, in radians.
     * @param[in] measurement A chassis-frame angle in radians.
     * @return The **signed** error, positive when the setpoint is counterclockwise of the
     *      measurement. Wrapped to [-PI, PI] when the axis is unlimited; unwrapped, and so possibly
     *      larger than PI, when it is limited.
     *
     * @note `measurement` need not be wrapped. For a limited axis it specifically should **not**
     *      be, or the endstop logic will wrap the wrong way.
     */
    virtual float getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
        const = 0;

    /// @return The output last written to the hardware motor, in the motor's unitless command
    /// range.
    virtual int16_t getMotorOutput() const = 0;

    /// @return The mounting and travel-limit configuration this axis was constructed with.
    virtual const TurretMotorConfig &getConfig() const = 0;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_HPP_
