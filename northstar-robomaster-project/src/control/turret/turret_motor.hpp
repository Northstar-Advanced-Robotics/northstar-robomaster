#ifndef TURRET_MOTOR_HPP_
#define TURRET_MOTOR_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/communication/sensors/encoder/encoder_interface.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor_config.hpp"

namespace src::control::turret
{
/**
 * @ingroup turret
 *
 * One axis of the turret gimbal (pitch or yaw). Wraps a hardware `tap::motor::MotorInterface` and
 * adds the turret-specific logic on top: the chassis-frame angle measurement, the setpoint, the
 * travel limits, and a record of which controller is currently driving the axis.
 *
 * Nothing here is specific to a motor model. Motor-specific numbers (the output clamp, encoder
 * gearing) come from `TurretMotorConfig`, so any `MotorInterface` can drive an axis.
 */
class TurretMotor
{
public:
    /**
     * @param[in] motor The hardware motor driving this axis. Not owned; must outlive this object.
     * @param[in] motorConfig Mounting, gearing, output, and travel limits for this axis. Asserts
     *      `minAngle <= maxAngle`.
     * @param[in] velocityEncoder The encoder velocity is read from, scaled by
     *      `motorConfig.velocityRatio`. Not owned. `nullptr` (the default) uses
     *      `motor->getEncoder()`, the same encoder position is read from. Pass the motor's internal
     *      encoder when position comes from an external axis-mounted encoder.
     */
    TurretMotor(
        tap::motor::MotorInterface *motor,
        const TurretMotorConfig &motorConfig,
        const tap::encoder::EncoderInterface *velocityEncoder = nullptr);

    /// Brings the hardware motor up. Must be called before the axis can be driven.
    inline void initialize() { motor->initialize(); }

    /// Samples the encoder and refreshes the cached chassis-frame angle, scaling by `config.ratio`.
    /// Falls back to `config.startAngle` while the motor is offline. Call once per control loop
    /// iteration, before reading any measurement.
    void updateMotorAngle();

    /**
     * Commands the hardware motor, if it is online. Ignored while offline.
     *
     * @param[in] out The desired output in the motor's own command units, clamped here to
     *      +/-`config.maxOutput`. The PID controllers upstream are separately capped at each
     *      robot's `MAX_OUTPUT_GM6020` (25,000 on the standard), so in practice the clamp here is
     *      rarely the binding one.
     */
    void setMotorOutput(float out);

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    inline void attachTurretController(const TurretControllerInterface *turretController)
    {
        this->turretController = turretController;
    }

    /**
     * Sets the chassis-frame angle this axis should be driven to.
     *
     * Clamped to [`config.minAngle`, `config.maxAngle`], but **only** when
     * `config.limitMotorAngles` is set; a freely rotating axis stores the setpoint unchanged.
     *
     * @param[in] setpoint The desired chassis-frame angle, in radians.
     */
    void setChassisFrameSetpoint(tap::algorithms::WrappedFloat setpoint);

    /// @return `true` if the hardware motor is connected and powered on
    inline bool isOnline() const { return motor->isMotorOnline(); }

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians
     */
    inline tap::algorithms::WrappedFloat getChassisFrameSetpoint() const
    {
        return chassisFrameSetpoint;
    }

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    inline const tap::algorithms::WrappedFloat &getChassisFrameMeasuredAngle() const
    {
        return chassisFrameMeasuredAngle;
    }

    /**
     * @return This axis' angular velocity in rad/s, positive in the motor's own direction of
     *      rotation. Read from the velocity encoder given at construction and scaled by
     *      `config.velocityRatio`.
     */
    inline float getChassisFrameVelocity() const
    {
        return velocityEncoder->getVelocity() * config.velocityRatio;
    }

    /// @return The controller currently driving this axis, as set by `attachTurretController`, or
    /// `nullptr` if none. Lets a command discover which controller is running, since controllers
    /// outlive the commands that use them.
    const TurretControllerInterface *getTurretController() const { return turretController; }

    /// @return The mounting and travel-limit configuration this axis was constructed with.
    const TurretMotorConfig &getConfig() const { return config; }

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
    float getValidChassisMeasurementError() const;

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
    float getValidMinError(
        const tap::algorithms::WrappedFloat setpoint,
        const tap::algorithms::WrappedFloat measurement) const;

    /// @return The output last written to the hardware motor, in the motor's command units.
    int16_t getMotorOutput() const { return motor->getOutputDesired(); }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Encoder velocity is read from: the one given at construction, or `motor->getEncoder()`.
    const tap::encoder::EncoderInterface *velocityEncoder;

    /// Associated turret controller interface that is being used by a command to control this
    /// motor
    const TurretControllerInterface *turretController = nullptr;

    /// Chassis-frame setpoint in radians, as last set by `setChassisFrameSetpoint`. Clamped to
    /// [`config.minAngle`, `config.maxAngle`] only when `config.limitMotorAngles` is set.
    tap::algorithms::WrappedFloat chassisFrameSetpoint;

    /// Wrapped chassis frame measured angle between [0, 2*PI). Units radians.
    tap::algorithms::WrappedFloat chassisFrameMeasuredAngle;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_HPP_
