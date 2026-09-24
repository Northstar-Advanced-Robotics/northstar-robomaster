#ifndef TURRET_MOTOR_GM6020_HPP_
#define TURRET_MOTOR_GM6020_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor.hpp"
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
class TurretMotorDJI final : public TurretMotor
{
public:
    /// Maximum output, voltage control between [-24, 24] volts scaled up to [-30,000, 30,000] units
    static constexpr float MAX_OUT_6020 = 30'000;

    /**
     * @param[in] motor The hardware motor driving this axis. Not owned; must outlive this object.
     * @param[in] motorConfig Mounting and travel limits for this axis. Asserts
     *      `minAngle <= maxAngle`.
     */
    TurretMotorDJI(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig);

    /// Brings the hardware motor up. Must be called before the axis can be driven.
    inline void initialize() override { motor->initialize(); }

    /// Samples the encoder and refreshes the cached chassis-frame angle, scaling by `config.ratio`.
    /// Falls back to `config.startAngle` while the motor is offline. Call once per iteration.
    void updateMotorAngle() override;

    /**
     * Commands the hardware motor, if it is online. Ignored while offline.
     *
     * @param[in] out The desired output in the motor's own command units, clamped here to
     *      +/-`MAX_OUT_6020`.
     */
    void setMotorOutput(float out) override;

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    inline void attachTurretController(
        const algorithms::TurretControllerInterface *turretController) override
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
    void setChassisFrameSetpoint(WrappedFloat setpoint) override;

    /// @return `true` if the hardware motor is connected and powered on
    inline bool isOnline() const { return motor->isMotorOnline(); }

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians
     */
    inline WrappedFloat getChassisFrameSetpoint() const override { return chassisFrameSetpoint; }

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    inline const WrappedFloat &getChassisFrameMeasuredAngle() const override
    {
        return chassisFrameMeasuredAngle;
    }

/// Motor-to-axis gearing for the yaw belt drive: the M3508 gearbox ratio times the pulley ratio
/// (64:94 on hero, 54:81 elsewhere). Used only by `getChassisFrameVelocitySUS`.
#ifdef TARGET_HERO
    static constexpr float RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * (64.0f / 94.0f);
#else
    static constexpr float RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * (54.0f / 81.0f);
#endif
    /**
     * @return This axis' angular velocity in rad/s, scaled by `RATIO` rather than by
     *      `config.ratio`.
     *
     * @warning Reaches past the `MotorInterface` abstraction to the concrete `DjiMotor`'s internal
     *      encoder, so it is only valid when the motor really is a `DjiMotor`. Prefer
     *      `getChassisFrameVelocity` unless you specifically need the belt-ratio scaling.
     */
    inline float getChassisFrameVelocitySUS() const
    {
        return static_cast<tap::motor::DjiMotor *>(motor)->getInternalEncoder().getVelocity() *
               RATIO;
    }

    /**
     * @return angular velocity of the turret, in rad/sec, positive rotation is defined by the
     * motor.
     */
    inline float getChassisFrameVelocity() const override
    {
        return motor->getEncoder()->getVelocity();
    }

    /// @return turret controller controlling this motor (as specified by `attachTurretController`)
    const algorithms::TurretControllerInterface *getTurretController() const override
    {
        return turretController;
    }

    /// @return The mounting and travel-limit configuration this axis was constructed with.
    const TurretMotorConfig &getConfig() const override { return config; }

    /**
     * @return How far this axis is from its own stored setpoint, in radians. **Signed**: positive
     *      means the setpoint is counterclockwise of the measurement. Wrapped to [-PI, PI] when
     *      the axis is unlimited; unwrapped, and so possibly larger than PI, when it is limited.
     */
    float getValidChassisMeasurementError() const override;

    /**
     * The same error calculation as `getValidChassisMeasurementError`, but against a
     * caller-supplied setpoint and measurement rather than the ones this object stores.
     *
     * Use this when the measurement comes from somewhere other than the encoder -- typically a
     * turret-mounted IMU transformed into the chassis frame.
     *
     * @param[in] setpoint The setpoint to measure from, in radians.
     * @param[in] measurement A chassis-frame angle in radians.
     * @return The **signed** error, positive when the setpoint is counterclockwise of the
     *      measurement.
     *
     * @note `measurement` need not be wrapped. For a limited axis it specifically should **not**
     *      be, or the endstop logic will wrap the wrong way.
     */
    float getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
        const override;

    /// @return The output last written to the hardware motor, in the motor's command units.
    int16_t getMotorOutput() const override { return motor->getOutputDesired(); }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Associated turret controller interface that is being used by a command to control this
    /// motor
    const algorithms::TurretControllerInterface *turretController = nullptr;

    /// Copy of `config.ratio`: output revolutions per motor revolution.
    float ratio;

    /// Chassis-frame setpoint in radians, as last set by `setChassisFrameSetpoint`. Clamped to
    /// [`config.minAngle`, `config.maxAngle`] only when `config.limitMotorAngles` is set.
    WrappedFloat chassisFrameSetpoint;

    /// Wrapped chassis frame measured angle between [0, 2*PI). Units radians.
    WrappedFloat chassisFrameMeasuredAngle;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_GM6020_HPP_
