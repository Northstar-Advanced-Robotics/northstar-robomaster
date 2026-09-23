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

    /**
     * Construct a turret motor with some particular hardware motor interface and a motor
     * configuration struct.
     */
    virtual inline void initialize() = 0;

    /// Updates the measured motor angle
    virtual void updateMotorAngle() = 0;

    /**
     * Set the motor's desired output when the motor is online. The output is expected to be in the
     * motor's unitless form. For the GM6020, the motor output is limited between [-MAX_OUT_6020,
     * MAX_OUT_6020].
     *
     * @param[in] out The desired motor output.
     */
    virtual void setMotorOutput(float out) = 0;

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    virtual inline void attachTurretController(
        const TurretControllerInterface *turretController) = 0;
    /**
     * Sets (and limits!) the chassis frame turret measurement.
     *
     * The setpoint is limited between the min and max config angles as specified in the
     * constructor.
     */
    virtual void setChassisFrameSetpoint(tap::algorithms::WrappedFloat setpoint) = 0;

    /// @return `true` if the hardware motor is connected and powered on
    virtual inline bool isOnline() const = 0;

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians
     */
    virtual inline tap::algorithms::WrappedFloat getChassisFrameSetpoint() const = 0;

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    virtual inline const tap::algorithms::WrappedFloat &getChassisFrameMeasuredAngle() const = 0;

    /**
     * @return angular velocity of the turret, in rad/sec, positive rotation is defined by the
     * motor.
     */
    virtual inline float getChassisFrameVelocity() const = 0;

    /// @return turret controller controlling this motor (as specified by `attachTurretController`)
    virtual const TurretControllerInterface *getTurretController() const = 0;

    /**
     * @return Valid minimum error between the chassis relative setpoint and measurement, in
     * radians.
     *
     * @note A valid measurement error is either:
     * - The shortest wrapped distance between the chassis frame measurement and setpoint
     *   if the turret motor is not limited to some min/max values.
     * - The absolute difference between the chassis frame measurement and setpoint if the
     *   turret motor is limited to some min/max values.
     */
    virtual float getValidChassisMeasurementError() const = 0;

    /**
     * @param[in] measurement A turret measurement in the chassis frame, an angle in radians. This
     * can be encoder based (via getChassisFrameMeasuredAngle) or can be measured by some other
     * means (for example, an IMU on the turret that is than transformed to the chassis frame).
     *
     * @return The minimum error between the chassis frame setpoint and the specified measurement.
     * If the turret motor is not limited, the error is wrapped between [0, 2*PI), otherwise the
     * error is absolute.
     *
     * @note Call getValidChassisMeasurementError if you want the error between the chassis-frame
     * setpoint and measurement
     *
     * @note The measurement does not need to be normalized to [0, 2*PI]. In fact, if the turret
     * motor is limited, an unwrapped measurement should be used in order to avoid unexpected
     * wrapping errors.
     *
     * @note Before calling this function, you **must** first set the chassis frame setpoint before
     * calling this function (i.e. call `setChassisFrameSetpoint`).
     */
    virtual float getValidMinError(
        const tap::algorithms::WrappedFloat setpoint,
        const tap::algorithms::WrappedFloat measurement) const = 0;

    virtual int16_t getMotorOutput() const = 0;

    /// @return The turret motor config struct associated with this motor
    virtual const TurretMotorConfig &getConfig() const = 0;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_HPP_
