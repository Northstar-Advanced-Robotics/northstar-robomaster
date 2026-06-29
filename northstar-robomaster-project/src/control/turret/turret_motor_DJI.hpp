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
     * Construct a turret motor with some particular hardware motor interface and a motor
     * configuration struct.
     */
    TurretMotorDJI(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig);

    inline void initialize() override { motor->initialize(); }

    /// Updates the measured motor angle
    void updateMotorAngle() override;

    /**
     * Set the motor's desired output when the motor is online. The output is expected to be in the
     * motor's unitless form. For the GM6020, the motor output is limited between [-MAX_OUT_6020,
     * MAX_OUT_6020].
     *
     * @param[in] out The desired motor output.
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
     * Sets (and limits!) the chassis frame turret measurement.
     *
     * The setpoint is limited between the min and max config angles as specified in the
     * constructor.
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

/**
 * @return angular velocity of the turret, in SUS rad/sec, positive rotation is defined by the
 * motor.
 */
#ifdef TARGET_HERO
    static constexpr float RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * (64.0f / 94.0f);
#else
    static constexpr float RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * (54.0f / 81.0f);
#endif
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

    /// @return The turret motor config struct associated with this motor
    const TurretMotorConfig &getConfig() const override { return config; }

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
    float getValidChassisMeasurementError() const override;

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
    float getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
        const override;

    int16_t getMotorOutput() const override { return motor->getOutputDesired(); }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Associated turret controller interface that is being used by a command to control this
    /// motor
    const algorithms::TurretControllerInterface *turretController = nullptr;

    /// ratio of motor rotations per rotation of controled pivot
    float ratio;

    /// Unwrapped chassis frame setpoint specified by the user and limited to `[config.minAngle,
    /// config.maxAngle]`. Units radians.
    WrappedFloat chassisFrameSetpoint;

    /// Wrapped chassis frame measured angle between [0, 2*PI). Units radians.
    WrappedFloat chassisFrameMeasuredAngle;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_GM6020_HPP_
