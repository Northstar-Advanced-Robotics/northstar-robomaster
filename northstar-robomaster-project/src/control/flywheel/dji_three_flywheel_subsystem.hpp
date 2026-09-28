#ifndef DJI_THREE_FLYWHEEL_SUBSYSTEM_HPP
#define DJI_THREE_FLYWHEEL_SUBSYSTEM_HPP

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "three_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * A three-wheel flywheel driven by three DJI motors: left, right, and down.
 *
 * Each wheel runs its own velocity PID against the encoder, with ramped setpoints so that spinning
 * up does not draw a current spike. The wheels are given different launch speeds according to the
 * selected spin setting, which is what puts spin on the projectile, so each wheel's speed is
 * tracked separately rather than as a single flywheel speed.
 */
class DJIThreeFlywheelSubsystem : public ThreeFlywheelSubsystem
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] leftMotorId CAN ID of the left flywheel motor.
     * @param[in] rightMotorId CAN ID of the right flywheel motor.
     * @param[in] downMotorId CAN ID of the lower flywheel motor.
     * @param[in] canBus The CAN bus all three motors are on.
     */
    DJIThreeFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::motor::MotorId downMotorId,
        tap::can::CanBus canBus);

    /// Initializes all three motors. Must be called before the flywheels can be commanded.
    void initialize() override;

    /**
     * Selects the spin setting, which changes the ratio between the wheels' speeds and the table
     * used to convert launch speed to RPM.
     *
     * @param[in] spin The spin setting as a percentage. Unsupported values fall back to 100.
     */
    void setDesiredSpin(u_int16_t spin) override;

    /// @return The currently selected spin setting.
    float getDesiredSpin() const override { return desiredSpin; }

    /**
     * Sets the launch speed, distributing it across the three wheels according to the selected
     * spin setting.
     *
     * @param[in] speed The desired launch speed, in meters/second.
     */
    void setDesiredLaunchSpeed(float speed) override;

    /**
     * Sets all three wheels to a motor speed directly, bypassing the launch speed and spin
     * conversion.
     *
     * @param[in] rpm The desired motor speed, in RPM.
     */
    void setDesiredFlywheelSpeed(float rpm) override;

    /// @return The requested launch speed for the left wheel, in meters/second.
    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    /// @return The requested launch speed for the right wheel, in meters/second.
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    /// @return The requested launch speed for the lower wheel, in meters/second.
    float getDesiredLaunchSpeedDown() const { return desiredLaunchSpeedDown; }

    /// @return The requested launch speed averaged across all three wheels, in meters/second.
    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight + desiredLaunchSpeedDown) / 3.0f;
    }

    /// @return The left wheel's target motor speed, in RPM.
    float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    /// @return The right wheel's target motor speed, in RPM.
    float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    /// @return The lower wheel's target motor speed, in RPM.
    float getDesiredFlywheelSpeedDown() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedDown);
    }

    /// @return The target motor speed averaged across all three wheels, in RPM.
    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight() +
                getDesiredFlywheelSpeedDown()) /
               3.0f;
    }

    /// @return The left wheel's measured speed, in RPM.
    float getCurrentLeftFlywheelMotorRPM() const { return getWheelRPM(&leftWheel); }

    /// @return The right wheel's measured speed, in RPM.
    float getCurrentRightFlywheelMotorRPM() const { return getWheelRPM(&rightWheel); }

    /// @return The lower wheel's measured speed, in RPM.
    float getCurrentDownFlywheelMotorRPM() const { return getWheelRPM(&downWheel); }

    /// @return The measured speed averaged across all three wheels, in RPM.
    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM() +
                getCurrentDownFlywheelMotorRPM()) /
               3.0f;
    }

    /// Advances the speed ramps and runs each wheel's velocity PID. Called once per control loop
    /// iteration by the scheduler.
    void refresh() override;

    /// Cuts all three motors' output. Called instead of `refresh` when the remote disconnects, so
    /// the flywheels do not keep spinning unattended.
    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);  // TODO CHANGE
        rightWheel.setDesiredOutput(0);
        downWheel.setDesiredOutput(0);
    }

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char *getName() const override { return "Flywheels"; }

private:
    /// Velocity controller for the left wheel.
    modm::Pid<float> velocityPidLeftWheel;
    /// Velocity controller for the right wheel.
    modm::Pid<float> velocityPidRightWheel;
    /// Velocity controller for the lower wheel.
    modm::Pid<float> velocityPidDownWheel;

    /// The left wheel's requested launch speed, in meters/second.
    float desiredLaunchSpeedLeft;
    /// The right wheel's requested launch speed, in meters/second.
    float desiredLaunchSpeedRight;
    /// The lower wheel's requested launch speed, in meters/second.
    float desiredLaunchSpeedDown;

    /// Time in milliseconds of the previous refresh, used to advance the ramps.
    uint32_t prevTime = 0;

    /// Ramps the left wheel's commanded speed toward its target.
    tap::algorithms::Ramp desiredRpmRampLeft;
    /// Ramps the right wheel's commanded speed toward its target.
    tap::algorithms::Ramp desiredRpmRampRight;
    /// Ramps the lower wheel's commanded speed toward its target.
    tap::algorithms::Ramp desiredRpmRampDown;

    /// The left flywheel motor.
    tap::motor::DjiMotor leftWheel;
    /// The right flywheel motor.
    tap::motor::DjiMotor rightWheel;
    /// The lower flywheel motor.
    tap::motor::DjiMotor downWheel;

    /**
     * @param[in] launchSpeed The desired launch speed, in meters/second.
     * @return The motor speed that produces it, in RPM, interpolated from the measured table for
     *      the currently selected spin setting.
     */
    float launchSpeedToFlywheelRpm(float launchSpeed) const override;

    /**
     * @param[in] motor The motor to read.
     * @return Its measured speed in RPM, converted from the encoder's radians/second.
     */
    float getWheelRPM(const tap::motor::DjiMotor *motor) const
    {
        return motor->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }
};

}  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_SUBSYSTEM
