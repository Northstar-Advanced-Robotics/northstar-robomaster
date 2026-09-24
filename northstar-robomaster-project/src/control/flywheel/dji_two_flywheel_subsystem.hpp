#ifndef DJI_TWO_FLYWHEEL_SUBSYSTEM_HPP_
#define DJI_TWO_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * A two-wheel flywheel driven by a pair of DJI motors.
 *
 * Each wheel runs its own velocity PID against the encoder, and the setpoints are ramped rather
 * than stepped so that spinning up does not draw a current spike. The two wheels are tracked
 * separately throughout, since a launch speed that is correct on average can still be wrong if one
 * wheel is lagging, and the difference is what tells you a wheel is slipping or stalling.
 */
class DJITwoFlywheelSubsystem : public TwoFlywheelSubsystem
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] leftMotorId CAN ID of the left flywheel motor.
     * @param[in] rightMotorId CAN ID of the right flywheel motor.
     * @param[in] canBus The CAN bus both motors are on.
     * @param[in] inInverted `true` if the motors are mounted such that their default directions
     *      must be flipped for the projectile to be launched forward.
     */
    DJITwoFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        bool inInverted = false);

    /// Initializes both motors. Must be called before the flywheels can be commanded.
    void initialize() override;

    /**
     * Sets both wheels to the motor speed that produces the requested launch speed.
     *
     * @param[in] speed The desired launch speed, in meters/second.
     */
    void setDesiredLaunchSpeed(float speed) override;

    /**
     * Sets both wheels to a motor speed directly, bypassing the launch speed conversion.
     *
     * @param[in] rpm The desired motor speed, in RPM.
     */
    void setDesiredFlywheelSpeed(float rpm) override;

    /// @return The requested launch speed for the left wheel, in meters/second.
    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    /// @return The requested launch speed for the right wheel, in meters/second.
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }

    /// @return The requested launch speed averaged across both wheels, in meters/second.
    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight) / 2.0f;
    }

    /// @return The left wheel's target motor speed, in RPM. This is the ramp's destination, not
    /// the value currently being commanded.
    float getDesiredFlywheelSpeedLeft() const { return desiredRpmRampLeft.getTarget(); }
    /// @return The right wheel's target motor speed, in RPM. This is the ramp's destination, not
    /// the value currently being commanded.
    float getDesiredFlywheelSpeedRight() const { return desiredRpmRampRight.getTarget(); }

    /// @return The target motor speed averaged across both wheels, in RPM.
    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight()) / 2.0f;
    }

    /// @return The left wheel's measured speed, in RPM.
    float getCurrentLeftFlywheelMotorRPM() const
    {
        return leftWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    /// @return The right wheel's measured speed, in RPM.
    float getCurrentRightFlywheelMotorRPM() const
    {
        return rightWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    /// @return The measured speed averaged across both wheels, in RPM.
    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM()) / 2.0f;
    }

    /// Advances the speed ramps and runs each wheel's velocity PID. Called once per control loop
    /// iteration by the scheduler.
    void refresh() override;

    /// Cuts both motors' output. Called instead of `refresh` when the remote disconnects, so the
    /// flywheels do not keep spinning unattended.
    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);
        rightWheel.setDesiredOutput(0);
    }

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char *getName() const override { return "Flywheels"; }

private:
    /// Velocity controller for the left wheel.
    modm::Pid<float> velocityPidLeftWheel;
    /// Velocity controller for the right wheel.
    modm::Pid<float> velocityPidRightWheel;

    /// The left wheel's requested launch speed, in meters/second.
    float desiredLaunchSpeedLeft;
    /// The right wheel's requested launch speed, in meters/second.
    float desiredLaunchSpeedRight;

    /// Time in milliseconds of the previous refresh, used to advance the ramps.
    uint32_t prevTime = 0;

    /// Ramps the left wheel's commanded speed toward its target.
    tap::algorithms::Ramp desiredRpmRampLeft;
    /// Ramps the right wheel's commanded speed toward its target.
    tap::algorithms::Ramp desiredRpmRampRight;

    /// The left flywheel motor.
    tap::motor::DjiMotor leftWheel;
    /// The right flywheel motor.
    tap::motor::DjiMotor rightWheel;

    /**
     * @param[in] launchSpeed The desired launch speed, in meters/second.
     * @return The motor speed that produces it, in RPM, interpolated from the measured
     *      `MPS_TO_RPM` table.
     */
    float launchSpeedToFlywheelRpm(float launchSpeed) const override;
};  // namespace src::control::flywheel

}  // namespace src::control::flywheel

#endif