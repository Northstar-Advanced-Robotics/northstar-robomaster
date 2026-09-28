#ifndef HOPPER_SUBSYSTEM_HPP_
#define HOPPER_SUBSYSTEM_HPP_

#include "tap/communication/gpio/pwm.hpp"
#include "tap/control/subsystem.hpp"


namespace src::control::hopper
{
/**
 * @ingroup hopper_kicker
 *
 * The servo-driven lid over the projectile hopper.
 *
 * The lid is opened to reload and kept closed during a match so projectiles are not thrown out when
 * the robot is hit or beyblading. The servo is held at one of two PWM duty cycles, which are
 * configured per robot since the linkage geometry differs.
 */
class HopperSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] minPwm The duty cycle that holds the lid closed.
     * @param[in] maxPwm The duty cycle that holds the lid open.
     * @param[in] pwmPin The pin the servo's signal line is wired to.
     */
    HopperSubsystem(tap::Drivers* drivers, float minPwm, float maxPwm, tap::gpio::Pwm::Pin pwmPin);

    /// Sets the servo's PWM timer to 50Hz, the rate hobby servos expect. Must be called before the
    /// lid can be driven.
    void initialize() override;

    /// Writes the current duty cycle to the servo. Called once per control loop iteration by the
    /// scheduler.
    void refresh() override;

    /// Does nothing, deliberately: the lid holds its position when the remote disconnects rather
    /// than springing open.
    void refreshSafeDisconnect() override {}

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char* getName() const override { return "hopper subsystem"; }

    /**
     * Writes a duty cycle straight to the servo pin, bypassing the open/closed positions.
     *
     * @param[in] dutyCycle The duty cycle to write, from 0 to 1.
     */
    void setPWM(float dutyCycle);

    /// Drives the lid to its open position.
    void open();

    /// Drives the lid to its closed position.
    void close();

private:
    /// Time in milliseconds of the previous refresh.
    uint32_t prevTime = 0;

    /// The pin the servo's signal line is wired to.
    tap::gpio::Pwm::Pin pwmPin;

    /// The duty cycle that holds the lid closed.
    float minPwm;

    /// The duty cycle that holds the lid open.
    float maxPwm;

    /// The duty cycle currently being written to the servo.
    float pwm;

    /// Unused; the lid is driven to a position rather than at a velocity.
    float velocitySetpoint = 0;

    /// Whether the lid was last commanded open.
    bool isOpen = false;
};

}  // namespace src::control::hopper

#endif  // HERO_AGITATOR_SUBSYSTEM_HPP_