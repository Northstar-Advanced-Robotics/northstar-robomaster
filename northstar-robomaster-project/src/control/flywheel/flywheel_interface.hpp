#ifndef FLYWHEEL_INTERFACE_HPP_
#define FLYWHEEL_INTERFACE_HPP_

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * The interface every flywheel subsystem presents to the code that fires projectiles.
 *
 * Callers think in launch speed (meters/second, which is what the referee system limits and what
 * ballistics needs) while the hardware is commanded in motor RPM. Implementations own the
 * conversion between the two, so a two-wheel and a three-wheel flywheel with different wheel sizes
 * can be driven by the same commands.
 */
class FlywheelInterface
{
public:
    FlywheelInterface() = default;

    ~FlywheelInterface() = default;

    /**
     * Sets the speed projectiles should leave the barrel at, converting it to the corresponding
     * motor speed.
     *
     * @param[in] speed The desired launch speed in meters/second, at most
     *      `MAX_DESIRED_LAUNCH_SPEED_MPS`.
     */
    virtual void setDesiredLaunchSpeed(float speed) = 0;
    /// @return The currently requested launch speed, in meters/second.
    virtual float getDesiredLaunchSpeed() const = 0;
    /**
     * Sets the flywheel motor speed directly, bypassing the launch speed conversion.
     *
     * @param[in] rpm The desired motor speed, in RPM.
     */
    virtual void setDesiredFlywheelSpeed(float rpm) = 0;
    /// @return The currently requested motor speed, in RPM.
    virtual float getDesiredFlywheelSpeed() const = 0;
    /// @return The measured speed averaged across the flywheel motors, in RPM. Compare against
    /// `getDesiredFlywheelSpeed` to tell whether the flywheels have spun up.
    virtual float getCurrentFlywheelAverageMotorRPM() const = 0;

protected:
    /// Ceiling on the requested launch speed, in meters/second. Above the referee system's limit,
    /// so it guards against nonsense inputs rather than enforcing the rules.
    static constexpr float MAX_DESIRED_LAUNCH_SPEED_MPS = 30.0f;

private:
    /**
     * Maps a launch speed to the motor speed that produces it. Depends on wheel diameter and on
     * how much the projectile slips, so each implementation supplies its own mapping, usually an
     * interpolation over measured data.
     *
     * @param[in] launchSpeed The desired launch speed, in meters/second.
     * @return The motor speed to command, in RPM.
     */
    virtual float launchSpeedToFlywheelRpm(float launchSpeed) const = 0;
};

}  // namespace src::control::flywheel

#endif  // FLYWHEEL_INTERFACE_HPP_