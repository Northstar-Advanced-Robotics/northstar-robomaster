#ifndef TWO_FLYWHEEL_RUN_COMMAND
#define TWO_FLYWHEEL_RUN_COMMAND

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"

#include "control/flywheel/two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * Holds the flywheels at a launch speed for as long as it is scheduled, trimming that speed to keep
 * the projectile's measured muzzle velocity inside the referee system's limit.
 *
 * The commanded launch speed and the speed the referee system actually measures diverge as wheels
 * wear and projectiles vary, so after each shot the commanded speed is nudged down if the measured
 * speed came in above `upperLimit` and up if it came in below `lowerLimit`. Aiming at a band rather
 * than a single value keeps the correction from oscillating. Runs until interrupted, and spins the
 * flywheels down when it ends.
 */
class TwoFlywheelRunCommand : public tap::control::Command
{
public:
    /**
     * @param[in] flywheel The flywheel subsystem to drive, taken as a subsystem requirement.
     * @param[in] launchSpeed The initial launch speed, in meters/second.
     * @param[in] refSerial The referee system link supplying measured muzzle velocities. When
     *      `nullptr` the launch speed is held fixed with no trimming.
     */
    TwoFlywheelRunCommand(
        TwoFlywheelSubsystem *flywheel,
        float launchSpeed,
        tap::communication::serial::RefSerial *refSerial);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Flywheel Run Command"; }

    /// Commands the flywheels to spin up to the current launch speed.
    void initialize() override;

    /// Adjusts the launch speed when the referee system reports a new shot outside the target
    /// band, then re-commands it.
    void execute() override;

    /**
     * Commands the flywheels to zero, spinning them down.
     *
     * @param[in] interrupted Ignored; the flywheels are stopped either way.
     */
    void end(bool interrupted) override;

    /// @return Always `false`; this command runs until something else interrupts it.
    bool isFinished() const { return false; }

private:
    /// The flywheel subsystem being driven.
    TwoFlywheelSubsystem *flywheel;

    /// The launch speed currently commanded, in meters/second. Trimmed over time to track the
    /// referee system's measurements.
    float launchSpeed;

    /// The referee system link, or `nullptr` if muzzle velocity feedback is unavailable.
    tap::communication::serial::RefSerial *refSerial;

/// Target band and step size for the launch speed trim, in meters/second. The hero fires a
/// larger projectile under a lower speed limit, so it aims at a different band.
#ifdef TARGET_HERO
    float upperLimit = 14.5f;
    float lowerLimit = 13.5f;
    float increment = 0.1f;
    float decrement = 0.1f;
    float lastShotSpeed = 14;
#else
    float upperLimit = 24.5f;
    float lowerLimit = 23.5f;
    float increment = 0.1f;
    float decrement = 0.1f;
    float lastShotSpeed = 24;
#endif
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
