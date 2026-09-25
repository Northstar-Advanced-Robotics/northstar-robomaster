#ifndef THREE_FLYWHEEL_RUN_COMMAND
#define THREE_FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/three_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * Holds a three-wheel flywheel at a launch speed and spin setting for as long as it is scheduled.
 *
 * Both values are set once at `initialize` and left alone; unlike the two-wheel command there is no
 * trimming against referee system feedback. Runs until interrupted, and spins the flywheels down
 * when it ends.
 */
class ThreeFlywheelRunCommand : public tap::control::Command
{
public:
    /**
     * @param[in] flywheel The flywheel subsystem to drive, taken as a subsystem requirement.
     * @param[in] launchSpeed The launch speed to hold, in meters/second.
     * @param[in] spin The spin setting to apply, as a percentage.
     */
    ThreeFlywheelRunCommand(
        ThreeFlywheelSubsystem *flywheel,
        float launchSpeed = 20.0f,
        float spin = 100.0f);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Flywheel Run Command"; }

    /// Applies the spin setting and commands the flywheels to spin up to the configured launch
    /// speed.
    void initialize() override;

    /// Does nothing; the setpoints applied in `initialize` are held by the subsystem.
    void execute() override {}

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
    ThreeFlywheelSubsystem *flywheel;

    /// The launch speed being held, in meters/second.
    float launchSpeed;
    /// The spin setting being applied, as a percentage.
    float spin;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
