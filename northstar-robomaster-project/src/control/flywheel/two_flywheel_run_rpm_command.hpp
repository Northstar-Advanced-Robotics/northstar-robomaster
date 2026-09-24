#ifndef TWO_FLYWHEEL_RUN_RPM_COMMAND
#define TWO_FLYWHEEL_RUN_RPM_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * Holds the flywheels at a fixed motor speed for as long as it is scheduled.
 *
 * Commands RPM directly rather than a launch speed, bypassing the launch-speed-to-RPM
 * interpolation. Used when bringing up hardware or measuring the points that go into that
 * interpolation, where the mapping is the thing being established. Runs until interrupted, and
 * spins the flywheels down when it ends.
 */
class TwoFlywheelRunRPMCommand : public tap::control::Command
{
public:
    /**
     * @param[in] flywheel The flywheel subsystem to drive, taken as a subsystem requirement.
     * @param[in] rpm The motor speed to hold, in RPM.
     */
    TwoFlywheelRunRPMCommand(TwoFlywheelSubsystem *flywheel, float rpm);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Flywheel Run Command"; }

    /// Commands the flywheels to spin up to the configured RPM.
    void initialize() override;

    /// Re-commands the configured RPM.
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

    /// The motor speed being held, in RPM.
    float rpm;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
