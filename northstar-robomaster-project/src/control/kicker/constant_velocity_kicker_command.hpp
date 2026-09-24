#ifndef CONSTANT_VELOCITY_KICKER_COMMAND_HPP
#define CONSTANT_VELOCITY_KICKER_COMMAND_HPP

#include "tap/control/command.hpp"

#include "kicker_subsystem.hpp"

namespace src::control::kicker
{
/**
 * @ingroup hopper_kicker
 *
 * Runs the kicker at a fixed velocity for as long as it is scheduled.
 *
 * On the hero the kicker feeds projectiles from the hopper into the flywheels, and unlike the
 * agitator it is not indexed to discrete shots, so it is simply run continuously while firing.
 * Stops the kicker when it ends.
 */
class ConstantVelocityKickerCommand : public tap::control::Command
{
public:
    /**
     * @param[in] kicker The kicker to drive, taken as a subsystem requirement.
     * @param[in] velocitySetpoint The velocity to run at, in radians/second at the output shaft.
     */
    ConstantVelocityKickerCommand(src::kicker::KickerSubsystem* kicker, float velocitySetpoint);

    /// Commands the kicker to the configured velocity.
    void initialize() override;

    /// Does nothing; the subsystem holds the setpoint applied in `initialize`.
    void execute() override;

    /**
     * Commands the kicker to a stop.
     *
     * @param[in] interrupted Ignored; the kicker is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return Always `true`; the kicker can be run at any time. Gate this command on a governor
    /// to make firing conditional.
    bool isReady() override;

    /// @return Always `false`; the kicker runs until this command is interrupted.
    bool isFinished() const override;

    /// @return The name used to identify this command in logs and the scheduler.
    const char* getName() const override { return "ConstantVelocityKickerCommand"; }

private:
    /// The kicker being driven.
    src::kicker::KickerSubsystem* kicker;

    /// The velocity being commanded, in radians/second at the output shaft.
    float velocitySetpoint;
};
}  // namespace src::control::kicker

#endif  // CONSTANT_VELOCITY_KICKER_COMMAND_HPP