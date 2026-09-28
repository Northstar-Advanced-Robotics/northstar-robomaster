#ifndef SET_FIRE_RATE_COMMAND_HPP_
#define SET_FIRE_RATE_COMMAND_HPP_

#include <optional>

#include "tap/control/command.hpp"

#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"

namespace src::agitator
{
/**
 * @ingroup agitator
 *
 * Sets the agitator's manually selected fire rate and finishes immediately.
 *
 * Intended to be bound to a remote input so the operator can switch between fire rates. Above a
 * threshold rate the agitator is switched to constant rotation rather than firing discrete shots,
 * since at that point the two are indistinguishable and constant rotation is smoother.
 */
class SetFireRateCommand : public tap::control::Command
{
public:
    /**
     * @param[in] subsystem The agitator subsystem, taken as a requirement so this command
     *      interrupts whatever else is driving the agitator.
     * @param[in] fireRateReselectionManager The manager whose fire rate is being set.
     * @param[in] fireRate The fire rate to select, in shots per second.
     * @param[in] command The agitator command to switch between discrete shots and constant
     *      rotation. Optional; when absent only the fire rate is changed.
     */
    SetFireRateCommand(
        tap::control::Subsystem *subsystem,
        src::control::agitator::ManualFireRateReselectionManager &fireRateReselectionManager,
        u_int8_t fireRate,
        std::optional<src::control::agitator::ConstantVelocityAgitatorCommand *> command =
            std::nullopt)
        : fireRateReselectionManager(fireRateReselectionManager),
          fireRate(fireRate),
          command(command)
    {
        addSubsystemRequirement(subsystem);
    }

    const char *getName() const override { return "Fire rate command"; }

    /**
     * Applies the fire rate and, if an agitator command was supplied, enables constant rotation
     * when the requested rate is high enough that discrete shots are no longer worthwhile.
     */
    void initialize() override
    {
        if (command.has_value())
        {
            if (fireRate > 30)  // TODO change for acctual
            {
                command.value()->enableConstantRotation(true);
            }
            else
            {
                command.value()->enableConstantRotation(false);
            }
        }
        fireRateReselectionManager.setFireRate(fireRate);
    }

    /// Does nothing; all of the work happens in `initialize`.
    void execute() override {}

    /// Does nothing; the selected fire rate persists after this command ends.
    void end([[maybe_unused]] bool interrupted) override {}

    /// @return Always `true`; this command completes in the iteration it is scheduled.
    bool isFinished() const { return true; }

private:
    /// The manager holding the operator's selected fire rate.
    src::control::agitator::ManualFireRateReselectionManager &fireRateReselectionManager;
    /// The fire rate this command selects, in shots per second.
    u_int8_t fireRate = 0;
    /// The agitator command toggled between discrete shots and constant rotation, if any.
    std::optional<src::control::agitator::ConstantVelocityAgitatorCommand *> command;
};
}  // namespace src::agitator

#endif
