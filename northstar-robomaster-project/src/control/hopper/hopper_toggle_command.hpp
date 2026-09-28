#ifndef HOPPER_TOGGLE_COMMAND_HPP_
#define HOPPER_TOGGLE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/hopper/hopper_subsystem.hpp"

namespace src::control::hopper
{
/**
 * @ingroup hopper_kicker
 *
 * Holds the hopper lid open for as long as it is scheduled, closing it again when it ends.
 *
 * Bound to a held input, so the lid stays open only while the operator asks for it and cannot be
 * left open by accident.
 */
class HopperToggleCommand : public tap::control::Command
{
public:
    /**
     * @param[in] hopper The hopper to drive, taken as a subsystem requirement.
     */
    HopperToggleCommand(HopperSubsystem* hopper);

    /// Opens the lid.
    void initialize() override;

    /// Does nothing; the subsystem holds the lid at the position set in `initialize`.
    void execute() override;

    /// Closes the lid, whether this command finished or was interrupted.
    void end(bool) override;

    /// @return Always `true`; the lid can be opened at any time.
    bool isReady() override;

    /// @return Always `false`; the lid stays open until this command is interrupted.
    bool isFinished() const override;

    /// @return The name used to identify this command in logs and the scheduler.
    const char* getName() const override { return "hopper toggle"; }

private:
    /// The hopper being driven.
    HopperSubsystem* hopper;

    /// Time in milliseconds at which the lid was opened.
    uint32_t startTime = 0;
};

}  // namespace src::control::hopper

#endif  // HERO_AGITATOR_SHOOT_COMMAND_HPP_
