#ifndef TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_
#define TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/dummy_subsystem.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"

namespace src::control::turret::cv
{
/**
 * @ingroup turret
 *
 * Toggles the CV control command between full auto-aim and pitch-only assist.
 *
 * In pitch-only mode vision handles the ballistic drop while the operator keeps control of yaw,
 * which is what you want when auto-aim is picking the wrong target or the operator wants to lead a
 * moving one. Requires only a `DummySubsystem`, since it changes a mode flag rather than driving
 * hardware, and so does not interrupt the turret command it is toggling. Finishes in a single
 * iteration.
 */
class TurretCVTargetingToggleCommand : public tap::control::Command
{
public:
    /**
     * @param[in] dummySubsystem Stands in as this command's subsystem requirement; nothing is
     *      actually reserved.
     * @param[in] turretCVControlCommand The CV control command whose mode is toggled.
     */
    TurretCVTargetingToggleCommand(
        DummySubsystem *dummySubsystem,
        TurretCVControlCommand *turretCVControlCommand)
        : turretCVControlCommand(turretCVControlCommand)
    {
        TurretCVTargetingToggleCommand::addSubsystemRequirement(dummySubsystem);
    };

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Turret CV Targeting Toggle"; }

    /// Flips the CV control command between full auto-aim and pitch-only assist.
    void initialize() override
    {
        turretCVControlCommand->setPitchOnlyMode(!turretCVControlCommand->getPitchOnlyMode());
    }

    /// Does nothing; the toggle happens in `initialize`.
    void execute() override{};

    /// @return Always `true`; the toggle completes in the iteration it is scheduled.
    bool isFinished() const override { return true; }

    /**
     * Does nothing; the toggled mode persists after this command ends.
     *
     * @param[in] interupted Ignored.
     */
    void end(bool interupted) override{};

private:
    /// The CV control command whose targeting mode is toggled.
    TurretCVControlCommand *turretCVControlCommand;
};
}  // namespace src::control::turret::cv

#endif  // TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_
