#ifndef TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_
#define TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/dummy_subsystem.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"

namespace src::control::turret::cv
{
class TurretCVTargetingToggleCommand : public tap::control::Command
{
public:
    TurretCVTargetingToggleCommand(
        DummySubsystem *dummySubsystem,
        TurretCVControlCommand *turretCVControlCommand)
        : turretCVControlCommand(turretCVControlCommand)
    {
        TurretCVTargetingToggleCommand::addSubsystemRequirement(dummySubsystem);
    };

    const char *getName() const override { return "Turret CV Targeting Toggle"; }

    void initialize() override
    {
        turretCVControlCommand->setPitchOnlyMode(!turretCVControlCommand->getPitchOnlyMode());
    }

    void execute() override{};

    bool isFinished() const override { return true; }

    void end(bool interupted) override{};

private:
    TurretCVControlCommand *turretCVControlCommand;
};
}  // namespace src::control::turret::cv

#endif  // TURRET_CV_TARGETING_TOGGLE_COMMAND_HPP_
