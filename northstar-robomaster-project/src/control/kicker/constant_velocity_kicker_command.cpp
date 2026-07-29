#include "constant_velocity_kicker_command.hpp"

#include "kicker_subsystem.hpp"

namespace src::control::kicker
{
ConstantVelocityKickerCommand::ConstantVelocityKickerCommand(
    src::kicker::KickerSubsystem* kicker,
    float velocitySetpoint)
    : tap::control::Command(),
      kicker(kicker),
      velocitySetpoint(velocitySetpoint)
{
    addSubsystemRequirement(kicker);
}

void ConstantVelocityKickerCommand::initialize() { kicker->setSetpoint(velocitySetpoint); }

void ConstantVelocityKickerCommand::execute() {}

void ConstantVelocityKickerCommand::end(bool) { kicker->setSetpoint(0); }

bool ConstantVelocityKickerCommand::isReady() { return true; }

bool ConstantVelocityKickerCommand::isFinished() const { return false; }

}  // namespace src::control::kicker