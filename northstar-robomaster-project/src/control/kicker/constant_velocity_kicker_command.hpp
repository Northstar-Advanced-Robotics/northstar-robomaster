#ifndef CONSTANT_VELOCITY_KICKER_COMMAND_HPP
#define CONSTANT_VELOCITY_KICKER_COMMAND_HPP

#include "tap/control/command.hpp"

#include "kicker_subsystem.hpp"

namespace src::control::kicker
{
class ConstantVelocityKickerCommand : public tap::control::Command
{
public:
    ConstantVelocityKickerCommand(
        src::control::kicker::KickerSubsystem* kicker,
        float velocitySetpoint);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "ConstantVelocityKickerCommand"; }

private:
    src::control::kicker::KickerSubsystem* kicker;

    float velocitySetpoint;
};
}  // namespace src::control::kicker

#endif  // CONSTANT_VELOCITY_KICKER_COMMAND_HPP