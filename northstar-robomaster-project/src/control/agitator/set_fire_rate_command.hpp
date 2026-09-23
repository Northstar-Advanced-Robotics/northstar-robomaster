#ifndef SET_FIRE_RATE_COMMAND_HPP_
#define SET_FIRE_RATE_COMMAND_HPP_

#include <optional>

#include "tap/control/command.hpp"

#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"

namespace src::control::agitator
{
class SetFireRateCommand : public tap::control::Command
{
public:
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

    void execute() override {}

    void end([[maybe_unused]] bool interrupted) override {}

    bool isFinished() const { return true; }

private:
    src::control::agitator::ManualFireRateReselectionManager &fireRateReselectionManager;
    u_int8_t fireRate = 0;
    std::optional<src::control::agitator::ConstantVelocityAgitatorCommand *> command;
};
}  // namespace src::control::agitator

#endif
