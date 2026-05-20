#pragma once

#include "tap/control/command.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::chassis
{
class ChassisSubsystem;

class ChassisSprintCommand : public tap::control::Command
{
public:
    ChassisSprintCommand(ChassisSubsystem *chassis);

    const char *getName() const override { return "Chassis sprint"; }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;
};
}  // namespace src::chassis