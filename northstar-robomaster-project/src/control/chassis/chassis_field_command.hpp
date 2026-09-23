#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

namespace src
{
class Drivers;
}  // namespace src

namespace src::robot
{
class ControlOperatorInterface;
}  // namespace src::robot

namespace src::control::chassis
{
class ChassisSubsystem;

class ChassisFieldCommand : public tap::control::Command
{
public:
    ChassisFieldCommand(
        ChassisSubsystem *chassis,
        src::robot::ControlOperatorInterface *operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::control::chassis::ChassisSubsystem *chassis;

    src::robot::ControlOperatorInterface *operatorInterface;
};
}  // namespace src::control::chassis