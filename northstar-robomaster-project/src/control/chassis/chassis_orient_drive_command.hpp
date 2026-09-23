#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"

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

class ChassisOrientDriveCommand : public tap::control::Command
{
public:
    ChassisOrientDriveCommand(
        ChassisSubsystem *chassis,
        src::robot::ControlOperatorInterface *operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::control::chassis::ChassisSubsystem *chassis;

    src::robot::ControlOperatorInterface *operatorInterface;

    float rotationalValue;
};
}  // namespace src::control::chassis