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

class ChassisWiggleCommand : public tap::control::Command
{
public:
    ChassisWiggleCommand(
        ChassisSubsystem *chassis,
        src::robot::ControlOperatorInterface *operatorInterface,
        float period,
        float maxWiggleSpeed);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::control::chassis::ChassisSubsystem *chassis;

    src::robot::ControlOperatorInterface *operatorInterface;

    uint32_t prevTime;

    uint32_t accumTime;

    float period;

    float maxWiggleSpeed;

    float calculateWiggle(uint32_t dt);
};
}  // namespace src::control::chassis