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

class ChassisBeybladeCommand : public tap::control::Command
{
public:
    ChassisBeybladeCommand(
        ChassisSubsystem *chassis,
        src::robot::ControlOperatorInterface *operatorInterface,
        short direction,
        bool isVariable);

    const char *getName() const override { return "Chassis beyblade drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

    float calculateBeyBladeRotationSpeed(float maxSpeed, uint32_t dt);

private:
    src::control::chassis::ChassisSubsystem *chassis;

    src::robot::ControlOperatorInterface *operatorInterface;

    uint32_t prevTime;

    uint32_t accumTime;

    float distScaleFactor;

    short direction;

    bool isVariable;

    float calcSpeed;

    float beyBladeFastSpinSpeedThreshold = 0.3f;
};
}  // namespace src::control::chassis