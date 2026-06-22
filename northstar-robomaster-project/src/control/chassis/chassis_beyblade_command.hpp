#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

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

class ChassisBeybladeCommand : public tap::control::Command
{
public:
    ChassisBeybladeCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface,
        short direction,
        bool isVariable);

    const char *getName() const override { return "Chassis beyblade drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;

    src::control::ControlOperatorInterface *operatorInterface;

    uint32_t prevTime;

    uint32_t accumTime;

    float distScaleFactor;

    short direction;

    bool isVariable;

    float calcSpeed;

    float beyBladeFastSpinSpeedThreshold = 0.1f;

    float calculateBeyBladeRotationSpeed(float maxSpeed, uint32_t dt);
};
}  // namespace src::chassis