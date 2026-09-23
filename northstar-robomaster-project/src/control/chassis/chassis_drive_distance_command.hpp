#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "chassis_subsystem.hpp"

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

class ChassisDriveDistanceCommand : public tap::control::Command
{
public:
    ChassisDriveDistanceCommand(
        ChassisSubsystem *chassis,
        src::control::chassis::ChassisOdometry *chassisOdometry,
        float xDist,
        float yDist,
        float maxError);

    const char *getName() const override { return "Chassis drive dist"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    static constexpr float MAXIMUM_MPS = 1.0f;
    static constexpr float MINIMUM_MPS = 0.38f;

    src::control::chassis::ChassisSubsystem *chassis;
    src::control::chassis::ChassisOdometry *chassisOdometry;

    modm::Vector<float, 2> targetPosition;
    float maxError;
};
}  // namespace src::control::chassis