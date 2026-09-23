#ifndef ODOMETRY_RESET_CMD_HPP
#define ODOMETRY_RESET_CMD_HPP

#include "tap/control/command.hpp"

#include "control/chassis/chassis_odometry.hpp"

#include "chassis_subsystem.hpp"

namespace src::control::chassis
{
class ChassisSubsystem;

class OdometryResetCommand : public tap::control::Command
{
public:
    OdometryResetCommand(
        ChassisSubsystem *chassis,
        src::control::chassis::ChassisOdometry *odometry)
        : chassis(chassis),
          odometry(odometry)
    {
        addSubsystemRequirement(chassis);
    }

    const char *getName() const override { return "Odometry Reset"; }

    void initialize() {}

    void execute() { odometry->zeroOdometry(); }

    void end(bool interrupted) {}

    bool isFinished() const { return true; }

private:
    src::control::chassis::ChassisSubsystem *chassis;
    src::control::chassis::ChassisOdometry *odometry;
};
}  // namespace src::control::chassis

#endif