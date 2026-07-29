#ifndef ODOMETRY_RESET_CMD_HPP
#define ODOMETRY_RESET_CMD_HPP

#include "tap/control/command.hpp"

#include "control/chassis/chassis_odometry.hpp"

#include "chassis_subsystem.hpp"

namespace src::chassis
{
class ChassisSubsystem;

class OdometryResetCommand : public tap::control::Command
{
public:
    OdometryResetCommand(ChassisSubsystem *chassis, src::chassis::ChassisOdometry *odometry)
        : chassis(chassis),
          odometry(odometry)
    {
        addSubsystemRequirement(chassis);
    }

    const char *getName() const override { return "Odometry Reset"; }

    void initialize() {}

    void execute() { odometry->zeroOdometry(); }

    void end(bool) {}

    bool isFinished() const { return true; }

private:
    src::chassis::ChassisSubsystem *chassis;
    src::chassis::ChassisOdometry *odometry;
};
}  // namespace src::chassis

#endif