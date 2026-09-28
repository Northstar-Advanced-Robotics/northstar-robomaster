#ifndef ODOMETRY_RESET_CMD_HPP
#define ODOMETRY_RESET_CMD_HPP

#include "tap/control/command.hpp"

#include "control/chassis/chassis_odometry.hpp"

#include "chassis_subsystem.hpp"

namespace src::chassis
{
class ChassisSubsystem;

/**
 * @ingroup chassis
 *
 * Zeroes the chassis odometry, making the robot's current pose the new origin.
 *
 * Bound to an operator input so accumulated dead-reckoning drift can be cleared by driving the
 * robot to a known spot on the field and resetting there. Finishes in a single iteration.
 */
class OdometryResetCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis, taken as a subsystem requirement so a drive command cannot
     *      be updating the pose while it is being reset.
     * @param[in] odometry The odometry to zero.
     */
    OdometryResetCommand(ChassisSubsystem *chassis, src::chassis::ChassisOdometry *odometry)
        : chassis(chassis),
          odometry(odometry)
    {
        addSubsystemRequirement(chassis);
    }

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Odometry Reset"; }

    /// Does nothing; the reset happens in `execute`.
    void initialize() {}

    /// Zeroes the odometry's accumulated pose.
    void execute() { odometry->zeroOdometry(); }

    /**
     * Does nothing.
     *
     * @param[in] interrupted Ignored.
     */
    void end(bool interrupted) {}

    /// @return Always `true`; the reset completes in the iteration it is scheduled.
    bool isFinished() const { return true; }

private:
    /// The chassis held as a requirement for the duration of the reset.
    src::chassis::ChassisSubsystem *chassis;
    /// The odometry being zeroed.
    src::chassis::ChassisOdometry *odometry;
};
}  // namespace src::chassis

#endif