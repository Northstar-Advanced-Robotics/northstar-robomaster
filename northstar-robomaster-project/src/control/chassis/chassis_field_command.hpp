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

/**
 * @ingroup chassis
 *
 * Drives the chassis from operator input, with translation interpreted relative to the field
 * rather than to the robot.
 *
 * Pushing the stick forward always moves the robot the same direction down the field, regardless
 * of how the chassis or turret are oriented, which keeps driving intuitive when the operator loses
 * track of which way the robot is facing. Runs until interrupted, and stops the chassis when it
 * ends.
 */
class ChassisFieldCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis to drive, taken as a subsystem requirement.
     * @param[in] operatorInterface The source of the operator's translation and rotation input.
     */
    ChassisFieldCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Chassis tank drive"; }

    /// Does nothing; this command holds no state between iterations.
    void initialize() override {}

    /// Normalizes the operator's translation input and passes it, along with the requested
    /// rotation, to the chassis in field-relative terms.
    void execute() override;

    /**
     * Commands the chassis to a stop.
     *
     * @param[in] interrupted Ignored; the chassis is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return Always `false`; this command runs until something else interrupts it.
    bool isFinished() const { return false; }

private:
    /// The chassis being driven.
    src::chassis::ChassisSubsystem *chassis;

    /// The source of operator input.
    src::control::ControlOperatorInterface *operatorInterface;
};
}  // namespace src::chassis