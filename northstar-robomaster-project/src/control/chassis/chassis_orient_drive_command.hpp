#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"

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
 * Drives the chassis from operator input while automatically rotating it to stay squared up with
 * the turret.
 *
 * Rotation is not taken from the operator; instead a PID controller drives the angle between the
 * chassis and the turret to zero, so the chassis follows the turret as the operator aims. Its
 * output is low-pass filtered, and the filter is opened up as the misalignment grows so that large
 * corrections are applied quickly while small ones stay smooth. Runs until interrupted, and stops
 * the chassis when it ends.
 */
class ChassisOrientDriveCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis to drive, taken as a subsystem requirement.
     * @param[in] operatorInterface The source of the operator's translation input.
     */
    ChassisOrientDriveCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Chassis tank drive"; }

    /// Seeds the filtered rotation with the chassis' current rotation speed, so taking over from
    /// another drive command does not jolt the chassis.
    void initialize() override;

    /// Runs the alignment PID, filters its output, and applies it together with the operator's
    /// turret-relative translation.
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

    /// The low-pass filtered rotational velocity carried between iterations, in radians/second.
    float rotationalValue;
};
}  // namespace src::chassis