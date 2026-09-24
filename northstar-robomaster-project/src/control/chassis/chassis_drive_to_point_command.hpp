#pragma once

#include "tap/control/command.hpp"

#include "modm/math/filter/pid.hpp"

#include "chassis_subsystem.hpp"

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
 * Drives the chassis to an absolute position on the field, using odometry to decide when it has
 * arrived.
 *
 * The target is a fixed field-frame point, so the command is only as accurate as the odometry's
 * current estimate of where the robot is; see `ChassisDriveDistanceCommand` for the relative
 * equivalent. Used in autonomous routines. Finishes once the robot is within `maxError` of the
 * target.
 */
class ChassisDriveToPointCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis to drive, taken as a subsystem requirement.
     * @param[in] chassisOdometry The odometry used to track progress toward the target.
     * @param[in] xPosition The target's field-frame x coordinate, in meters.
     * @param[in] yPosition The target's field-frame y coordinate, in meters.
     * @param[in] maxError How close the robot must get to the target before the command finishes,
     *      in meters.
     */
    ChassisDriveToPointCommand(
        ChassisSubsystem *chassis,
        src::chassis::ChassisOdometry *chassisOdometry,
        float xPosition,
        float yPosition,
        float maxError);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Chassis drive to point"; }

    /// Does nothing; the target position is fixed at construction.
    void initialize() override;

    /// Drives toward the target, scaling speed with the remaining distance but keeping it between
    /// `MINIMUM_MPS` and `MAXIMUM_MPS`.
    void execute() override;

    /**
     * Commands the chassis to a stop.
     *
     * @param[in] interrupted Ignored; the chassis is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return `true` once the robot is within `maxError` of the target position.
    bool isFinished() const override;

private:
    /// Fastest the chassis will be driven toward the target, in meters/second.
    static constexpr float MAXIMUM_MPS = 1.0f;
    /// Slowest the chassis will be driven toward the target, in meters/second. A floor keeps the
    /// robot from stalling against friction as it closes in.
    static constexpr float MINIMUM_MPS = 0.38f;

    /// The chassis being driven.
    src::chassis::ChassisSubsystem *chassis;
    /// The odometry supplying the robot's position.
    src::chassis::ChassisOdometry *chassisOdometry;

    /// The field-frame position being driven to, in meters.
    modm::Vector<float, 2> targetPosition;
    /// How close to `targetPosition` counts as arrived, in meters.
    float maxError;
};
}  // namespace src::chassis