#include "chassis_drive_distance_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveDistanceCommand::ChassisDriveDistanceCommand(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry,
    float xDist,
    float yDist,
    float maxError)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      maxError(maxError)

{
    addSubsystemRequirement(chassis);

    targetPosition =
        chassisOdometry->getPositionGlobal() + chassisOdometry->convertLocalToGlobal(
                                                   modm::Vector<float, 2>(xDist, yDist),
                                                   chassisOdometry->getRotation());
}

void ChassisDriveDistanceCommand::initialize() {}

void ChassisDriveDistanceCommand::execute()
{
    auto scale = [](float raw) -> float { return limitVal(raw, -1.0f, 1.0f) * 0.5f; };
    modm::Vector<float, 2> dirToTarget = (targetPosition - chassisOdometry->getPositionGlobal());

    float distanceToTarget = dirToTarget.getLength();
    modm::Vector<float, 2> velocityToTarget;

    if (distanceToTarget > MAXIMUM_MPS)
    {
        velocityToTarget = dirToTarget.normalized() * MAXIMUM_MPS;
    }
    else if (distanceToTarget < MINIMUM_MPS)
    {
        velocityToTarget = dirToTarget.normalized() * MINIMUM_MPS;
    }
    else
    {
        velocityToTarget = dirToTarget;
    }

    chassis->setVelocityFieldDrive(velocityToTarget.y, velocityToTarget.x, 0);
}

void ChassisDriveDistanceCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}

bool ChassisDriveDistanceCommand::isFinished() const
{
    return chassisOdometry->getPositionGlobal().getDistanceTo(targetPosition) <= maxError;
}

};  // namespace src::chassis