#include "chassis_drive_to_point_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveToPointCommand::ChassisDriveToPointCommand(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry,
    float xPosition,
    float yPosition,
    float maxError)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      maxError(maxError)

{
    addSubsystemRequirement(chassis);
    targetPosition = modm::Vector<float, 2>(xPosition, yPosition);
}

void ChassisDriveToPointCommand::initialize() {}

void ChassisDriveToPointCommand::execute()
{
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
        velocityToTarget = dirToTarget.normalized();
    }

    chassis->setVelocityFieldDrive(velocityToTarget.y, velocityToTarget.x, 0);
}

void ChassisDriveToPointCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }

bool ChassisDriveToPointCommand::isFinished() const
{
    return chassisOdometry->getPositionGlobal().getDistanceTo(targetPosition) <= maxError;
}

};  // namespace src::chassis