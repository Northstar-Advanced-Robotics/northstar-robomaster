#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveCommand::ChassisDriveCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
}

void ChassisDriveCommand::execute()
{
    modm::Pair<float, float> normInput = getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    chassis->setVelocityTurretDrive(
        normInput.first,
        -normInput.second,
        operatorInterface->getDrivetrainRotationalTranslation());
}

void ChassisDriveCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}
};  // namespace src::chassis