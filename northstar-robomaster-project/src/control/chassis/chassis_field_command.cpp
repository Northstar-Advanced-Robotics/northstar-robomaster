#include "chassis_field_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

namespace src::control::chassis
{
ChassisFieldCommand::ChassisFieldCommand(
    ChassisSubsystem* chassis,
    src::robot::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
}

void ChassisFieldCommand::execute()
{
    modm::Pair<float, float> normInput = getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    chassis->setVelocityFieldDrive(
        normInput.first,
        normInput.second,
        operatorInterface->getDrivetrainRotationalTranslation());
}

void ChassisFieldCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}
}  // namespace src::control::chassis