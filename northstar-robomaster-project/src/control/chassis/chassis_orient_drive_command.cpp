#include "chassis_orient_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisOrientDriveCommand::ChassisOrientDriveCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
}

void ChassisOrientDriveCommand::initialize()
{
    rotationalValue = chassis->getChassisRotationSpeed();
}

void ChassisOrientDriveCommand::execute()
{
    float rotationOffset = tap::algorithms::WrappedFloat(
                               chassis->getChassisZeroTurret(),
                               -M_TWOPI,
                               M_TWOPI)  // M_PI_4 for 90 sides
                               .getWrappedValue();
    float rotationFromPID = chassis->chassisSpeedRotationPID(rotationOffset);

    float rotationalAlpha = std::max<float>(1.0f - abs(rotationOffset) / M_PI, AUTO_ROTATION_ALPHA);

    rotationalValue =
        tap::algorithms::lowPassFilter(rotationalValue, rotationFromPID, rotationalAlpha);

    modm::Pair<float, float> normInput = src::chassis::getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    chassis->setVelocityTurretDrive(normInput.first, -normInput.second, rotationalValue);
}

void ChassisOrientDriveCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}
};  // namespace src::chassis