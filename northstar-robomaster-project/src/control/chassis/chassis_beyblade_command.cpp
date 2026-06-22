#include "chassis_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface,
    short direction,
    bool isVariable)
    : chassis(chassis),
      operatorInterface(operatorInterface),
      direction(direction),
      isVariable(isVariable)
{
    addSubsystemRequirement(chassis);
}

void ChassisBeybladeCommand::initialize()
{
    prevTime = tap::arch::clock::getTimeMilliseconds();
    calcSpeed = 1.0f * direction;
    chassis->isBeyblading = true;
}

float calcedRot;

void ChassisBeybladeCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    modm::Pair<float, float> normInput = getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    float verticalSpeed = normInput.first;
    float horizontalSpeed = normInput.second;
    calcedRot = calculateBeyBladeRotationSpeed(
        chassis->calculateMaxRotationSpeed(verticalSpeed, horizontalSpeed),
        dt);
    if (powf(verticalSpeed, 2) + powf(horizontalSpeed, 2) < beyBladeFastSpinSpeedThreshold)
    {
        calcedRot *= BEYBLADE_SPEEDUP_FACTOR;
    }
    chassis->setVelocityTurretDrive(verticalSpeed, -horizontalSpeed, calcedRot);
}

void ChassisBeybladeCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
    chassis->isBeyblading = false;
}

float ChassisBeybladeCommand::calculateBeyBladeRotationSpeed(float maxSpeed, uint32_t dt)
{
    accumTime += dt;
    if (!isVariable)
    {
        return maxSpeed * direction;
    }
    RandomNumberGenerator::enable();

    if (accumTime > 500)
    {
        calcSpeed = limitVal<float>(1.0f, 0.0f, 0.9f) * direction;
        if (RandomNumberGenerator::isReady())
        {
            calcSpeed = limitVal<float>(
                0.1f * static_cast<float>(sin(RandomNumberGenerator::getValue())) + calcSpeed,
                -1.0f,
                1.0f);
        }
        accumTime = 0;
    }
    return calcSpeed * maxSpeed * direction;
}
};  // namespace src::chassis