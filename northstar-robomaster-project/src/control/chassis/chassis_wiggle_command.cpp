#include "chassis_wiggle_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisWiggleCommand::ChassisWiggleCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface,
    float period,
    float maxWiggleSpeed)
    : chassis(chassis),
      operatorInterface(operatorInterface),
      period(period),
      maxWiggleSpeed(maxWiggleSpeed)
{
    addSubsystemRequirement(chassis);
}

void ChassisWiggleCommand::initialize() { prevTime = tap::arch::clock::getTimeMilliseconds(); }

void ChassisWiggleCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };
    modm::Pair<float, float> normInput = getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    chassis->setVelocityTurretDrive(
        scale(normInput.first),
        -scale(normInput.second),
        calculateWiggle(dt));
}

void ChassisWiggleCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}

float ChassisWiggleCommand::calculateWiggle(uint32_t dt)
{
    if (dt > 50)
    {
        accumTime = 0;
        return 0;
    }
    accumTime += dt;
    return maxWiggleSpeed * sin((1 / period) * ((float)accumTime / 1000.0f) * M_TWOPI);
}
};  // namespace src::chassis