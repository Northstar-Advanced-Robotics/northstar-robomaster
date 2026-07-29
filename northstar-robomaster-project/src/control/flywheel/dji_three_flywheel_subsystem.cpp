#include "dji_three_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
DJIThreeFlywheelSubsystem::DJIThreeFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::motor::MotorId downMotorId,
    tap::can::CanBus canBus)
    : ThreeFlywheelSubsystem(drivers, SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
      velocityPidLeftWheel(
          FLYWHEEL_PID_KP_DJI,
          FLYWHEEL_PID_KI_DJI,
          FLYWHEEL_PID_KD_DJI,
          FLYWHEEL_PID_MAX_ERROR_SUM_DJI,
          FLYWHEEL_PID_MAX_OUTPUT_DJI),
      velocityPidRightWheel(
          FLYWHEEL_PID_KP_DJI,
          FLYWHEEL_PID_KI_DJI,
          FLYWHEEL_PID_KD_DJI,
          FLYWHEEL_PID_MAX_ERROR_SUM_DJI,
          FLYWHEEL_PID_MAX_OUTPUT_DJI),
      velocityPidDownWheel(
          FLYWHEEL_PID_KP_DJI,
          FLYWHEEL_PID_KI_DJI,
          FLYWHEEL_PID_KD_DJI,
          FLYWHEEL_PID_MAX_ERROR_SUM_DJI,
          FLYWHEEL_PID_MAX_OUTPUT_DJI),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedDown(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampDown(0),
      leftWheel(drivers, leftMotorId, canBus, true, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      downWheel(drivers, downMotorId, canBus, true, "Down Flywheel")
{
}

void DJIThreeFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    downWheel.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void DJIThreeFlywheelSubsystem::setDesiredSpin(u_int16_t spin)
{
    if (auto spinSet = toSpinPreset(spin))
    {
        desiredSpin = spinSet.value();
        desiredSpinValue = spin;
    }
}

void DJIThreeFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED_RPM);
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED_RPM);
    desiredLaunchSpeedDown = limitVal(
        speed * (desiredSpinValue / 100.0f),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM);  // uses spin

    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight));
    desiredRpmRampDown.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedDown));
}

void DJIThreeFlywheelSubsystem::setDesiredFlywheelSpeed(float rpm)
{
    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(rpm));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(rpm));
    desiredRpmRampDown.setTarget(launchSpeedToFlywheelRpm(rpm * (desiredSpinValue / 100.0f)));
}

float DJIThreeFlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        static_cast<uint8_t>(spinToRPMMap.at(desiredSpin).size())};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
}
float debugWheelLeft2 = 0;
float debugWheelRight2 = 0;
float debugWheelDown2 = 0;
float debugDesiredLeft2 = 0;
float debugDesiredRight2 = 0;
float debugDesiredDown2 = 0;
void DJIThreeFlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampDown.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;
    velocityPidLeftWheel.update(desiredRpmRampLeft.getValue() - getWheelRPM(&leftWheel));
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRampRight.getValue() - getWheelRPM(&rightWheel));
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    velocityPidDownWheel.update(desiredRpmRampDown.getValue() - getWheelRPM(&downWheel));
    downWheel.setDesiredOutput(static_cast<int32_t>(velocityPidDownWheel.getValue()));
    debugDesiredLeft2 = desiredRpmRampLeft.getValue();
    debugDesiredRight2 = desiredRpmRampRight.getValue();
    debugDesiredDown2 = desiredRpmRampDown.getValue();
    debugWheelLeft2 = getWheelRPM(&leftWheel);
    debugWheelRight2 = getWheelRPM(&rightWheel);
    debugWheelDown2 = getWheelRPM(&downWheel);
}
}  // namespace src::control::flywheel
