#include "dji_two_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
DJITwoFlywheelSubsystem::DJITwoFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::can::CanBus canBus,
    bool inInverted)
    : TwoFlywheelSubsystem(drivers),
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
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      leftWheel(drivers, leftMotorId, canBus, inInverted, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, !inInverted, "Right Flywheel")
{
}

void DJITwoFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void DJITwoFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED_RPM);
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED_RPM);

    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight));
}

void DJITwoFlywheelSubsystem::setDesiredFlywheelSpeed(float rpm)
{
    desiredRpmRampLeft.setTarget(rpm);
    desiredRpmRampRight.setTarget(rpm);
}

float DJITwoFlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    return launchSpeedLinearInterpolator.interpolate(launchSpeed);
}
float debugWheelLeft = 0;
float debugWheelRight = 0;
float debugDesiredLeft = 0;
float debugDesiredRight = 0;
void DJITwoFlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;
    velocityPidLeftWheel.update(desiredRpmRampLeft.getValue() - getCurrentLeftFlywheelMotorRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(
        desiredRpmRampRight.getValue() - getCurrentRightFlywheelMotorRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));

    debugDesiredLeft = desiredRpmRampLeft.getValue();
    debugDesiredRight = desiredRpmRampRight.getValue();

    debugWheelLeft = getCurrentLeftFlywheelMotorRPM();
    debugWheelRight = getCurrentRightFlywheelMotorRPM();
}
}  // namespace src::control::flywheel
