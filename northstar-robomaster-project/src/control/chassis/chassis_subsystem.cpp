#include "chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

#include "control/turret/turret_motor.hpp"

using tap::algorithms::limitVal;

namespace src::control::chassis
{
float ChassisSubsystem::getTurretYaw()
{
    return yawMotor->getChassisFrameMeasuredAngle().getWrappedValue();
}

float ChassisSubsystem::chassisSpeedRotationPID(float angleOffset)
{
    // Deadzone logic, make deadzone area a constant eventaully.
    if (abs(angleOffset) < modm::toRadian(3.0f))
    {
        return 0.0f;
    }
    // P
    float currRotationPidP = angleOffset * CHASSIS_ROTATION_P;  // P

    // D
    float currentRotationPidD = -drivers->bmi088.getGz() * CHASSIS_ROTATION_D;  // D

    float chassisRotationSpeed = limitVal<float>(
        currRotationPidP + currentRotationPidD,
        -CHASSIS_ROTATION_MAX_VEL,
        CHASSIS_ROTATION_MAX_VEL);

    return chassisRotationSpeed;
}

float ChassisSubsystem::chassisSpeedRotationAutoDrivePID(float angleOffset)
{
    // P
    float currentRotationPidP = angleOffset * 5;  // P

    // D
    float currentRotationPidD = -getChassisRotationSpeed() * 0.05f;  // D

    float chassisRotationSpeed = limitVal<float>(
        currentRotationPidP + currentRotationPidD,
        -CHASSIS_ROTATION_MAX_VEL,
        CHASSIS_ROTATION_MAX_VEL);

    return chassisRotationSpeed;
}
}  // namespace src::control::chassis
