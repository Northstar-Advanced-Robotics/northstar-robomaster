#include "chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::limitVal;

/*
    Chassis Subsystem uses a 2D coordinate system, using the ground as the XY plane
    +X: Right
    +Y: Forward
    +Rotation: CW
*/

namespace src::chassis
{
modm::Pair<int, float> ChassisSubsystem::lastComputedMaxWheelSpeed =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0];

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::control::turret::TurretMotor* yawMotor,
    ChassisOdometry* chassisOdometry_)
    : Subsystem(drivers),
      desiredOutput{},
      rampControllers{},
      pidControllers{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)},
      motors{
          Motor(drivers, config.leftFrontId, config.canBus, false, "LF", false, CHASSIS_GEAR_RATIO),
          Motor(drivers, config.leftBackId, config.canBus, false, "LB", false, CHASSIS_GEAR_RATIO),
          Motor(
              drivers,
              config.rightFrontId,
              config.canBus,
              false,
              "RF",
              false,
              CHASSIS_GEAR_RATIO),
          Motor(drivers, config.rightBackId, config.canBus, false, "RB", false, CHASSIS_GEAR_RATIO),
      },
      yawMotor(yawMotor),
      chassisOdometry(chassisOdometry_)
{
}

void ChassisSubsystem::initialize()
{
    for (auto& i : motors)
    {
        i.initialize();
    }
}
float LFSpeed;
float LBSpeed;
float RFSpeed;
float RBSpeed;

inline float ChassisSubsystem::getTurretYaw()
{
    return yawMotor->getChassisFrameMeasuredAngle().getWrappedValue();
}

float ChassisSubsystem::getChassisZeroTurret()
{
    float angle = (getTurretYaw());
    return (angle > M_PI) ? angle - M_TWOPI : angle;
}

float ChassisSubsystem::getChassisRotationSpeed()
{
    float motorSum = 0.0f;
    for (const Motor& i : motors)
    {
        motorSum += i.getEncoder()->getVelocity();
    }

    return (motorSum * WHEEL_DIAMETER_M / 2.0f) / (4 * DIST_TO_CENTER);
}

float ChassisSubsystem::calculateMaxRotationSpeed(float vert, float hor)
{
    float maxWheelSpeed = getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        ChassisSubsystem::getChassisPowerLimit(drivers));

    float linearSpeedRPM =
        (chassisOdometry != nullptr)
            ? mpsToRpm(chassisOdometry->getVelocityLocal().getLength())
            : mpsToRpm(sqrtf(
                  rampControllers[0].getValue() * rampControllers[0].getValue() +
                  rampControllers[1].getValue() * rampControllers[1].getValue()));

    float allowedWheelSpeed = maxWheelSpeed - linearSpeedRPM / 1.4142;

    if (allowedWheelSpeed < 0.0f)
    {
        allowedWheelSpeed = 0.0f;
    }
    return (allowedWheelSpeed * (CHASSIS_GEAR_RATIO) * (M_TWOPI / 60.0f) * (WHEEL_DIAMETER_M / 2)) /
           DIST_TO_CENTER;
}

void ChassisSubsystem::setVelocityTurretDrive(float forward, float sideways, float rotational)
{
    float turretRot = getTurretYaw();
    driveBasedOnHeading(forward, sideways, rotational, turretRot);
}

void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
{
    float robotHeading = fmod(getTurretYaw() - drivers->bmi088.getYaw(), 2 * M_PI);
    driveBasedOnHeading(forward, sideways, rotational, robotHeading);
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

float ChassisSubsystem::getChassisPowerDraw()
{
    float powerSum = 0.0f;
    for (size_t motor_idx = 0; motor_idx < motors.size(); motor_idx++)
    {
        powerSum += abs(
            (((float)motors[motor_idx].getOutputDesired() / DjiMotor::MAX_OUTPUT_C620) * 20.0f) *
            (((motors[motor_idx].getEncoder()->getVelocity() * 60.0f / M_TWOPI /
               CHASSIS_GEAR_RATIO) /
              MAX_M3508_RPM_CHASSIS) *
             24.0f));
    }
    return powerSum;
}

void ChassisSubsystem::driveBasedOnHeading(
    float forward,
    float sideways,
    float rotational,
    float heading)
{
    float maxWheelSpeed = getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        getChassisPowerLimit(drivers));

    float dynamicAccel = CHASSIS_ACCEL_VALUE;
    if (chassisOdometry != nullptr)
    {
        auto vel = chassisOdometry->getVelocityLocal();
        float currentSpeed = sqrtf(vel.x * vel.x + vel.y * vel.y);
        float maxSpeedMPS = maxWheelSpeed * (WHEEL_DIAMETER_M * M_PI / 60.0f * CHASSIS_GEAR_RATIO);
        float speedFraction = limitVal<float>(currentSpeed / maxSpeedMPS, 0.0f, 1.0f);
        dynamicAccel = CHASSIS_ACCEL_VALUE * (1.0f - ACCEL_TAPER_FACTOR * speedFraction);
    }

    float maxRotSpeed =
        (maxWheelSpeed * CHASSIS_GEAR_RATIO * M_TWOPI / 60.0f * (WHEEL_DIAMETER_M / 2.0f)) /
        DIST_TO_CENTER;
    float rotFraction = limitVal<float>(abs(getChassisRotationSpeed()) / maxRotSpeed, 0.0f, 1.0f);
    float dynamicRotAccel =
        ROTATION_ACCEL_VALUE * (1.0f - ROTATION_ACCEL_TAPER_FACTOR * rotFraction);

    rampControllers[0].setTarget(forward);

    ChassisSubsystem::applyAccelerationToRamp(
        rampControllers[0],
        dynamicAccel,
        CHASSIS_DECCEL_VALUE,
        static_cast<float>(tap::Drivers::DT) / 1E3F);

    rampControllers[1].setTarget(sideways);

    ChassisSubsystem::applyAccelerationToRamp(
        rampControllers[1],
        dynamicAccel,
        CHASSIS_DECCEL_VALUE,
        static_cast<float>(tap::Drivers::DT) / 1E3F);

    rampControllers[2].setTarget(rotational);

    ChassisSubsystem::applyAccelerationToRamp(
        rampControllers[2],
        dynamicRotAccel,
        ROTATION_ACCEL_VALUE,
        static_cast<float>(tap::Drivers::DT) / 1E3F);

    float rampedForward = rampControllers[0].getValue();
    float rampedSideways = rampControllers[1].getValue();
    float rampedRotational = rampControllers[2].getValue();

    float cos_theta = cos(heading);
    float sin_theta = sin(heading);

    float vx_local = rampedForward * cos_theta + rampedSideways * sin_theta;
    float vy_local = -rampedForward * sin_theta + rampedSideways * cos_theta;

    isPeeking = abs(vx_local) > 0.1;
    isPeekingLeft = isPeeking && (vx_local < 0);

    LFSpeed = mpsToRpm(
        (vx_local - vy_local) / M_SQRT2 +
        (rampedRotational)*DIST_TO_CENTER * M_SQRT2);  // Front-left wheel
    RFSpeed = mpsToRpm(
        (-vx_local - vy_local) / M_SQRT2 +
        (rampedRotational)*DIST_TO_CENTER * M_SQRT2);  // Front-right wheel
    RBSpeed = mpsToRpm(
        (-vx_local + vy_local) / M_SQRT2 +
        (rampedRotational)*DIST_TO_CENTER * M_SQRT2);  // Rear-right wheel
    LBSpeed = mpsToRpm(
        (vx_local + vy_local) / M_SQRT2 +
        (rampedRotational)*DIST_TO_CENTER * M_SQRT2);  // Rear-left wheel
    int LF = static_cast<int>(MotorId::LF);
    int LB = static_cast<int>(MotorId::LB);
    int RF = static_cast<int>(MotorId::RF);
    int RB = static_cast<int>(MotorId::RB);
    float calculatedMaxRPMPower = limitVal<float>(
        getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            ChassisSubsystem::getChassisPowerLimit(drivers)),
        -MAX_M3508_RPM_CHASSIS,
        MAX_M3508_RPM_CHASSIS);

    float sumSpeed = std::abs(LFSpeed) + std::abs(LBSpeed) + std::abs(RFSpeed) + std::abs(RBSpeed);
    float powerBudget;
    if (isBeyblading)
    {
        powerBudget = calculatedMaxRPMPower * 4.0f * BEYBLADE_SPEEDUP_FACTOR;
    }
    else
    {
        powerBudget = calculatedMaxRPMPower * 4.0f;
    }
    float scale = (sumSpeed > powerBudget && sumSpeed > 0.0f) ? powerBudget / sumSpeed : 1.0f;
    desiredOutput[LF] = LFSpeed * scale;
    desiredOutput[LB] = LBSpeed * scale;
    desiredOutput[RF] = RFSpeed * scale;
    desiredOutput[RB] = RBSpeed * scale;

    // desiredOutput[LF] = limitVal<float>(LFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    // desiredOutput[LB] = limitVal<float>(LBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    // desiredOutput[RF] = limitVal<float>(RFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    // desiredOutput[RB] = limitVal<float>(RBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
}

void ChassisSubsystem::refresh()
{
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput) {
        pid.update(
            desiredOutput -
            motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO);
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }

    if (chassisOdometry != nullptr)
    {
        chassisOdometry->updateOdometry(
            motors[static_cast<int>(MotorId::LF)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::LB)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::RF)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::RB)].getEncoder()->getVelocity());
    }
}
}  // namespace src::chassis