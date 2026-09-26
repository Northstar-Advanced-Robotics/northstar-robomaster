#include "holonomic_chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

#include "chassis_power.hpp"

using tap::algorithms::limitVal;
using tap::motor::DjiMotor;

/*
    Chassis subsystem uses right hand rule, causing the following.
    +X: Forward
    +Y: Left
    +Rotation: CCW (headings, getChassisYaw, getChassisRotationSpeed)

    Exception: the `rotational` argument of the drive methods is CW positive. See the class
    documentation in chassis_subsystem.hpp.
*/

namespace src::control::chassis
{
HolonomicChassisSubsystem::HolonomicChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::control::turret::TurretMotor* yawMotor,
    ChassisOdometry* chassisOdometry)
    : ChassisSubsystem(drivers, yawMotor, chassisOdometry),
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
      }
{
    for (auto& pid : pidControllers)
    {
        pid.setParameter(config.wheelVelocityPidConfig);
    }
}

void HolonomicChassisSubsystem::initialize()
{
    for (auto& motor : motors)
    {
        motor.initialize();
    }
}

float HolonomicChassisSubsystem::getChassisRotationSpeed()
{
    float motorSum = 0.0f;
    for (const Motor& motor : motors)
    {
        motorSum += motor.getEncoder()->getVelocity();
    }

    return -(motorSum * WHEEL_DIAMETER_M / 2.0f) / (4 * DIST_TO_CENTER);
}

float HolonomicChassisSubsystem::calculateMaxRotationSpeed()
{
    float maxWheelSpeed =
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassisPowerLimit(drivers));

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

void HolonomicChassisSubsystem::setVelocityTurretDrive(
    float forward,
    float sideways,
    float rotational)
{
    driveBasedOnHeading(forward, sideways, rotational, getTurretYaw());
}

void HolonomicChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
{
    driveBasedOnHeading(forward, sideways, rotational, -getChassisYaw());
}

float HolonomicChassisSubsystem::getChassisPowerDraw()
{
    float powerSum = 0.0f;
    for (const Motor& motor : motors)
    {
        powerSum += abs(
            (((float)motor.getOutputDesired() / DjiMotor::MAX_OUTPUT_C620) * 20.0f) *
            (((motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO) /
              MAX_M3508_RPM_CHASSIS) *
             24.0f));
    }
    return powerSum;
}

void HolonomicChassisSubsystem::applyAccelerationToRamp(
    tap::algorithms::Ramp& ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt)
{
    if (tap::algorithms::getSign(ramp.getTarget()) == tap::algorithms::getSign(ramp.getValue()) &&
        abs(ramp.getTarget()) > abs(ramp.getValue()))
    {
        // we are trying to speed up
        ramp.update(maxAcceleration * dt);
    }
    else
    {
        // we are trying to slow down
        ramp.update(maxDeceleration * dt);
    }
}

void HolonomicChassisSubsystem::driveBasedOnHeading(
    float forward,
    float sideways,
    float rotational,
    float heading)
{
    if (!motors[0].isMotorOnline() && !motors[1].isMotorOnline() && !motors[2].isMotorOnline() &&
        !motors[3].isMotorOnline())
    {
        forward = 0;
        sideways = 0;
        rotational = 0;
    }

    float maxWheelSpeed =
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassisPowerLimit(drivers));

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

    const float dt = static_cast<float>(tap::Drivers::DT) / 1E3F;

    rampControllers[0].setTarget(forward);
    applyAccelerationToRamp(rampControllers[0], dynamicAccel, CHASSIS_DECCEL_VALUE, dt);

    rampControllers[1].setTarget(sideways);
    applyAccelerationToRamp(rampControllers[1], dynamicAccel, CHASSIS_DECCEL_VALUE, dt);

    rampControllers[2].setTarget(rotational);
    applyAccelerationToRamp(rampControllers[2], dynamicRotAccel, ROTATION_ACCEL_VALUE, dt);

    float rampedXVelocity = rampControllers[0].getValue();
    float rampedYVelocity = rampControllers[1].getValue();
    float rampedRotational = rampControllers[2].getValue();

    float cos_theta = cos(heading);
    float sin_theta = sin(heading);

    float vx_local = rampedXVelocity * cos_theta - rampedYVelocity * sin_theta;
    float vy_local = rampedXVelocity * sin_theta + rampedYVelocity * cos_theta;

    setPeeking(abs(vy_local) > 0.1, vy_local > 0);

    float rotationalComponent = rampedRotational * DIST_TO_CENTER * M_SQRT2;
    float LFSpeed = mpsToRpm((vx_local - vy_local) / M_SQRT2 + rotationalComponent);
    float RFSpeed = mpsToRpm((-vx_local - vy_local) / M_SQRT2 + rotationalComponent);
    float RBSpeed = mpsToRpm((-vx_local + vy_local) / M_SQRT2 + rotationalComponent);
    float LBSpeed = mpsToRpm((vx_local + vy_local) / M_SQRT2 + rotationalComponent);

    float calculatedMaxRPMPower =
        limitVal<float>(maxWheelSpeed, -MAX_M3508_RPM_CHASSIS, MAX_M3508_RPM_CHASSIS);

    float sumSpeed = std::abs(LFSpeed) + std::abs(LBSpeed) + std::abs(RFSpeed) + std::abs(RBSpeed);
    float powerBudget = calculatedMaxRPMPower * 4.0f;
    if (isBeybladingOnly())
    {
        powerBudget *= BEYBLADE_SPEEDUP_FACTOR;
    }
    float scale = (sumSpeed > powerBudget && sumSpeed > 0.0f) ? powerBudget / sumSpeed : 1.0f;
    desiredOutput[static_cast<int>(MotorId::LF)] = LFSpeed * scale;
    desiredOutput[static_cast<int>(MotorId::LB)] = LBSpeed * scale;
    desiredOutput[static_cast<int>(MotorId::RF)] = RFSpeed * scale;
    desiredOutput[static_cast<int>(MotorId::RB)] = RBSpeed * scale;
}

// Debugger watch variables. Left non-static so the optimizer keeps them.
modm::Vector2f debugGlobalPose;
modm::Vector2f debugGlobalvelocity;
modm::Vector2f debugLocalvelocity;

void HolonomicChassisSubsystem::refresh()
{
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        pidControllers[i].update(
            desiredOutput[i] -
            motors[i].getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO);
        motors[i].setDesiredOutput(pidControllers[i].getValue());
    }

    if (chassisOdometry != nullptr)
    {
        chassisOdometry->updateOdometry(
            motors[static_cast<int>(MotorId::LF)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::LB)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::RF)].getEncoder()->getVelocity(),
            motors[static_cast<int>(MotorId::RB)].getEncoder()->getVelocity());

        debugGlobalPose = chassisOdometry->getPositionGlobal();
        debugGlobalvelocity = chassisOdometry->getVelocityGlobal();
        debugLocalvelocity = chassisOdometry->getVelocityLocal();
    }
}
}  // namespace src::control::chassis
