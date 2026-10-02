#include "holonomic_chassis_subsystem.hpp"

#include <algorithm>
#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

#include "chassis_power.hpp"

using tap::algorithms::limitVal;

/*
    Chassis subsystem uses right hand rule, causing the following.
    +X: Forward
    +Y: Left
    +Rotation: CCW (headings, getChassisYaw, getChassisRotationSpeed, `rotational`)
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
    float translationSpeed =
        (chassisOdometry != nullptr)
            ? chassisOdometry->getVelocityLocal().getLength()
            : sqrtf(
                  rampControllers[0].getValue() * rampControllers[0].getValue() +
                  rampControllers[1].getValue() * rampControllers[1].getValue());

    // While spinning, every wheel eventually carries the whole translation, so reserve all of it.
    float availableWheelSpeed = std::max(rpmToMps(MAX_M3508_RPM_CHASSIS) - translationSpeed, 0.0f);

    return rotationBudgetFraction * availableWheelSpeed / DIST_TO_CENTER;
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
    // SUPERCAP: if (capacitorBank && capacitorBank->isEnabled()) the cap board measures the
    // chassis-side power directly; return that instead of the model below.
    PowerModel model;
    for (const Motor& motor : motors)
    {
        // Measured current, not the command.
        model.addMotor(motor.getTorque() * AMPS_DESIRED_OUTPUT_RATIO, motor.getEncoder()->getVelocity());
    }
    // Braking makes the mechanical term negative; the total draw can't be.
    return std::max(model.at(1.0f), 0.0f);
}

void HolonomicChassisSubsystem::resetDriveState()
{
    for (auto& ramp : rampControllers)
    {
        ramp.reset(0.0f);
    }
    desiredOutput.fill(0.0f);
    for (auto& pid : pidControllers)
    {
        pid.reset();
    }
    lastRotationalCommand = 0.0f;
    powerLimitScale = 1.0f;
}

void HolonomicChassisSubsystem::refreshSafeDisconnect()
{
    resetDriveState();
    for (auto& motor : motors)
    {
        motor.setDesiredOutput(0);
    }
}

void HolonomicChassisSubsystem::applyAccelerationToRamp(
    tap::algorithms::Ramp& ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt,
    bool allowAcceleration)
{
    bool accelerating = ramp.getTarget() * ramp.getValue() >= 0.0f &&
                        abs(ramp.getTarget()) > abs(ramp.getValue());
    if (!accelerating)
    {
        ramp.update(maxDeceleration * dt);
    }
    else if (allowAcceleration)
    {
        ramp.update(maxAcceleration * dt);
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
        resetDriveState();
        return;
    }
    lastRotationalCommand = rotational;

    float cos_theta = cos(heading);
    float sin_theta = sin(heading);

    float dynamicAccel = CHASSIS_ACCEL_VALUE;
    // Measured velocity in the request's frame, for ramp anti-windup. Assume it keeps up when
    // there is no odometry.
    float measuredForward = rampControllers[0].getValue();
    float measuredSideways = rampControllers[1].getValue();
    if (chassisOdometry != nullptr)
    {
        auto vel = chassisOdometry->getVelocityLocal();
        float speedFraction = limitVal<float>(vel.getLength() / MAX_CHASSIS_SPEED_MPS, 0.0f, 1.0f);
        dynamicAccel = CHASSIS_ACCEL_VALUE * (1.0f - ACCEL_TAPER_FACTOR * speedFraction);

        // Rotate by -heading: the inverse of the request-to-chassis rotation below.
        measuredForward = vel.x * cos_theta + vel.y * sin_theta;
        measuredSideways = -vel.x * sin_theta + vel.y * cos_theta;
    }

    const float dt = static_cast<float>(tap::Drivers::DT) / 1E3F;

    // While power limited, don't let a ramp run further ahead of the robot than the margin, so the
    // setpoint doesn't wind up and the robot stops promptly on release.
    bool limited = powerLimitScale < 1.0f;
    rampControllers[0].setTarget(forward);
    applyAccelerationToRamp(
        rampControllers[0],
        dynamicAccel,
        CHASSIS_DECCEL_VALUE,
        dt,
        !limited ||
            abs(rampControllers[0].getValue()) < abs(measuredForward) + RAMP_WINDUP_MARGIN_MPS);

    rampControllers[1].setTarget(sideways);
    applyAccelerationToRamp(
        rampControllers[1],
        dynamicAccel,
        CHASSIS_DECCEL_VALUE,
        dt,
        !limited ||
            abs(rampControllers[1].getValue()) < abs(measuredSideways) + RAMP_WINDUP_MARGIN_MPS);

    float measuredRotation = getChassisRotationSpeed();
    float maxRotationSpeed = MAX_CHASSIS_SPEED_MPS / DIST_TO_CENTER;
    float rotFraction = limitVal<float>(abs(measuredRotation) / maxRotationSpeed, 0.0f, 1.0f);
    float dynamicRotAccel =
        ROTATION_ACCEL_VALUE * (1.0f - ROTATION_ACCEL_TAPER_FACTOR * rotFraction);

    rampControllers[2].setTarget(rotational);
    applyAccelerationToRamp(
        rampControllers[2],
        dynamicRotAccel,
        ROTATION_ACCEL_VALUE,
        dt,
        !limited || abs(rampControllers[2].getValue()) <
                        abs(measuredRotation) + ROTATION_WINDUP_MARGIN_RADPS);

    float rampedXVelocity = rampControllers[0].getValue();
    float rampedYVelocity = rampControllers[1].getValue();
    float rampedRotational = rampControllers[2].getValue();

    float vx_local = rampedXVelocity * cos_theta - rampedYVelocity * sin_theta;
    float vy_local = rampedXVelocity * sin_theta + rampedYVelocity * cos_theta;

    setPeeking(abs(vy_local) > 0.1, vy_local > 0);

    // Positive wheel output pivots the chassis CW, so negate to make `rotational` CCW positive.
    // Wheel speed from rotation is w*R, R = DIST_TO_CENTER (center to wheel).
    float rotationalComponent = -rampedRotational * DIST_TO_CENTER;
    std::array<float, NUM_MOTORS> wheelSpeeds;
    wheelSpeeds[static_cast<int>(MotorId::LF)] =
        mpsToRpm((vx_local - vy_local) / M_SQRT2 + rotationalComponent);
    wheelSpeeds[static_cast<int>(MotorId::RF)] =
        mpsToRpm((-vx_local - vy_local) / M_SQRT2 + rotationalComponent);
    wheelSpeeds[static_cast<int>(MotorId::RB)] =
        mpsToRpm((-vx_local + vy_local) / M_SQRT2 + rotationalComponent);
    wheelSpeeds[static_cast<int>(MotorId::LB)] =
        mpsToRpm((vx_local + vy_local) / M_SQRT2 + rotationalComponent);

    // If any wheel asks for more than the motor can do, slow all four together so the direction
    // of motion is kept.
    float fastestWheel = 0.0f;
    for (float speed : wheelSpeeds)
    {
        fastestWheel = std::max(fastestWheel, abs(speed));
    }
    float scale = fastestWheel > MAX_M3508_RPM_CHASSIS ? MAX_M3508_RPM_CHASSIS / fastestWheel : 1.0f;
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        desiredOutput[i] = wheelSpeeds[i] * scale;
    }
}

// Debugger watch variables. Left non-static so the optimizer keeps them.
modm::Vector2f debugGlobalPose;
modm::Vector2f debugGlobalvelocity;
modm::Vector2f debugLocalvelocity;
float debugPowerLimitScale;
float debugRotationBudgetFraction;

void HolonomicChassisSubsystem::refresh()
{
    // Main loop period, in seconds.
    const float dt = static_cast<float>(tap::Drivers::DT) / 1E3F;

    PowerModel model;
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        pidControllers[i].update(
            desiredOutput[i] -
            motors[i].getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO);
        model.addMotor(
            pidControllers[i].getValue() * AMPS_DESIRED_OUTPUT_RATIO,
            motors[i].getEncoder()->getVelocity());
    }

    // Power loop: scale every output together so the modelled draw meets the target.
    // SUPERCAP: with a cap enabled, limit to the cap board's output budget (see
    // getChassisPowerTarget) and prefer its measured power over the model for the feedback.
    powerLimitScale = solvePowerScale(model, getChassisPowerTarget(drivers));
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].setDesiredOutput(pidControllers[i].getValue() * powerLimitScale);
    }

    // Translation priority: while power is limiting a rotating chassis, back beyblade's rotation
    // budget off so translation gets the power; let it recover once power stops limiting.
    if (powerLimitScale >= 1.0f)
    {
        rotationBudgetFraction += BEYBLADE_BUDGET_UP_RATE * dt;
    }
    else if (abs(lastRotationalCommand) > 1.0f)
    {
        rotationBudgetFraction -= BEYBLADE_BUDGET_DOWN_RATE * dt;
    }
    rotationBudgetFraction =
        limitVal<float>(rotationBudgetFraction, BEYBLADE_BUDGET_MIN_FRACTION, 1.0f);

    debugPowerLimitScale = powerLimitScale;
    debugRotationBudgetFraction = rotationBudgetFraction;

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
