#include "turret_motor.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"

using tap::algorithms::Angle;
using tap::algorithms::limitVal;
using tap::algorithms::WrappedFloat;

namespace src::control::turret
{
TurretMotor::TurretMotor(
    tap::motor::MotorInterface *motor,
    const TurretMotorConfig &motorConfig,
    const tap::encoder::EncoderInterface *velocityEncoder)
    : config(motorConfig),
      motor(motor),
      velocityEncoder(velocityEncoder != nullptr ? velocityEncoder : motor->getEncoder()),
      chassisFrameSetpoint(Angle(config.startAngle)),
      chassisFrameMeasuredAngle(Angle(config.startAngle))
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        float chassisFrameUnwrappedMeasurement =
            motor->getEncoder()->getPosition().getUnwrappedValue();

        chassisFrameMeasuredAngle.setUnwrappedValue(chassisFrameUnwrappedMeasurement * config.ratio);
    }
    else
    {
        chassisFrameMeasuredAngle.setUnwrappedValue(config.startAngle);
    }
}

void TurretMotor::setMotorOutput(float out)
{
    out = limitVal(out, -config.maxOutput, config.maxOutput);

    if (motor->isMotorOnline())
    {
        motor->setDesiredOutput(out);
    }
    else
    {
        motor->setDesiredOutput(0);
    }
}

void TurretMotor::setChassisFrameSetpoint(WrappedFloat setpoint)
{
    chassisFrameSetpoint = setpoint;

    if (config.limitMotorAngles)
    {
        int status;
        chassisFrameSetpoint = Angle(WrappedFloat::limitValue(
            chassisFrameSetpoint,
            config.minAngle,
            config.maxAngle,
            &status));
    }
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

float TurretMotor::getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
    const
{
    if (config.limitMotorAngles)
    {
        float pos = WrappedFloat::rangeOverlap(
            measurement,
            setpoint,
            Angle(config.maxAngle),
            Angle(config.minAngle));
        float neg = WrappedFloat::rangeOverlap(
            setpoint,
            measurement,
            Angle(config.maxAngle),
            Angle(config.minAngle));

        if (pos < neg)
        {
            return (setpoint - measurement).getWrappedValue();
        }
        else if (pos > neg)
        {
            return (setpoint - measurement).getWrappedValue() - static_cast<float>(M_TWOPI);
        }
    }

    // the error can be wrapped around the unit circle
    // equivalent to this - other
    return measurement.minDifference(setpoint);
}

}  // namespace src::control::turret
