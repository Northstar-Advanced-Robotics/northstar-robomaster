#ifndef DJI_TWO_FLYWHEEL_SUBSYSTEM_HPP_
#define DJI_TWO_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class DJITwoFlywheelSubsystem : public TwoFlywheelSubsystem
{
public:
    DJITwoFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        bool inInverted = false);

    void initialize() override;

    void setDesiredLaunchSpeed(float speed) override;

    void setDesiredFlywheelSpeed(float rpm) override;

    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }

    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight) / 2.0f;
    }

    float getDesiredFlywheelSpeedLeft() const { return desiredRpmRampLeft.getTarget(); }
    float getDesiredFlywheelSpeedRight() const { return desiredRpmRampRight.getTarget(); }

    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight()) / 2.0f;
    }

    float getCurrentLeftFlywheelMotorRPM() const
    {
        return leftWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentRightFlywheelMotorRPM() const
    {
        return rightWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM()) / 2.0f;
    }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);
        rightWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Flywheels"; }

private:
    modm::Pid<float> velocityPidLeftWheel;
    modm::Pid<float> velocityPidRightWheel;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const override;
};  // namespace src::control::flywheel

}  // namespace src::control::flywheel

#endif