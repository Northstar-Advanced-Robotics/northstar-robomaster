#ifndef TURRET_MOTOR_MOCK_HPP_
#define TURRET_MOTOR_MOCK_HPP_

#include <gmock/gmock.h>

#include "control/turret/turret_motor.hpp"

namespace src::mock
{
/**
 * Mock of the abstract TurretMotor, used by the TurretSubsystem when building
 * for the unit test environment.
 */
class TurretMotorMock : public control::turret::TurretMotor
{
public:
    TurretMotorMock()
    {
        ON_CALL(*this, getChassisFrameMeasuredAngle)
            .WillByDefault(testing::ReturnRef(defaultAngle));
    }

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, updateMotorAngle, (), (override));
    MOCK_METHOD(void, setMotorOutput, (float), (override));
    MOCK_METHOD(
        void,
        attachTurretController,
        (const control::turret::algorithms::TurretControllerInterface *),
        (override));
    MOCK_METHOD(void, setChassisFrameSetpoint, (tap::algorithms::WrappedFloat), (override));
    MOCK_METHOD(bool, isOnline, (), (const, override));
    MOCK_METHOD(tap::algorithms::WrappedFloat, getChassisFrameSetpoint, (), (const, override));
    MOCK_METHOD(
        const tap::algorithms::WrappedFloat &,
        getChassisFrameMeasuredAngle,
        (),
        (const, override));
    MOCK_METHOD(float, getChassisFrameVelocity, (), (const, override));
    MOCK_METHOD(
        const control::turret::algorithms::TurretControllerInterface *,
        getTurretController,
        (),
        (const, override));
    MOCK_METHOD(float, getValidChassisMeasurementError, (), (const, override));
    MOCK_METHOD(
        float,
        getValidMinError,
        (const tap::algorithms::WrappedFloat, const tap::algorithms::WrappedFloat),
        (const, override));
    MOCK_METHOD(int16_t, getMotorOutput, (), (const, override));
    MOCK_METHOD(const control::turret::TurretMotorConfig &, getConfig, (), (const, override));

private:
    tap::algorithms::WrappedFloat defaultAngle{0, 0, M_TWOPI};
};

}  // namespace src::mock

#endif  // TURRET_MOTOR_MOCK_HPP_
