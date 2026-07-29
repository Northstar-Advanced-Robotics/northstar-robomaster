#pragma once

#include <array>

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else 
#include "tap/motor/dji_motor.hpp"
#endif

// class Drivers;

namespace HardwareTesting
{
///
/// @brief This subsystem encapsulates four motors that control the chassis.
///
class TestSubsystem : public tap::control::Subsystem
{
public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    TestSubsystem(tap::Drivers* drivers);

    ///
    /// @brief Initializes the drive motors.
    ///
    void initialize() override;

    ///
    /// @brief Runs velocity PID controllers for the drive motors.
    ///
    void refresh() override;

    const char* getName() const override { return "SubsystemTester"; }

private:
    Motor debugPitchMotor;

};  // class ChassisSubsystem
}  // namespace Communications::Rev
