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
/// @ingroup util
///
/// @brief Bench-test subsystem wrapping a single debug motor, for bringing up hardware in
/// isolation.
///
/// @warning Despite the "chassis" wording this comment used to carry, it owns exactly one motor
/// (`debugPitchMotor` on MOTOR5/CAN_BUS2) and drives nothing chassis-related.
///
class TestSubsystem : public tap::control::Subsystem
{
public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    ///
    /// @brief Constructs the subsystem around the debug pitch motor.
    ///
    /// @param drivers A pointer to the global drivers object.
    ///
    TestSubsystem(tap::Drivers* drivers);

    ///
    /// @brief Initializes the debug motor.
    ///
    void initialize() override;

    ///
    /// @brief Called each control loop iteration by the scheduler.
    ///
    /// @warning Currently a no-op in effect: it reads the encoder into an unused local and writes
    /// no motor output. There is no PID controller in this class.
    ///
    void refresh() override;

    ///
    /// @return The intended name for this subsystem.
    ///
    /// @warning Missing `const` and `override`, so this does **not** override
    /// `tap::control::Subsystem::getName() const`. The scheduler never calls it and reports the
    /// base class name instead.
    ///
    const char* getName() { return "SubsystemTester"; }

private:
    /// The motor driven while bringing up new hardware.
    Motor debugPitchMotor;

};  // class TestSubsystem
}  // namespace HardwareTesting
