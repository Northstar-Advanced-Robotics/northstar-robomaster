#include "test_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

//#define THING

#include <cmath>

namespace src::control::test_subsystem
{
// STEP 1 (Tank Drive): create constructor
TestSubsystem::TestSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      debugPitchMotor(
          drivers,
          tap::motor::MOTOR5,
          tap::can::CanBus::CAN_BUS2,
          false,
          "SingleRevMotor")
{
}

// STEP 2 (Tank Drive): initialize function
void TestSubsystem::initialize()
{
    debugPitchMotor.initialize();

    // motor1.setTargetVoltage(0.1f); //causing error bacause setTargetVoltage dosen't exist
}

// void RevMotorTester::initialize()
// {
//     motor1.initialize();
//     motor2.initialize();
//     motor3.initialize();
//     // motor1.setTargetVoltage(0.1f); //causing error bacause setTargetVoltage dosen't exist
//     motor1.setControlValue(0.25f);
//     motor2.setControlValue(0.25f);
//     motor3.setControlValue(0.25f);
// }

void TestSubsystem::refresh()
{
    [[maybe_unused]] double fuckYOU =
        debugPitchMotor.getInternalEncoder().getPosition().getUnwrappedValue();
}
// STEP
}  // namespace src::control::test_subsystem
