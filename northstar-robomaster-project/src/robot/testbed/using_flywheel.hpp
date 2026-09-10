#ifdef USING_FLYWHEEL

#ifndef USING_FLYWHEEL_HPP_
#define USING_FLYWHEEL_HPP_

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::flywheel;

FlywheelSubsystem flywheel(
    drivers(),
    LEFT_MOTOR_ID,
    RIGHT_MOTOR_ID,
    UP_MOTOR_ID,
    CAN_BUS,
    tap::motor::RevMotor::PIDConfig{
        .PIDSlot = 0,
        .kP = FLYWHEEL_PID_KP,
        .kI = FLYWHEEL_PID_KI,
        .kD = FLYWHEEL_PID_KD,
        .kF = FLYWHEEL_PID_KF,
    });

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
Trigger fPressedRunFlywheel =
    TriggerHelpers::button(drivers(), tap::communication::serial::Remote::Key::F)
        .toggleOnTrue(&flywheelRunCommand);
#endif

#endif  // USING_FLYWHEEL_HPP_