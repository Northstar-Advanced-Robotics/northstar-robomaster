/**
 * Declares the flywheels: the subsystem, its run command, and the remote mapping that spins it up.
 *
 * `testbed_control.cpp` includes this file unconditionally; the `#ifdef USING_FLYWHEEL` below is
 * what actually selects it, and that switch is set in `test_def.hpp`.
 *
 * Everything is declared at file scope, which is why exactly one translation unit may include it.
 */
#ifdef USING_FLYWHEEL

#ifndef USING_FLYWHEEL_HPP_
#define USING_FLYWHEEL_HPP_

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::flywheel;

DJIThreeFlywheelSubsystem flywheel(
    drivers(),

);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
Trigger fPressedRunFlywheel =
    TriggerHelpers::button(drivers(), tap::communication::serial::Remote::Key::F)
        .toggleOnTrue(&flywheelRunCommand);
#endif

#endif  // USING_FLYWHEEL_HPP_