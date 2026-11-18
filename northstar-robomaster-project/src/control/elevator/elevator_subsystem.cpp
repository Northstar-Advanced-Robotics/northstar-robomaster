#include "elevator_subsystem.hpp"

namespace src::control::elevator
{
ElevatorSubsystem::ElevatorSubsystem(
    tap::Drivers* drivers,
    const ElevatorConfig& config,
    ElevatorMotor* elevator_motor)
    : tap::control::Subsystem(drivers),
      elevator_motor(elevator_motor)
{
}

void ElevatorSubsystem::initialize() { elevator_motor->initialize(); }

void ElevatorSubsystem::refresh()
{
    elevator_motor->updateMotorAngle();

    // TODO(PID step 4): run the height loop here, every tick.
    //
    //   1. Not homed? Send the homing output (0 unless the home command set one) and stop. The
    //      height is meaningless until homing finishes. NOTE: Taproot runs command execute()
    //      BEFORE subsystem refresh(), so the home command must NOT call setMotorOutput itself
    //      (this line would overwrite it). It calls setHomingOutput(), which stores homingOutput.
    //        if (!elevator_motor->isHomed()) { elevator_motor->setMotorOutput(homingOutput); return; }
    //
    //   2. dt since the last tick (needs #include "tap/architecture/clock.hpp"):
    //        uint32_t now = tap::arch::clock::getTimeMilliseconds();
    //        float dt = now - prevTimeMs;
    //        prevTimeMs = now;
    //
    //   3. Error in meters:
    //        float error = targetPosition - elevator_motor->getHeight();
    //
    //   4. PID. Pass -velocity as the derivative: at a fixed target, d(error)/dt = -velocity, and
    //      it's much less noisy than differentiating the error yourself.
    //        float out = heightPid.runController(error, -elevator_motor->getVelocity(), dt);
    //
    //   5. Gravity comp. Add it AFTER the PID so the PID's maxOutput clamp doesn't cut it off:
    //        out += kG;
    //
    //   6. Clamp to what the motor accepts and send it:
    //        out = tap::algorithms::limitVal(out, -MAX_OUTPUT, MAX_OUTPUT);  // C620: 16384
    //        elevator_motor->setMotorOutput(out);
}

void ElevatorSubsystem::refreshSafeDisconnect() { elevator_motor->setMotorOutput(0.0f); }

// The ONLY way commands move the elevator: set a target height (meters), and refresh() does the rest.
void ElevatorSubsystem::setTargetPosition(float position)
{
    targetPosition = position;
    if (position > elevator_motor->config.maxHeight)
        targetPosition = elevator_motor->config.maxHeight;
    if (position < elevator_motor->config.minHeight)
        targetPosition = elevator_motor->config.minHeight;
}

// TODO(PID step 5): commands, then tuning.
//
// Commands (commands/ folder). Each one only calls setTargetPosition():
//   home:   see commands/elevator_home_command.hpp. Skipped if the motor doesn't need homing.
//   preset: setTargetPosition(presetHeight) in initialize(). Finished when |error| < tolerance.
//   hold:   setTargetPosition(current height) once in initialize(). This is the default command.
//   jog:    each tick, targetPosition += joystick * maxSpeed * dt.
//
// Tuning order (motor on a stand, hand on the e-stop, low maxOutput at first):
//   1. kp = ki = kd = 0. Raise kG until the carriage stays still at mid-height.
//      Slides down = kG too low. Creeps up = too high. If raising kG makes it fall faster, flip the sign.
//   2. Raise kp until it reaches targets briskly with a little overshoot.
//   3. Raise kd until the overshoot / bouncing goes away (too much kd = buzzing).
//   4. Only add a small ki (and set maxICumulative) if it consistently stops short of the target.
//   5. Raise maxOutput to the real limit once it behaves.

}  // namespace src::control::elevator