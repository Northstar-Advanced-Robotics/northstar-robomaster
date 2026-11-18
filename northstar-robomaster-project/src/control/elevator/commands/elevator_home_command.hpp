// TODO(PID step 5a): ElevatorHomeCommand. Finds the bottom and sets it as 0 m.
//
// Why: see the HOMING note in motors/elevator_motor.hpp. In short, an M3508 doesn't know the
// carriage height at power-up, so the PID can't run until we've touched a known spot.
//
// How it should work:
//   initialize():
//     - If !motor->needsHoming() (or homing is disabled in config), do nothing. isFinished()
//       returns true right away.
//   execute(), every tick:
//     - Drive DOWN with a small fixed output: elevator->setHomingOutput(HOMING_OUTPUT).
//       The PID can't be used because there's no valid height yet. Don't call setMotorOutput
//       directly: subsystem refresh() runs after execute() and would overwrite it.
//       Gravity helps going down, so keep it small (start around -1500 on a C620, flipped if
//       the motor is reversed).
//     - If |velocity| < a small threshold, add dt to a stall timer. Otherwise reset the timer.
//   isFinished():
//     - stall timer >= ~150 ms (we're sitting on the hard stop), OR a bottom limit switch is
//       pressed, if you have one (more reliable), OR a ~3 s timeout (something is wrong).
//   end(interrupted):
//     - elevator->setHomingOutput(0).
//     - Only if it finished normally (not interrupted, not timed out): motor->zeroHere().
//     - Then setTargetPosition(0) (or a few mm up) so the hold loop takes over smoothly.
//
// Schedule it once at startup, before any preset/jog/hold command can move the elevator.
// While !isHomed(), refresh() outputs 0, so the other commands can't do damage.
