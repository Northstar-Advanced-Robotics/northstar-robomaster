#ifndef ELEVATOR_MOTOR_HPP
#define ELEVATOR_MOTOR_HPP
#include "control/elevator/elevator_config.hpp"

namespace src::control::elevator
{

class ElevatorMotor
{
public:
    void initialize();

    /// Samples the encoder and caches the reading. Call once per control loop iteration, before
    /// reading getHeight() / getVelocity().
    void updateMotorAngle();

    void setMotorOutput(float out);

    // TODO(PID step 2): report the carriage position in meters so the PID works in real units.
    //
    //   float getHeight() const;
    //       (unwrapped encoder position - zeroOffset) * config.metersPerMotorRad
    //       Unwrapped position: motor.getEncoder()->getPosition().getUnwrappedValue()
    //       Use the UNWRAPPED value: the motor turns many times over the full travel.
    //
    //   float getVelocity() const;
    //       motor.getEncoder()->getVelocity() * config.metersPerMotorRad   (m/s)
    //
    //   void zeroHere();      // zeroOffset = current unwrapped position; homed = true
    //   bool isHomed() const; // return !needsHoming() || homed;
    //
    //   private:
    //   float zeroOffset = 0.0f;  // encoder reading at the bottom, set by the home command
    //   bool homed = false;       // set by zeroHere(). Don't run the PID until isHomed() is true.
    //
    // HOMING: what it is and why this class decides it.
    //   Homing = drive the carriage to a known spot (the bottom hard stop or a limit switch) and
    //   say "this encoder reading is 0 m". It's needed when the encoder can't tell where the
    //   carriage is at power-up. An M3508's encoder is on the motor shaft, before the gearbox, and
    //   the lift takes many motor turns. At power-up it knows its angle within one turn but not
    //   WHICH turn, so a carriage left at 0.5 m reads as 0 m. "Go to 1.0 m" would then drive 1.0 m
    //   from the real position and crash into the top.
    //
    //   Whether homing is needed depends on the motor or sensor, so each motor implementation
    //   says so itself:
    //
    //   virtual bool needsHoming() const = 0;
    //       true  -> relative encoder (M3508, M2006): start un-homed and run the home command.
    //       false -> absolute position at power-up (single-turn GM6020 over the whole travel, a
    //                Damiao dual encoder, an external absolute encoder or string pot):
    //                isHomed() is true immediately and the home command finishes right away.
    //                zeroOffset is then a fixed calibration constant from config, not measured.
    //
    //   To force homing off during bench testing (e.g. no hard stop built yet), add a config flag
    //   `bool homingEnabled = true;`. When it's false, treat the motor as homed, with the carriage's
    //   power-up position as 0 m. In that case, ALWAYS power on with the carriage at the bottom.

    ElevatorConfig& config;
};

}  // namespace src::control::elevator

#endif  // ELEVATOR_MOTOR_HPP