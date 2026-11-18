#ifndef ELEVATOR_CONFIG_HPP
#define ELEVATOR_CONFIG_HPP

namespace src::control::elevator
{

struct ElevatorConfig
{
    double maxHeight;     // Maximum height of the elevator in meters
    double minHeight;     // Minimum height of the elevator in meters
    double maxSpeed;      // Maximum speed of the elevator in meters per second
    double acceleration;  // Acceleration of the elevator in meters per second squared
    double deceleration;  // Deceleration of the elevator in meters per second squared

    // TODO(PID step 1): add the fields the height loop needs.
    //   (needs #include "tap/algorithms/smooth_pid.hpp")
    //
    //   tap::algorithms::SmoothPidConfig pidConfig;
    //       kp / ki / kd / maxOutput for the height PID. Error is in METERS, output is raw motor
    //       units (C620: +-16384). Start with kp only, ki = kd = 0, and a LOW maxOutput (~3000)
    //       for the first bench test.
    //
    //   float kG;
    //       Constant output that holds the carriage still with the PID turned off (gravity comp).
    //       Same units as the motor output. Find it by tuning (see step 5). Its sign depends on
    //       which way the motor is mounted.
    //
    //   float metersPerMotorRad;
    //       Converts motor encoder radians to carriage meters = drum (or pulley) radius / gear
    //       ratio. Check it by moving the carriage a measured distance and comparing.
};

}  // namespace src::control::elevator
#endif  // ELEVATOR_CONFIG_HPP